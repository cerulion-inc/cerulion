#!/usr/bin/env python3
"""Cross-check that bench.py::read_p50_ns and compile_csv.py::percentile /
summarize produce byte-identical p50s on the same fixture vectors, and that
the A1 cross-rep aggregation (compile_csv.aggregate_reps: median-of-rep-p50s
headline + min/max spread; bench.median_int for the A4 smoke baseline)
matches hand-written oracles — a single rep must aggregate byte-identically
to the plain summarize path, so legacy artifacts cannot drift.

Run: python3 benches/latency/check_percentile_parity.py
Exit 0 = pass; non-zero = divergence detected (or, for the wedged-peer
case described below, a run that could not take its lock).

ONE AT A TIME per checkout. Several arms write their fixtures into the
real tree on purpose — the CERULION= guard reads the bench workspace's
own `target/debug`, so the arm that drives it must put a file there —
which makes two concurrent runs from one directory race. The script
therefore takes an exclusive advisory lock for its whole run (system
temp dir, keyed by this directory: different checkouts still run in
parallel) and says so on stderr while it waits. If the lock cannot be
taken at all -- no `fcntl`, a filesystem whose `flock` does not work,
something else sitting on the lock path -- it REFUSES rather than racing;
`--no-lock` says you have serialized the runs yourself.

Unlike the earlier version (which carried inline COPIES of both
implementations and relied on humans keeping three code paths in sync),
this script imports the two REAL implementations from the sibling modules
and drives bench.read_p50_ns through an actual temp .bin file — so the
parity it reports is the parity the smoke gate and the sweep actually have,
and an edit to either implementation is tested directly, not via a stale
mirror.

Stdlib only — runnable on any host with no extra dependencies, as a gates
check before committing changes to either percentile path. (plot.py
imports matplotlib LAZILY, so the plot arms' stdlib halves always run and
only the render half skips loudly where matplotlib is absent — pinned by
check_plot_arms_skip_without_matplotlib; the run_bench.sh arm drives the
real bash script, which needs no ROS.)
"""

import argparse
import ast
import collections
import contextlib
import html
import inspect
import io
import json
import os
import platform
import re
import shutil
import stat
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# The login gate is on in every build, and the arms below run the `cerulion`
# binary directly, so they do not pick up the workspace cargo configuration.
# Set here, in the process environment, because the call sites inherit it
# either verbatim or through a dict built from `os.environ`. This is how this
# repository's own runs pass the gate without an account.
os.environ["CERULION_LOGIN_GATE"] = "off"


# ---------------------------------------------------------------------------
# ONE checker at a time, per checkout.
#
# Two instances of this script run from one directory are NOT independent,
# and the collision is not the module globals the finding named — those are
# per-process. What they share is the CHECKOUT. `check_cerulion_binary_
# freshness` deliberately writes its fixtures into the REAL bench workspace
# (`workspace/target/debug/cerulion` and a `cerulion_link` symlink beside
# it), because the guard under test reads that exact directory; two runs
# then race on creating and unlinking the same two paths: without the lock,
# a concurrent run dies with
#     FileExistsError: [Errno 17] File exists: '.../cerulion_link'
# — a traceback, not a FAIL line, so it does not even report as a
# divergence. The repo-tree `__pycache__/` the finding also names is
# written through CPython's tmp-file+rename, so it is safe on its own; the
# lock covers it anyway by being taken BEFORE the sibling imports.
#
# So: an exclusive advisory lock, held for the whole process. Keyed by a
# hash of THIS script's resolved directory, so two different checkouts
# still run in parallel, and living in the system temp dir so the checkout
# gains no new file. A waiting run says so, loudly, on stderr.
#
# Acquired at module scope and only under `__main__`, which is the ONE
# thing that has to happen before `import bench` — importing this module
# for its helpers (the self-test children below do exactly that) must not
# block on a lock the parent already holds.
try:
    import fcntl                       # noqa: E402  (POSIX advisory locks)
except ImportError:                    # pragma: no cover — non-POSIX host
    fcntl = None                       # type: ignore[assignment]
import errno                           # noqa: E402
import hashlib                         # noqa: E402

# How long a second instance waits before giving up. ~17x a normal run, so
# reaching it means a peer is wedged, not merely slow.
RUN_LOCK_TIMEOUT_S = 600.0
RUN_LOCK_POLL_S = 0.25

# The acquirer's OWN keeper. `flock` is released when the last descriptor
# closes, so the handle has to outlive the call — and making that the
# CALLER's job meant an ordinary "this module global is never read" cleanup
# of the `_RUN_LOCK =` binding would silently disarm the lock while both
# self-test arms still printed ok. The lock now keeps
# itself; the return value is for the tests and the children.
_RUN_LOCK_HELD: list = []

# errnos that mean SOMEONE ELSE HOLDS IT. Anything else out of `flock`
# (ENOLCK, EOPNOTSUPP on some network/overlay mounts, EBADF, EINVAL) means
# locking does not WORK here, which is the `fcntl is None` case wearing a
# different hat — polling it for ten minutes and then blaming a holder that
# does not exist is the wrong answer twice over.
_LOCK_CONTENDED_ERRNOS = (errno.EWOULDBLOCK, errno.EAGAIN, errno.EACCES)


def _run_lock_path() -> Path:
    """This checkout's lock file: a hash of THIS script's resolved
    directory, under the system temp dir.

    Keyed by directory so two different checkouts still run in parallel;
    in the temp dir so the checkout gains no new file. RESIDUAL, stated
    because it is real: `gettempdir()` honours `$TMPDIR`, so two runs of
    one checkout under different `TMPDIR`s would take two different locks
    and not exclude each other. The path is printed whenever a run waits,
    which is where a mismatch would show."""
    key = hashlib.sha256(
        str(Path(__file__).resolve().parent).encode("utf-8")).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / f"parity_checker_{key}.lock"


# Why this run holds no lock, when it holds none. Read by the self-test:
# a host whose filesystem cannot `flock` is a SKIP, and reporting it as
# "the acquisition went inert" would send someone hunting a code defect
# that is not there.
_RUN_LOCK_UNAVAILABLE = ""


# Wide enough for any pid, and fixed so a holder line is overwritten in
# place rather than truncated. `read(200)` covers it with room to spare.
_LOCK_RECORD_WIDTH = 64


def _open_lock_file(path: Path):
    """Open (creating) the lock file, or `(None, reason)`.

    NEVER follows a symlink at the final component, and never truncates.
    The lock name is DERIVED — a hash of this script's directory under
    `gettempdir()` — so it is predictable, and the temp dir is world
    writable: any local user can pre-plant a symlink there, and the
    plain `open(path, "a+")` this replaces followed it and then wrote a
    pid into whatever it pointed at. MEASURED on the pre-fix code: a
    symlink at the lock path left the victim file holding `pid 60471` and
    nothing else.

    `O_NOFOLLOW` turns that into an `ELOOP` refusal. Its scope is the FINAL
    component, which is the whole of the exposure here: the parent
    directories are the system temp dir and its own parents, and on macOS
    `/var` really is a symlink to `/private/var`, so refusing on those
    would refuse every run on the platform this is developed on. An
    attacker who can replace a parent of the system temp directory has
    already won something much larger than this gate.

    What refuses a FIFO is the `S_ISREG` check, not `O_NONBLOCK`:
    MEASURED, `os.open(fifo, O_RDWR)` does not block here and the reason
    returned is literally "is not a regular file". `O_NONBLOCK` stays
    because `O_RDWR` on a FIFO is not defined by POSIX and a platform that
    DID block there would hang the gate rather than fail it. `S_ISREG`
    also refuses every other file type the path could hold — a directory,
    a device. `0o600` on create, because a lock file has no reason to be
    readable by anyone else.

    Mirrors the same class closed in `cerulion_cli_engine`'s
    `WorkspaceLock`."""
    # REQUIRED, not best-effort. `getattr(os, "O_NOFOLLOW", 0)` reads like
    # a portability courtesy and is really a silent downgrade: on a
    # platform without it the open would follow the planted symlink again,
    # with nothing in the output to say the defence was not applied. Every
    # platform that has `fcntl` (checked above, before this is reached) has
    # O_NOFOLLOW, so this is unreachable in practice -- which is exactly
    # why it must refuse rather than carry an untested fallback.
    if not hasattr(os, "O_NOFOLLOW"):
        return None, ("this platform has no `O_NOFOLLOW`, so the lock file "
                      "cannot be opened without following a symlink at its "
                      "path")
    # `O_NONBLOCK` keeps its `getattr` fallback, and the difference from
    # O_NOFOLLOW above is the BLAST RADIUS, not the style: missing
    # O_NOFOLLOW silently reopens a symlink-following write to an
    # arbitrary file, while missing O_NONBLOCK can at worst hang a gate on
    # a platform where `O_RDWR` on a FIFO blocks -- loud by being stuck,
    # not silent.
    flags = (os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW
             | getattr(os, "O_NONBLOCK", 0))
    try:
        fd = os.open(path, flags, 0o600)
    except OSError as e:
        # Reachable without an attacker, too: one `sudo` run leaves a
        # root-owned lock file that every later run from this checkout
        # cannot open.
        return None, f"cannot open its lock file {path} ({e})"
    # `closed` is the fd's single owner: every `return` below leaves the
    # `try`, so it reaches neither the `except` nor the `else`, and three
    # refusal paths leaked a descriptor each (MEASURED: 5 over 5 FIFO
    # refusals, and arm (c3)'s FIFO plant leaks one per suite run).
    handle = None
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode):
            return None, (f"its lock path {path} is not a regular file — "
                          f"something else is sitting on it")
        # A HARD LINK is a regular file, so `O_NOFOLLOW` and the check
        # above both pass it. Same class, same blast radius, `link()`
        # instead of `symlink()` — MEASURED: a hard link at the lock path
        # was accepted and the victim's first 64 bytes became the holder
        # record. macOS, the primary dev platform, has no
        # `protected_hardlinks`, so any local user can link any path the
        # runner can resolve into this predictable name. A lock file this
        # function created has exactly one link, so this never refuses a
        # genuine one.
        if st.st_nlink != 1:
            return None, (f"its lock path {path} has {st.st_nlink} links — "
                          f"it is hard-linked to something else")
        if st.st_uid != os.getuid():
            return None, (f"its lock path {path} is owned by uid "
                          f"{st.st_uid}, not this user")
        # `O_CREAT`'s mode applies only when the file is CREATED, so a
        # pre-existing lock file keeps whatever mode it had — 0666 from a
        # careless umask, or from someone who planted it.
        if stat.S_IMODE(st.st_mode) & 0o077:
            os.fchmod(fd, 0o600)
        # `fdopen` takes ownership of the descriptor on success; if it
        # raises, it does not, so only these paths close it. The except is
        # NOT `OSError` alone: `fdopen` in TEXT mode resolves an encoding
        # and raises `LookupError` on a broken `LC_ALL`/`PYTHONIOENCODING`,
        # which would escape this function as a traceback and leak the fd —
        # the one path that would not honour the loud-refusal contract.
        handle = os.fdopen(fd, "r+")
    except (OSError, ValueError, LookupError) as e:
        return None, f"cannot use its lock file {path} ({e})"
    finally:
        # `fdopen` takes ownership on success and NOT on failure, so the
        # descriptor is closed here exactly when no handle owns it —
        # covering the guard returns, both exception classes, and an
        # exception this function does not model.
        if handle is None:
            os.close(fd)
    return handle, None


def _lock_unavailable(reason: str):
    """Record and announce that NO lock was taken, and return None.

    Whether the run then proceeds is the CALLER's decision
    (`_lock_gate_decision`), not this function's. It used to say "then
    run", and its note used to add "run them one at a time" — which, once
    the gate started refusing, read to an operator as `this run is
    proceeding unserialized`, the exact inference the refusal exists to
    prevent, and stated the path and reason twice over."""
    global _RUN_LOCK_UNAVAILABLE
    _RUN_LOCK_UNAVAILABLE = reason
    print(f"note  [parity checker: no lock taken — {reason}]",
          file=sys.stderr)
    return None


def _acquire_run_lock(path: "Path" = None,
                      timeout_s: float = RUN_LOCK_TIMEOUT_S,
                      poll_s: float = RUN_LOCK_POLL_S):
    """Hold this checkout's checker lock for the life of the process.

    The handle is retained in `_RUN_LOCK_HELD` and also returned, so a
    caller that discards the return value still holds the lock. Returns
    None when no lock was taken — always after saying why.

    Polled rather than a blocking `flock`, so the wait is bounded and a
    wedged peer is REPORTED instead of hanging a gate forever."""
    if path is None:
        path = _run_lock_path()
    if fcntl is None:
        return _lock_unavailable("no `fcntl` on this platform")
    handle, why = _open_lock_file(path)
    if handle is None:
        return _lock_unavailable(why)
    deadline = time.monotonic() + timeout_s
    announced = False
    blocked = 0
    while True:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            break
        except OSError as e:
            if e.errno not in _LOCK_CONTENDED_ERRNOS:
                handle.close()
                return _lock_unavailable(
                    f"`flock` on {path} is unavailable ({e})")
            if time.monotonic() >= deadline:
                handle.close()
                raise SystemExit(
                    f"parity checker: waited {timeout_s:.0f}s for {path} and "
                    f"the holder never released it. Read the pid from that "
                    f"file and confirm it is gone (`kill -0 <pid>`); a lock "
                    f"releases itself when its holder exits. Do NOT delete "
                    f"the file — a fresh inode would let both runs proceed.")
            time.sleep(poll_s)
            blocked += 1
            # Not on the FIRST block: the holder records its pid only after
            # it has the lock, so a waiter that reads immediately reports
            # "unknown" for a holder that is perfectly identifiable a
            # moment later.
            if not announced and blocked >= 2:
                handle.seek(0)
                # Bounded to the record this file writes. An unbounded read
                # echoed whatever a pre-existing lock file happened to
                # contain straight into stderr, and from there into CI
                # logs — MEASURED with a planted file whose tail was
                # printed verbatim in this note.
                holder = handle.read(_LOCK_RECORD_WIDTH).strip() or "unknown"
                print(f"note  [parity checker: another instance holds "
                      f"{path} ({holder}) — waiting up to "
                      f"{timeout_s:.0f}s. This checker writes fixtures into "
                      f"the real bench workspace, so runs from one checkout "
                      f"are serialized.]", file=sys.stderr)
                announced = True
    # A FIXED-WIDTH record, so the holder line is overwritten IN PLACE.
    # `truncate()` is the one call here that could destroy CONTENT if the
    # path ever turned out not to be ours, and the hard-link case is why
    # that is not hypothetical: a hard link IS a regular file, so it is
    # `st_nlink` above -- one check, one revert away -- that stands between
    # this write and a victim file. Truncating zeroes such a victim
    # entirely; a fixed-width write damages its first 64 bytes. The
    # `r+` mode is load-bearing for this and pinned by its own arm: `a+`
    # carries O_APPEND, so `seek(0); write(...)` would append instead, the
    # file would grow by a record per acquisition, and the holder note
    # would name the FIRST, long-dead holder forever.
    handle.seek(0)
    handle.write(f"pid {os.getpid()}".ljust(_LOCK_RECORD_WIDTH - 1) + "\n")
    handle.flush()
    _RUN_LOCK_HELD.append(handle)
    return handle


# The one way to run WITHOUT the lock, and it has to be asked for.
NO_LOCK_FLAG = "--no-lock"

# `main()` returns the error COUNT on its normal path, so a usage error
# must not be spelled as a small integer a real run could produce: exit 2
# would be indistinguishable from two failing checks. 64 is the
# conventional EX_USAGE and is out of reach of any plausible count.
_EX_USAGE = 64

# Set by the end-to-end gate arm on the children it spawns, so a child can
# never spawn children of its own.
_GATE_CHILD_ENV = "CER_PARITY_GATE_CHILD"


def _lock_gate_decision(acquired: bool, override: bool) -> str:
    """PURE: what a run does about the lock. "run" / "run-unlocked" /
    "refuse".

    FAIL CLOSED. Warning and continuing was the old behaviour and it
    defeats the lock exactly where the lock matters: on a platform or
    filesystem that cannot `flock`, two runs would both print the warning
    and then both walk into the checks that create and delete fixtures in
    the real bench workspace. An unserialized run is a choice, so it is
    spelled out on the command line."""
    if acquired:
        return "run"
    return "run-unlocked" if override else "refuse"


# Module scope, before the sibling imports, and ONLY as a script. Pinned by
# check_run_lock_serializes_one_checkout, which asserts the position, the
# path, that THIS process really holds that path, and the exclusion.
_OVERRIDE = NO_LOCK_FLAG in sys.argv[1:]
if __name__ == "__main__" and _OVERRIDE:
    # SKIP the acquisition, do not merely consult the flag afterwards. A
    # peer holding the lock sends the acquirer into its contended loop,
    # which waits RUN_LOCK_TIMEOUT_S and then raises from INSIDE the
    # acquirer — before the gate is ever reached. MEASURED: with a peer
    # holding, `--no-lock` waited for the release instead of bypassing it,
    # so the refusal's own advice ("or pass --no-lock") did not work in the
    # case that produces the refusal. It also left a pre-planted,
    # permanently-held lock file as an un-escapable stop.
    _RUN_LOCK_UNAVAILABLE = f"{NO_LOCK_FLAG} was passed"
    _RUN_LOCK = None
else:
    _RUN_LOCK = _acquire_run_lock() if __name__ == "__main__" else None
if __name__ == "__main__":
    _GATE = _lock_gate_decision(_RUN_LOCK is not None, _OVERRIDE)
    if _GATE == "refuse":
        raise SystemExit(
            f"parity checker: refusing to run without its lock "
            f"({_RUN_LOCK_UNAVAILABLE or 'the lock could not be taken'}). "
            f"Several checks create and delete fixtures inside the real "
            f"bench workspace, so two unserialized runs from one checkout "
            f"corrupt each other's results. Run them one at a time, or "
            f"pass {NO_LOCK_FLAG} to say you have.")
    if _GATE == "run-unlocked":
        print(f"note  [parity checker: running WITHOUT the lock because "
              f"{NO_LOCK_FLAG} was passed — nothing is stopping a second "
              f"instance from writing the same fixtures]", file=sys.stderr)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench          # noqa: E402  (real read_p50_ns)

# bench.REPO_ROOT, not a second derivation of it: two copies of one path
# is the drift this file exists to catch elsewhere.
REPO_ROOT = bench.REPO_ROOT
import compile_csv    # noqa: E402  (real percentile + summarize)
import usage_sampler  # noqa: E402  (real parse_pids_spec)


def _make_blob(values):
    """Pack a list of ints into a u64-LE binary blob (the .bin wire format)."""
    return b"".join(struct.pack("<Q", v) for v in values)


# ---------------------------------------------------------------------------
# Fixture vectors: even/odd lengths, uniform, ascending, descending, outlier,
# and the minimum-safe-N edge (n=2 → p50 interpolates between the two).
# ---------------------------------------------------------------------------
FIXTURES = [
    # (description, unsorted values)
    ("2 values — minimum safe N after warmup", [100, 200]),
    ("3 values — odd, median is exact", [10, 20, 30]),
    ("4 values — even, p50 interpolated", [10, 20, 30, 40]),
    ("10 values — larger even", list(range(10, 110, 10))),
    ("11 values — larger odd", list(range(100, 1200, 100))),
    ("descending input", [1000, 900, 800, 700, 600, 500]),
    ("all equal", [42] * 7),
    ("single outlier high", [100, 100, 100, 100, 9999]),
    ("large values (nanosecond scale)", [1_000_000, 2_000_000, 3_000_000, 4_000_000]),
]


# A child that reports whether it could take `flock` on a path, and a
# child that drives the real `_acquire_run_lock` against a held one. Both
# print ONE json line; neither sleeps, so nothing here is timed.
_LOCK_PROBE = r'''
import fcntl, json, sys
ok = True
err = ""
try:
    h = open(sys.argv[1], "a+")
    fcntl.flock(h.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError as e:
    ok, err = False, str(e)
print(json.dumps({"acquired": ok, "error": err}))
'''

# Acquires a lock path and prints the pid that now holds it. A CHILD,
# because the point is a holder with a different pid from this process's.
_LOCK_REACQUIRE = r'''
import os, sys
sys.path.insert(0, %(dirq)s)
import check_percentile_parity as c
from pathlib import Path
h = c._acquire_run_lock(Path(sys.argv[1]), timeout_s=10.0, poll_s=0.02)
if h is None:
    raise SystemExit("the child could not acquire")
print(os.getpid(), flush=True)
h.close()
'''

_LOCK_WAITER = r'''
import json, sys
sys.path.insert(0, %(dirq)s)
import check_percentile_parity as c
from pathlib import Path
try:
    h = c._acquire_run_lock(Path(sys.argv[1]), timeout_s=float(sys.argv[2]),
                            poll_s=0.02)
except SystemExit as e:
    print(json.dumps({"exited": True, "acquired": False, "message": str(e)}),
          flush=True)
else:
    print(json.dumps({"exited": False, "acquired": h is not None,
                      "message": ""}), flush=True)
'''

# How long the arm will wait for a BLOCKED child to print its holder note.
# Generous on purpose: it bounds a CONDITION (the note appearing), and the
# arm asserts nothing about how long it took. A 0.2s timeout budget whose
# note fires after two 20ms polls leaves only a ~20ms margin, MEASURED at 3
# false FAILs in 6 runs under `taskpolicy -b`.
_LOCK_NOTE_DEADLINE_S = 30.0


class _LockWaiter:
    """A spawned `_acquire_run_lock` child, with its stderr on disk so the
    parent can watch for the holder note without racing a pipe."""

    def __init__(self, proc, err_path, err_file):
        self.proc = proc
        self.err_path = err_path
        self.err_file = err_file

    def read_err(self) -> str:
        try:
            return self.err_path.read_text(errors="replace")
        except OSError:
            return ""

    def close(self) -> None:
        """Kill (if still running), reap, and close BOTH handles. Safe to
        call twice, and it never raises: it runs from `finally` blocks, and
        a `TimeoutExpired` there would mask the exception being handled."""
        try:
            if self.proc.poll() is None:
                self.proc.kill()
            try:
                self.proc.wait(timeout=30)
            except subprocess.SubprocessError:
                pass
        finally:
            for handle in (self.proc.stdout, self.err_file):
                try:
                    if handle is not None and not handle.closed:
                        handle.close()
                except OSError:
                    pass


def _spawn_lock_waiter(path: Path, timeout_s: float, td: str,
                       tag: str) -> "_LockWaiter":
    err_path = Path(td) / f"waiter-{tag}.err"
    err_file = err_path.open("wb")
    try:
        proc = subprocess.Popen(
            [sys.executable, "-c",
             _LOCK_WAITER % {"dirq": repr(str(Path(__file__).resolve().parent))},
             str(path), str(timeout_s)],
            stdout=subprocess.PIPE, stderr=err_file, text=True)
    except BaseException:
        err_file.close()          # a failed spawn must not orphan it
        raise
    return _LockWaiter(proc, err_path, err_file)


def _await_lock_note(waiter: "_LockWaiter", deadline_s: float):
    """The child's holder note, or None if it never printed one. Polls the
    file; the child is still running when this returns."""
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        text = waiter.read_err()
        if "another instance holds" in text:
            return text
        if waiter.proc.poll() is not None:
            return None                  # it exited without ever announcing
        time.sleep(0.05)
    return None


def _finish_lock_waiter(waiter: "_LockWaiter"):
    """The child's verdict, or None (having reported) if it printed none."""
    out = ""
    try:
        out, _ = waiter.proc.communicate(timeout=180)
    except subprocess.TimeoutExpired:
        print("FAIL  [checker run lock: a waiting child never finished]")
        return None
    finally:
        waiter.close()
    try:
        return json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        print(f"FAIL  [checker run lock: a waiting child printed no "
              f"verdict: {out.strip()[-200:]!r} / "
              f"{waiter.read_err().strip()[-200:]!r}]")
        return None


def _drive_lock_waiter(path: Path, timeout_s: float, td: str, tag: str):
    """Spawn a waiter and read its verdict. For the arms that expect it to
    finish on its own."""
    return _finish_lock_waiter(_spawn_lock_waiter(path, timeout_s, td, tag))


def _probe_lock(path: Path) -> dict:
    """Can a SEPARATE process take `path`? No walls: `LOCK_NB` answers
    immediately, so the verdict is a fact about the lock, never about how
    fast this box scheduled two interpreters."""
    r = subprocess.run([sys.executable, "-c", _LOCK_PROBE, str(path)],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0 or not r.stdout.strip():
        raise RuntimeError(f"lock probe rc={r.returncode}: "
                           f"{(r.stdout + r.stderr).strip()[-300:]}")
    return json.loads(r.stdout.strip().splitlines()[-1])


def check_run_lock_serializes_one_checkout() -> int:
    """Two checker runs from ONE directory must not interleave.

    Four claims. The first three are what make the fourth mean anything,
    and each fails against the mutant it names:

    (a) POSITION — the acquisition is a module-level statement that runs
        BEFORE `import bench`. Read off this file's own AST, because what
        makes the lock cover the whole run (the sibling imports and every
        arm that writes into the real tree) is WHERE it sits.

    (b) PATH — `_run_lock_path()` against a hand-recomputed oracle. What
        gets serialized is the whole property: a constant path would
        serialize unrelated checkouts, and a cwd-keyed one would stop
        excluding two runs of the SAME checkout started from different
        directories, which is exactly the race this exists to end.

    (c) HELD — a separate process must FAIL to take that path while this
        one runs. This is the claim the first draft could not make: the
        docstring said the caller must keep the handle, and a bare
        `_acquire_run_lock()` (an ordinary unused-variable cleanup)
        released it the instant the statement ended while (a) and the old
        exclusion arm both still printed ok.

    (d) EXCLUSION and the TIMEOUT — the parent takes a temp lock; a child
        must fail on it, then succeed once it is released; and a child
        driving the real `_acquire_run_lock` against the held one must
        EXIT rather than proceed unlocked, naming the path. No sleeps, no
        intervals: the old version compared two children's wall-clock
        hold windows and its control could false-FAIL on a loaded box for
        a 0.6 s scheduling skew."""
    errors = 0
    if fcntl is None:
        print("skip  [checker run lock: no `fcntl` on this platform]")
        return 0

    # ---- (a) position ----------------------------------------------------
    tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    acquire_at = None
    import_bench_at = None
    for node in tree.body:
        if import_bench_at is None and isinstance(node, ast.Import) and \
                any(alias.name == "bench" for alias in node.names):
            import_bench_at = node.lineno
        if acquire_at is not None or isinstance(
                node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        for sub in ast.walk(node):
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name) \
                    and sub.func.id == "_acquire_run_lock":
                acquire_at = node.lineno
                break
    if acquire_at is None:
        print("FAIL  [checker run lock: no module-level `_acquire_run_lock` "
              "call — the lock is defined and never taken, so every arm "
              "that writes into the real tree races again]")
        errors += 1
    elif import_bench_at is None:
        print("FAIL  [checker run lock: this file no longer imports `bench` "
              "at module scope, so the position assertion below cannot be "
              "made — read this before trusting the lock]")
        errors += 1
    elif acquire_at > import_bench_at:
        print(f"FAIL  [checker run lock: taken at line {acquire_at}, AFTER "
              f"`import bench` at line {import_bench_at}. The sibling "
              f"imports write this directory's __pycache__, so the lock "
              f"must come first]")
        errors += 1
    else:
        print(f"ok    [checker run lock: taken at module scope (line "
              f"{acquire_at}) before `import bench` (line "
              f"{import_bench_at})]")

    # ---- (b) the path, against a hand oracle ------------------------------
    want_key = hashlib.sha256(
        str(Path(__file__).resolve().parent).encode("utf-8")).hexdigest()[:16]
    want_path = Path(tempfile.gettempdir()) / f"parity_checker_{want_key}.lock"
    got_path = _run_lock_path()
    if got_path != want_path:
        print(f"FAIL  [checker run lock: the lock path is {got_path}, want "
              f"{want_path} — it must be keyed by THIS script's resolved "
              f"directory, so one checkout excludes itself and two "
              f"checkouts do not exclude each other]")
        errors += 1
    else:
        print(f"ok    [checker run lock: keyed by this script's resolved "
              f"directory ({got_path.name})]")

    # ---- (c) this process really holds it ---------------------------------
    # EITHER keeper counts: the acquirer retains its own handle, so a bare
    # `_acquire_run_lock()` (the unused-variable cleanup) still HOLDS the
    # lock and must still be probed.
    holds = _RUN_LOCK is not None or bool(_RUN_LOCK_HELD)
    if not holds and __name__ != "__main__":
        # The module was IMPORTED rather than run, which is how a developer
        # drives one arm. The acquisition is guarded by `__name__ ==
        # "__main__"`, so there is genuinely nothing to probe.
        print("skip  [checker run lock: this process holds no lock — the "
              "module was imported, not run]")
    elif _RUN_LOCK_UNAVAILABLE:
        # The acquirer tried and the PLATFORM refused (no `fcntl`, an
        # unopenable lock file, a filesystem whose `flock` does not work).
        # Not a defect in this code, and calling it one sends someone
        # hunting a bug that is not there.
        print(f"skip  [checker run lock: this run is NOT serialized — "
              f"{_RUN_LOCK_UNAVAILABLE}]")
    elif not holds:
        # Run as the SCRIPT, holding nothing, and the acquirer never said
        # why: the acquisition went inert. This is the arm that a `skip`
        # used to swallow.
        print("FAIL  [checker run lock: this run holds NO lock — the "
              "module-scope acquisition is present but inert, so every arm "
              "that writes into the real tree races a concurrent run again]")
        errors += 1
    else:
        try:
            held = _probe_lock(got_path)
        except (RuntimeError, subprocess.SubprocessError, ValueError,
                IndexError) as e:
            print(f"FAIL  [checker run lock: the HELD probe could not be "
                  f"driven: {e}]")
            return errors + 1
        if held["acquired"]:
            print(f"FAIL  [checker run lock: another process took "
                  f"{got_path} while this run is in progress — this run is "
                  f"NOT protected, so every arm that writes into the real "
                  f"tree races again]")
            errors += 1
        else:
            print(f"ok    [checker run lock: a separate process is refused "
                  f"this run's lock ({held['error']})]")

    # ---- (c2) the platform-refusal branches, errno-injected --------------
    # `_lock_unavailable` decides that a run is not serialized, and every
    # one of its callers is a platform condition this box does not have.
    # Injected rather than left unreached: the branch that distinguishes
    # "someone holds it" from "locking does not work here" is the one that
    # stops a 10-minute poll against a holder that does not exist.
    saved_flock = fcntl.flock
    saved_reason = _RUN_LOCK_UNAVAILABLE
    before_errno = errors
    # try/finally, because this block installs process-global state of its
    # own (`fcntl.flock`, and the module's refusal reason) and a plain
    # trailing statement is exactly the restore-skipped-on-a-throw shape,
    # sitting in the arm that tests it.
    try:
      with tempfile.TemporaryDirectory(prefix="lockerr_") as td:
          probe = Path(td) / "errno.lock"
          for label, err, needle in (
              ("a filesystem whose flock is unsupported", errno.EOPNOTSUPP,
               "unavailable"),
              ("no locks available", errno.ENOLCK, "unavailable"),
          ):
              def _raise(_fd, _op, _e=err):
                  raise OSError(_e, os.strerror(_e))
              try:
                  fcntl.flock = _raise
                  err_out = io.StringIO()
                  with contextlib.redirect_stderr(err_out):
                      got = _acquire_run_lock(probe, timeout_s=0.2, poll_s=0.02)
              finally:
                  fcntl.flock = saved_flock
              if got is not None:
                  print(f"FAIL  [checker run lock: {label} must yield NO lock, "
                        f"got {got!r}]")
                  errors += 1
              elif needle not in err_out.getvalue():
                  print(f"FAIL  [checker run lock: {label} must say so on "
                        f"stderr, got {err_out.getvalue().strip()!r}]")
                  errors += 1
              elif not _RUN_LOCK_UNAVAILABLE:
                  print(f"FAIL  [checker run lock: {label} must RECORD why, "
                        f"or the arm above reports a code defect for a "
                        f"platform limitation]")
                  errors += 1
          # An unopenable lock file is the other caller: a `sudo` run leaves a
          # root-owned one behind, and every later run from this checkout
          # must degrade rather than crash.
          blocked_dir = Path(td) / "nodir" / "x.lock"
          err_out = io.StringIO()
          with contextlib.redirect_stderr(err_out):
              got = _acquire_run_lock(blocked_dir, timeout_s=0.2, poll_s=0.02)
          if got is not None or "cannot open its lock file" not in \
                  err_out.getvalue():
              print(f"FAIL  [checker run lock: an unopenable lock file must "
                    f"degrade loudly, got {got!r} / "
                    f"{err_out.getvalue().strip()!r}]")
              errors += 1
    finally:
        fcntl.flock = saved_flock
        globals()["_RUN_LOCK_UNAVAILABLE"] = saved_reason
    if errors == before_errno:
        print("ok    [checker run lock: a platform that cannot lock "
              "degrades loudly and records why, rather than polling "
              "for a holder that does not exist]")

    # ---- (c3) the lock path is not a place to be led ---------------------
    # The lock NAME is derived -- a hash of this script's directory under a
    # world-writable temp dir -- so any local user can predict it and
    # pre-plant something there. MEASURED on the pre-fix code: a symlink at
    # the lock path was followed, and the victim file came back holding
    # `pid 60471` and nothing else.
    before_plant = errors
    # Each plant below goes through `_lock_unavailable`, which RECORDS the
    # reason in a module global that arm (c) reads. (c) happens to run
    # first today, so leaving it set is latent rather than live — and a
    # reordering would make (c) print `skip [... NOT serialized ...]` and
    # swallow exactly the inert-lock defect its own comment says a skip
    # used to swallow.
    saved_reason_plant = _RUN_LOCK_UNAVAILABLE
    try:
      with tempfile.TemporaryDirectory(prefix="plant_") as td:
          victim = Path(td) / "victim.txt"
          original = "PRECIOUS CONTENT THE OPERATOR CARES ABOUT\n" * 3
          victim.write_text(original, encoding="utf-8")
          planted = {"a symlink": Path(td) / "link.lock",
                     "a directory": Path(td) / "dir.lock",
                     "a FIFO": Path(td) / "fifo.lock",
                     "a hard link": Path(td) / "hard.lock"}
          planted["a symlink"].symlink_to(victim)
          planted["a directory"].mkdir()
          # A HARD LINK is a regular file, so O_NOFOLLOW and S_ISREG both
          # pass it: same class, same blast radius, `link()` instead of
          # `symlink()`. MEASURED on the pre-fix code — the victim's first
          # 64 bytes became the holder record.
          os.link(victim, planted["a hard link"])
          try:
              os.mkfifo(planted["a FIFO"])
          except (AttributeError, OSError):
              planted.pop("a FIFO")        # not every platform has them
          for what, where in planted.items():
              err_out = io.StringIO()
              with contextlib.redirect_stderr(err_out):
                  got = _acquire_run_lock(where, timeout_s=0.5, poll_s=0.02)
              if got is not None:
                  got.close()
                  if got in _RUN_LOCK_HELD:
                      _RUN_LOCK_HELD.remove(got)
                  print(f"FAIL  [checker run lock: {what} planted at the lock "
                        f"path was ACCEPTED — the lock name is predictable "
                        f"and the temp dir is world writable, so this hands a "
                        f"pid write to whoever planted it]")
                  errors += 1
              elif not err_out.getvalue().strip():
                  print(f"FAIL  [checker run lock: {what} at the lock path "
                        f"was refused SILENTLY — a run that is not serialized "
                        f"must say so]")
                  errors += 1
          # THE claim: the symlink's target is untouched. A refusal that
          # still truncated the victim on its way out would satisfy every
          # line above.
          if victim.read_text(encoding="utf-8") != original:
              print(f"FAIL  [checker run lock: the symlink's TARGET was "
                    f"modified — it now holds "
                    f"{victim.read_text(encoding='utf-8')[:40]!r}. Refusing "
                    f"after writing is not refusing]")
              errors += 1
          # The O_NOFOLLOW REQUIREMENT, driven by hiding the constant: a
          # platform without it must be refused, not silently served an open
          # that follows the very symlink this guards against.
          saved_nofollow = getattr(os, "O_NOFOLLOW", None)
          try:
              if saved_nofollow is None:
                  raise RuntimeError("no O_NOFOLLOW to hide")
              del os.O_NOFOLLOW
              err_out = io.StringIO()
              with contextlib.redirect_stderr(err_out):
                  got = _acquire_run_lock(Path(td) / "nofollowless.lock",
                                          timeout_s=0.5, poll_s=0.02)
              if got is not None:
                  got.close()
                  if got in _RUN_LOCK_HELD:
                      _RUN_LOCK_HELD.remove(got)
                  print("FAIL  [checker run lock: a platform with no "
                        "`O_NOFOLLOW` was served a lock anyway — the open "
                        "would follow a planted symlink and nothing would "
                        "say the defence was skipped]")
                  errors += 1
              elif "O_NOFOLLOW" not in err_out.getvalue():
                  print(f"FAIL  [checker run lock: refusing for want of "
                        f"`O_NOFOLLOW` must SAY so, got "
                        f"{err_out.getvalue().strip()[-160:]!r}]")
                  errors += 1
          except RuntimeError:
              print("skip  [checker run lock: this platform has no "
                    "`O_NOFOLLOW` to hide, so the requirement cannot be "
                    "driven here]")
          finally:
              if saved_nofollow is not None:
                  os.O_NOFOLLOW = saved_nofollow
          # DESCRIPTORS. Every refusal above leaves the `try` by `return`,
          # reaching neither the `except` nor the `else`, and three of them
          # leaked an fd each until the `finally` took ownership — one per
          # suite run, from this arm's own FIFO. Counted, because nothing
          # else in this file observes a descriptor.
          fd_probe = Path(td) / "fdprobe"
          fd_probe.mkdir()
          def _open_count() -> int:
              n = 0
              for candidate in range(3, 512):
                  try:
                      os.fstat(candidate)
                  except OSError:
                      continue
                  n += 1
              return n
          before_fds = _open_count()
          for where in list(planted.values()) * 4:
              with contextlib.redirect_stderr(io.StringIO()):
                  leaked = _acquire_run_lock(where, timeout_s=0.3,
                                             poll_s=0.02)
              if leaked is not None:     # never, given the arms above
                  leaked.close()
                  if leaked in _RUN_LOCK_HELD:
                      _RUN_LOCK_HELD.remove(leaked)
          grew = _open_count() - before_fds
          if grew > 0:
              print(f"FAIL  [checker run lock: {len(planted) * 4} refused "
                    f"acquisitions leaked {grew} descriptor(s) — a refusal "
                    f"that holds the file open is a refusal that still has "
                    f"it]")
              errors += 1
          # The OWNER guard. A box with one uid cannot plant a
          # foreign-owned file, so the uid is injected instead -- the same
          # shape as hiding `O_NOFOLLOW` above. What it stands in for is
          # real: a root-owned lock file left by one `sudo` run, or one
          # planted by another user.
          owned = Path(td) / "owned.lock"
          owned.write_text("", encoding="utf-8")
          saved_getuid = os.getuid
          try:
              os.getuid = lambda: saved_getuid() + 1
              err_out = io.StringIO()
              with contextlib.redirect_stderr(err_out):
                  got = _acquire_run_lock(owned, timeout_s=0.5, poll_s=0.02)
          finally:
              os.getuid = saved_getuid
          if got is not None:
              got.close()
              if got in _RUN_LOCK_HELD:
                  _RUN_LOCK_HELD.remove(got)
              print("FAIL  [checker run lock: a lock file owned by another "
                    "user was accepted — this run would write a pid into "
                    "somebody else's file and trust it as its own lock]")
              errors += 1
          elif "owned by uid" not in err_out.getvalue():
              print(f"FAIL  [checker run lock: refusing a foreign-owned "
                    f"lock file must say so, got "
                    f"{err_out.getvalue().strip()[-160:]!r}]")
              errors += 1
          # A pre-existing lock file keeps its own mode -- `O_CREAT`'s
          # applies only on creation -- so a 0666 one (a careless umask, or
          # a plant) must be tightened rather than used as found.
          loose = Path(td) / "loose.lock"
          loose.write_text("", encoding="utf-8")
          os.chmod(loose, 0o666)
          with contextlib.redirect_stderr(io.StringIO()):
              got = _acquire_run_lock(loose, timeout_s=5.0, poll_s=0.02)
          if got is None:
              print("FAIL  [checker run lock: a pre-existing lock file this "
                    "user owns was refused — the guards refuse too much]")
              errors += 1
          else:
              got.close()
              if got in _RUN_LOCK_HELD:
                  _RUN_LOCK_HELD.remove(got)
              if stat.S_IMODE(os.stat(loose).st_mode) & 0o077:
                  print(f"FAIL  [checker run lock: a pre-existing 0666 lock "
                        f"file stayed "
                        f"{stat.S_IMODE(os.stat(loose).st_mode):04o} — "
                        f"O_CREAT's mode does not apply to a file that "
                        f"already exists, so it has to be tightened here]")
                  errors += 1
          # The HOLDER RECORD names the CURRENT holder, on the SECOND
          # acquisition of one inode. That is the only shape that
          # separates `r+` from `a+`: `a+` carries O_APPEND, so
          # `seek(0); write(...)` appends instead, the file grows by a
          # record per acquisition, and the note names the FIRST,
          # long-dead holder forever -- while the refusal tells the
          # operator to `kill -0` it. Every other arm here acquires a
          # freshly created inode, where append and offset-0 coincide.
          reused = Path(td) / "reused.lock"
          first = _acquire_run_lock(reused, timeout_s=5.0, poll_s=0.02)
          if first is not None:
              first.close()
              if first in _RUN_LOCK_HELD:
                  _RUN_LOCK_HELD.remove(first)
          # A CHILD, so the second holder has a genuinely different pid.
          second = subprocess.run(
              [sys.executable, "-c", _LOCK_REACQUIRE % {
                  "dirq": repr(str(Path(__file__).resolve().parent))},
               str(reused)], capture_output=True, text=True, timeout=120)
          record = reused.read_text(errors="replace")
          child_pid = second.stdout.strip().splitlines()[-1] \
              if second.stdout.strip() else ""
          if second.returncode != 0 or not child_pid.isdigit():
              print(f"FAIL  [checker run lock: the re-acquisition child "
                    f"failed: {(second.stdout + second.stderr)[-200:]!r}]")
              errors += 1
          elif f"pid {child_pid}" not in record[:_LOCK_RECORD_WIDTH]:
              print(f"FAIL  [checker run lock: after a SECOND acquisition "
                    f"the holder record is {record[:64]!r}, which does not "
                    f"name the current holder (pid {child_pid}). An "
                    f"appending handle would leave the first holder's line "
                    f"at offset 0 forever]")
              errors += 1
          elif len(record) != _LOCK_RECORD_WIDTH:
              print(f"FAIL  [checker run lock: the lock file is "
                    f"{len(record)} bytes after two acquisitions, not "
                    f"{_LOCK_RECORD_WIDTH} — it is growing per acquisition, "
                    f"which is what an appending handle does]")
              errors += 1
          # ...and the control: an ordinary path still locks, 0600, with a
          # readable holder record. Without it, an acquirer that refused
          # EVERYTHING would pass all of the above.
          fresh = Path(td) / "fresh.lock"
          ok = _acquire_run_lock(fresh, timeout_s=5.0, poll_s=0.02)
          if ok is None:
              print("FAIL  [checker run lock: an ordinary lock path was "
                    "refused too — the guards above refuse everything, so "
                    "they prove nothing]")
              errors += 1
          else:
              mode = stat.S_IMODE(os.stat(fresh).st_mode)
              record = fresh.read_text(errors="replace")
              ok.close()
              if ok in _RUN_LOCK_HELD:
                  _RUN_LOCK_HELD.remove(ok)
              if mode & 0o077:
                  print(f"FAIL  [checker run lock: the lock file is mode "
                        f"{mode:04o} — a lock file has no reason to be "
                        f"readable by anyone else]")
                  errors += 1
              # A separate `if`: chained, a mode regression would hide a
              # holder-record regression in the same run.
              if f"pid {os.getpid()}" not in record:
                  print(f"FAIL  [checker run lock: the holder record must "
                        f"name this pid, got {record[:40]!r}]")
                  errors += 1
    finally:
        globals()["_RUN_LOCK_UNAVAILABLE"] = saved_reason_plant
    if errors == before_plant:
          print("ok    [checker run lock: a symlink, a hard link, a directory "
                "and a FIFO planted at the lock path are each refused "
                "loudly with the symlink's target untouched; a platform "
                "with no O_NOFOLLOW is refused rather than served; and an "
                "ordinary path still locks at 0600, tightening a "
                "pre-existing looser one]")

    # ---- (c4) no lock, no run --------------------------------------------
    # Warning and continuing was the old behaviour, and it defeats the lock
    # exactly where the lock matters. The decision is pure, so it is driven
    # against a hand table; the WIRING is read off this file's own AST,
    # because what makes it a gate is that it sits at module scope and
    # raises.
    before_gate = errors
    for acquired, override, want in ((True, False, "run"),
                                     (True, True, "run"),
                                     (False, False, "refuse"),
                                     (False, True, "run-unlocked")):
        got = _lock_gate_decision(acquired, override)
        if got != want:
            print(f"FAIL  [checker lock gate: acquired={acquired} "
                  f"override={override} => {got!r}, want {want!r}]")
            errors += 1
    gate_raises = False
    for node in tree.body:
        if not isinstance(node, ast.If):
            continue
        for sub in ast.walk(node):
            if isinstance(sub, ast.Raise) and any(
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Name)
                    and call.func.id == "SystemExit"
                    for call in ast.walk(sub)):
                gate_raises = True
    if not gate_raises:
        print("FAIL  [checker lock gate: no module-level `raise SystemExit` "
              "— the decision is computed and then ignored, so a run with "
              "no lock proceeds into the fixture-mutating checks anyway]")
        errors += 1
    if errors == before_gate:
        print("ok    [checker lock gate: the decision matches its hand "
              "table on all four inputs, and a run that cannot lock exits "
              "at module scope]")

    # ---- (c5) the REAL script refuses, end to end ------------------------
    # The arms above drive the pieces; this drives the shipped entry point.
    # `_run_lock_path()` reads `gettempdir()`, which honours `$TMPDIR`, so
    # the child gets a lock path this arm owns and can plant on -- without
    # touching the one THIS run is holding. No recursion: the refusal
    # happens at module scope, so the child dies before it runs a single
    # check.
    before_e2e = errors
    e2e_ran = False
    if os.environ.get(_GATE_CHILD_ENV):
        # A child of this arm. Belt and braces: the control child below is
        # stopped by `main()`'s argv validation, and if that ever stopped
        # stopping it, the child would run the whole suite -- including
        # THIS arm, whose own children would do the same. One env marker
        # makes that impossible rather than merely unlikely.
        #
        # It skips THIS BLOCK and nothing else. A `return` here skipped
        # arm (d) as well -- exclusion, release, the holder announce, the
        # timeout -- and the run still printed the PASS line that claims
        # "a second process is refused it", which arm (d) is the only
        # proof of. A stray export of a plainly-named variable is enough
        # to arrange that.
        print("skip  [checker lock gate e2e: this process is one of the "
              "arm's own children]")
    else:
      e2e_ran = True
      with tempfile.TemporaryDirectory(prefix="e2e_") as td:
          env = dict(os.environ, TMPDIR=td, **{_GATE_CHILD_ENV: "1"})
          # The child's lock path, BY CONSTRUCTION rather than by
          # re-deriving the name here. Only the directory differs under a
          # redirected TMPDIR, so reusing the real path's basename means the
          # plant cannot miss. It mattering is not hypothetical: a plant that
          # missed would let the child take a lock of its own and run the
          # WHOLE suite, writing the same bench-workspace fixtures as the
          # parent — the exact collision this file exists to prevent.
          child_lock = Path(td) / _run_lock_path().name
          child_lock.mkdir()               # unopenable: not a regular file
          try:
              r = subprocess.run(
                  [sys.executable, str(Path(__file__).resolve())],
                  env=env, capture_output=True, text=True, timeout=300)
          except subprocess.TimeoutExpired:
              # This arm rests on the child honouring `$TMPDIR`. If it ever
              # did not, the child would take the REAL lock path this
              # process holds and wait it out — and an unattributed
              # TimeoutExpired out of main() is not the FAIL line the arm
              # exists to print.
              print("FAIL  [checker lock gate e2e: the child never "
                    "finished — it is not looking at the planted lock "
                    "path, so this arm is measuring nothing]")
              return errors + 1
          blob = r.stdout + r.stderr
          if r.returncode == 0:
              print(f"FAIL  [checker lock gate e2e: the script ran to "
                    f"completion with its lock path unusable — two such runs "
                    f"would write the same fixtures. rc={r.returncode}]")
              errors += 1
          elif "refusing to run without its lock" not in blob:
              print(f"FAIL  [checker lock gate e2e: it exited "
                    f"{r.returncode} without saying the lock is why: "
                    f"{blob.strip()[-300:]!r}]")
              errors += 1
          elif NO_LOCK_FLAG not in blob:
              print(f"FAIL  [checker lock gate e2e: the refusal must name the "
                    f"way out ({NO_LOCK_FLAG}), got {blob.strip()[-300:]!r}]")
              errors += 1
          elif "ok    [" in r.stdout:
              print(f"FAIL  [checker lock gate e2e: it refused, but only "
                    f"AFTER running checks — the gate must sit at module "
                    f"scope, before any fixture is written]")
              errors += 1
          # THE OVERRIDE AGAINST A HELD LOCK, which is the case the
          # refusal's own advice names. Asserted by the ABSENCE of the
          # waiting note rather than by a wall: that note is printed only
          # after the acquirer has blocked, so a run that waited cannot be
          # quiet and a run that bypassed cannot be noisy.
          # A SECOND temp dir, whose lock path is a real file this process
          # HOLDS. The first one has a directory planted at the lock path,
          # so a child pointed there never reaches the contended wait at
          # all and the arm would pass for the wrong reason.
          held_dir = tempfile.mkdtemp(prefix="held_")
          held = _acquire_run_lock(
              Path(held_dir) / _run_lock_path().name, timeout_s=5.0,
              poll_s=0.02)
          if held is not None:
              held_env = dict(env)
              held_env["TMPDIR"] = held_dir
              try:
                  r3 = subprocess.run(
                      [sys.executable, str(Path(__file__).resolve()),
                       NO_LOCK_FLAG, "--stop-here-this-is-not-a-flag"],
                      env=held_env, capture_output=True, text=True,
                      timeout=120)
              except subprocess.TimeoutExpired:
                  print("FAIL  [checker lock gate e2e: with the lock held, "
                        f"{NO_LOCK_FLAG} did not return — it waited for the "
                        "holder instead of bypassing it]")
                  r3 = None
                  errors += 1
              finally:
                  held.close()
                  if held in _RUN_LOCK_HELD:
                      _RUN_LOCK_HELD.remove(held)
                  shutil.rmtree(held_dir, ignore_errors=True)
              if r3 is not None and "another instance holds" in r3.stderr:
                  print(f"FAIL  [checker lock gate e2e: {NO_LOCK_FLAG} went "
                        f"into the contended wait — the flag is consulted "
                        f"AFTER the acquisition, so the one escape the "
                        f"refusal advertises does not work in the case that "
                        f"produces the refusal]")
                  errors += 1
          # The CONTROL: the same child, same planted path, but asked to run
          # unlocked. It must get PAST the gate -- which is all this arm
          # claims, since letting it run the whole suite here would recurse.
          # The second argument is deliberately NOT a flag: the gate runs at
          # module scope and `main()`'s argv validation then stops the run
          # immediately after it, which is how the control observes the gate
          # without executing a single check.
          try:
              r2 = subprocess.run(
                  [sys.executable, str(Path(__file__).resolve()),
                   NO_LOCK_FLAG, "--stop-here-this-is-not-a-flag"],
                  # Well under a full run: the control is supposed to die
                  # at argv validation in a fraction of a second, so a
                  # generous bound would let a runaway suite finish.
                  env=env, capture_output=True, text=True, timeout=60)
          except subprocess.TimeoutExpired:
              print("FAIL  [checker lock gate e2e: the control child never "
                    "finished — the override did not stop it at the gate]")
              return errors + 1
          blob2 = r2.stdout + r2.stderr
          # Assert the STOP by its own signature, rather than inferring
          # it from the absence of evidence that the child ran. If that
          # validation is ever relaxed, the control child runs the WHOLE
          # suite unlocked against the real bench workspace, and an
          # `ok    [` check would notice only after it had done so.
          if r2.returncode != _EX_USAGE or \
                  "unrecognized argument" not in blob2:
              print(f"FAIL  [checker lock gate e2e: the control did not "
                    f"stop at `main()`'s argv validation "
                    f"(rc={r2.returncode}) — it may have run the suite "
                    f"unlocked; read that validation]")
              errors += 1
          elif "ok    [" in r2.stdout:
              print(f"FAIL  [checker lock gate e2e: the control was supposed "
                    f"to stop at `main()`'s argv validation and it ran checks "
                    f"instead — read that validation before trusting this "
                    f"arm]")
              errors += 1
          elif "refusing to run without its lock" in blob2:
              print(f"FAIL  [checker lock gate e2e: {NO_LOCK_FLAG} was "
                    f"refused as well — the override is inert, and the only "
                    f"way to run on such a host is to delete the guard]")
              errors += 1
          elif "running WITHOUT the lock" not in blob2:
              print(f"FAIL  [checker lock gate e2e: running unlocked must SAY "
                    f"so, got {blob2.strip()[-300:]!r}]")
              errors += 1
    # Only when it actually ran: a `skip` that still printed this line
    # would be claiming the shipped script was driven when nothing was.
    if e2e_ran and errors == before_e2e:
        print("ok    [checker lock gate e2e: the shipped script refuses at "
              "module scope when its lock path is unusable, names the way "
              "out, and takes it when it is asked for]")

    # ---- (d) exclusion, release, the announce, and the timeout -----------
    with tempfile.TemporaryDirectory(prefix="lockarm_") as td:
        arm = Path(td) / "arm.lock"
        free = Path(td) / "free.lock"
        handle = _acquire_run_lock(arm, timeout_s=5.0, poll_s=0.02)
        if handle is None:
            print("FAIL  [checker run lock: the acquirer declined an "
                  "uncontended temp lock]")
            return errors + 1
        waiter = None
        try:
            blocked = _probe_lock(arm)
            # The CONTROL, on a path nobody holds: without it, a probe
            # that could never take ANY lock would satisfy the line above.
            control = _probe_lock(free)
            # RELEASE, on its own lock with no waiter. Deliberately NOT on
            # `arm`: a waiting child is about to compete for that one, and
            # probing it after the release is a race the child wins often
            # enough to matter (MEASURED: 1 red in 5 runs under
            # `taskpolicy -b`). The waiter's own acquisition below is the
            # release proof for `arm`; this is the un-raced claim that
            # closing a handle frees the file at all.
            spent = Path(td) / "spent.lock"
            spent_handle = _acquire_run_lock(spent, timeout_s=5.0,
                                             poll_s=0.02)
            if spent_handle is None:
                print("FAIL  [checker run lock: the acquirer declined a "
                      "second uncontended temp lock]")
                errors += 1
            else:
                spent_handle.close()
                if spent_handle in _RUN_LOCK_HELD:
                    _RUN_LOCK_HELD.remove(spent_handle)
                released = _probe_lock(spent)
                if not released["acquired"]:
                    print(f"FAIL  [checker run lock: the lock was still "
                          f"held after its handle was closed "
                          f"({released['error']}) — a run that ends must "
                          f"not wedge the next one]")
                    errors += 1
            if blocked["acquired"]:
                print("FAIL  [checker run lock: a second process took a "
                      "lock this process holds — the lock does not "
                      "exclude]")
                errors += 1
            if not control["acquired"]:
                print(f"FAIL  [checker run lock: the CONTROL probe could "
                      f"not take an UNHELD lock either ({control['error']}) "
                      f"— so the refusal above proves nothing about the "
                      f"lock]")
                errors += 1

            # A run that CANNOT get the lock must exit rather than proceed.
            # Its own arm, with a short budget and NO claim about the
            # waiting note: the note is emitted after two polls, so
            # asserting both against one 0.2s deadline is a ~20ms margin —
            # MEASURED at 3 false FAILs in 6 runs under `taskpolicy -b`.
            expired = _drive_lock_waiter(arm, 0.2, td, "expired")
            if expired is None:
                return errors + 1
            if not expired["exited"]:
                print("FAIL  [checker run lock: a run that could NOT get "
                      "the lock proceeded anyway — it would race the "
                      "fixtures the holder is writing into the real tree]")
                errors += 1
            elif str(arm) not in expired["message"]:
                print(f"FAIL  [checker run lock: the wedged-peer refusal "
                      f"must name the lock file, got "
                      f"{expired['message']!r}]")
                errors += 1

            # ...and a run that waits must SAY who holds it and then GET
            # it. Bounded by a CONDITION (the note appearing), never by a
            # wall in units of the poll interval: the child's budget is
            # 120s and nothing here asserts how long anything took.
            waiter = _spawn_lock_waiter(arm, 120.0, td, "waits")
            note = _await_lock_note(waiter, _LOCK_NOTE_DEADLINE_S)
            if note is None:
                print(f"FAIL  [checker run lock: a waiting run printed no "
                      f"holder note within {_LOCK_NOTE_DEADLINE_S:.0f}s — "
                      f"an operator staring at a stalled gate would have "
                      f"nothing to act on. stderr: "
                      f"{waiter.read_err().strip()[-200:]!r}]")
                errors += 1
            elif f"pid {os.getpid()}" not in note:
                print(f"FAIL  [checker run lock: the holder note must name "
                      f"the holding pid ({os.getpid()}), got "
                      f"{note.strip()[-200:]!r}]")
                errors += 1
            handle.close()
            if handle in _RUN_LOCK_HELD:
                _RUN_LOCK_HELD.remove(handle)
            handle = None
            # THE claim the deleted interval arm used to make, and the one
            # nothing else here covers: a waiting run does not merely
            # refuse, it eventually ACQUIRES.
            got = _finish_lock_waiter(waiter)
            waiter = None
            if got is None:
                return errors + 1
            if got["exited"] or not got["acquired"]:
                print(f"FAIL  [checker run lock: a run that waited for the "
                      f"lock never got it after the holder released "
                      f"({got}) — waiting would be a deadlock dressed as "
                      f"serialization]")
                errors += 1
            elif errors == 0:
                print("ok    [checker run lock: a second process is "
                      "refused while it is held, a waiting run names the "
                      "holder and then acquires once it is released, and a "
                      "run that cannot get it exits naming the file]")
        except (RuntimeError, subprocess.SubprocessError, ValueError,
                IndexError) as e:
            print(f"FAIL  [checker run lock: the exclusion arm could not "
                  f"be driven: {e}]")
            errors += 1
        finally:
            # The handle must not outlive this arm even on an error path:
            # `flock` releases on last-descriptor-close, and a leaked one
            # would hold a temp lock for the rest of the process.
            if handle is not None:
                handle.close()
                if handle in _RUN_LOCK_HELD:
                    _RUN_LOCK_HELD.remove(handle)
            if waiter is not None:
                waiter.close()
    return errors



def check_rep_aggregation() -> int:
    """Pin the A1 cross-rep aggregation rules against hand oracles.

    (a) A SINGLE rep must aggregate byte-identically to the plain summarize
        p50 (legacy rep-less artifacts stay unchanged) for every fixture.
    (b) The multi-rep headline is the MEDIAN of per-rep p50s with the
        min/max spread — hand-computed, never a self-compare.
    (c) bench.median_int (the smoke A4 median-of-rep-p50s) matches
        compile_csv.percentile at 0.5 on the same small vectors."""
    errors = 0

    for desc, values in FIXTURES:
        single = compile_csv.aggregate_reps([list(values)])
        plain = compile_csv.summarize(list(values))
        if (single["p50"] != plain["p50"]
                or single["rep_count"] != 1
                or single["rep_p50_min"] != plain["p50"]
                or single["rep_p50_max"] != plain["p50"]
                or single["iterations"] != plain["iterations"]):
            print(f"FAIL  [single-rep aggregate != summarize: {desc}]")
            print(f"      aggregate={single}")
            print(f"      summarize={plain}")
            errors += 1
        else:
            print(f"ok    [single-rep aggregate == summarize: {desc}]")

    # Hand oracle: rep p50s are 20, 200, 50 → median 50, spread [20, 200];
    # pooled n = 9.
    reps = [[10, 20, 30], [100, 200, 300], [40, 50, 60]]
    agg = compile_csv.aggregate_reps([list(r) for r in reps])
    want = {"p50": 50, "rep_p50_min": 20, "rep_p50_max": 200,
            "rep_count": 3, "iterations": 9, "one_way_p50": 25}
    got = {k: agg[k] for k in want}
    if got != want:
        print(f"FAIL  [3-rep median-of-p50s hand oracle]")
        print(f"      want={want}")
        print(f"      got ={got}")
        errors += 1
    else:
        print(f"ok    [3-rep median-of-p50s hand oracle]  headline p50=50, "
              f"spread=[20, 200]")

    for desc, values in FIXTURES:
        m_bench = bench.median_int(list(values))
        m_csv = compile_csv.percentile(sorted(values), 0.5)
        if m_bench != m_csv:
            print(f"FAIL  [bench.median_int diverges: {desc}] "
                  f"{m_bench} != {m_csv}")
            errors += 1
    return errors


def check_torn_dump_refused() -> int:
    """Both sample readers must REFUSE a torn / partial u64 dump (a length
    that is not a multiple of 8) rather than truncate-and-continue: a torn
    dump whose floor(len / 8) equals the schedule count would otherwise
    pass compile_csv's strict sample-count gate. bench.read_p50_ns has
    always refused; compile_csv.read_samples_ns raises ValueError."""
    errors = 0
    with tempfile.TemporaryDirectory(prefix="torn_") as td:
        torn = Path(td) / "iox2_chrt0_64.bin"
        torn.write_bytes(_make_blob([100, 200]) + b"\x01")   # 17 bytes
        try:
            compile_csv.read_samples_ns(torn)
        except ValueError as e:
            if "multiple of 8" in str(e):
                print("ok    [compile_csv.read_samples_ns refuses a torn dump]")
            else:
                print(f"FAIL  [torn dump: wrong ValueError text: {e}]")
                errors += 1
        else:
            print("FAIL  [compile_csv.read_samples_ns ACCEPTED a 17-byte "
                  "torn dump — truncate-and-continue is the bug]")
            errors += 1
        try:
            bench.read_p50_ns(torn)
        except bench.SmokeSetupError:
            print("ok    [bench.read_p50_ns refuses a torn dump]")
        else:
            print("FAIL  [bench.read_p50_ns ACCEPTED a 17-byte torn dump]")
            errors += 1
        # Anti-tautology: the intact 16-byte twin is read by BOTH.
        good = Path(td) / "iox2_chrt0_256.bin"
        good.write_bytes(_make_blob([100, 200]))
        if (compile_csv.read_samples_ns(good) != [100, 200]
                or bench.read_p50_ns(good) != 150):
            print("FAIL  [intact 16-byte dump not read by both readers]")
            errors += 1
        else:
            print("ok    [intact dump read by both readers]")
    return errors


def check_smoke_sample_contract() -> int:
    """G1 contract for the smoke dispatcher (bench._run_smoke_cells):
    every smoke cell retains SMOKE_MEASURED measured samples after
    SMOKE_WARMUP warmup, whichever spelling carries the count —
    CER_BENCH_SMOKE_N=SMOKE_TOTAL (a TOTAL the paced schedules carve
    warmup out of) for native/workspace, CER_BENCH_TARGET_SAMPLES=
    SMOKE_MEASURED / CER_BENCH_WARMUP=SMOKE_WARMUP (MEASURED + warmup)
    for ROS 2 + backtoback. Pinned by driving the REAL schedule mirrors
    under the exact env the dispatcher exports."""
    errors = 0
    want = (bench.SMOKE_MEASURED, bench.SMOKE_WARMUP)
    if bench.SMOKE_MEASURED + bench.SMOKE_WARMUP != bench.SMOKE_TOTAL:
        print("FAIL  [SMOKE_MEASURED + SMOKE_WARMUP != SMOKE_TOTAL]")
        errors += 1
    if bench.SMOKE_WARMUP != max(bench.SMOKE_TOTAL // 10, 1):
        print("FAIL  [SMOKE_WARMUP is not the schedules' max(N // 10, 1) rule]")
        errors += 1
    saved = {k: os.environ.get(k) for k in
             ("CER_BENCH_SMOKE_N", "CER_BENCH_TARGET_SAMPLES",
              "CER_BENCH_WARMUP")}
    try:
        # Drive the REAL dispatcher exports (bench.smoke_sample_env) into
        # the REAL schedule mirrors: what each spelling makes a cell retain.
        for label, fn in (("quiescent", bench.quiescent_schedule),
                          ("fixed100", bench.fixed100_schedule)):
            for k in saved:
                os.environ.pop(k, None)
            ros2_env, overrides = bench.smoke_sample_env(label)
            if set(overrides) != {"CER_BENCH_SMOKE_N"}:
                print(f"FAIL  [{label} smoke overrides should be SMOKE_N "
                      f"only: {overrides}]")
                errors += 1
            os.environ.update(overrides)
            for payload in (64, 1048576):
                _rate, total, warmup = fn(payload)
                got = (total - warmup, warmup)
                if got != want:
                    print(f"FAIL  [{label} schedule under the smoke "
                          f"SMOKE_N export: payload {payload} retains "
                          f"{got}, dispatcher promises {want}]")
                    errors += 1
                else:
                    print(f"ok    [{label} SMOKE_N={overrides['CER_BENCH_SMOKE_N']}"
                          f" @ {payload}: measured/warmup = {got}]")
            # The ROS 2 export for the same variant: MEASURED + warmup
            # under G1 (samples_for's backtoback arm reads exactly this
            # env pair; docker_args_for_cell passes it verbatim).
            for k in saved:
                os.environ.pop(k, None)
            os.environ.update({k: ros2_env[k] for k in
                               ("CER_BENCH_TARGET_SAMPLES", "CER_BENCH_WARMUP")})
            _rate, measured, warmup = bench.samples_for("backtoback", 64)
            legacy = (int(ros2_env["TARGET_SAMPLES"]), int(ros2_env["WARMUP"]))
            if (measured, warmup) != want or legacy != want:
                print(f"FAIL  [{label} ROS 2 smoke export retains "
                      f"{(measured, warmup)} / legacy {legacy}, want {want}]")
                errors += 1
            else:
                print(f"ok    [{label} ROS 2 TARGET_SAMPLES/WARMUP export: "
                      f"measured/warmup = {want}]")
        for k in saved:
            os.environ.pop(k, None)
        _ros2, overrides = bench.smoke_sample_env("backtoback")
        os.environ.update(overrides)
        _rate, measured, warmup = bench.samples_for("backtoback", 64)
        if (measured, warmup) != want:
            print(f"FAIL  [backtoback smoke export retains "
                  f"{(measured, warmup)}, want {want}]")
            errors += 1
        else:
            print(f"ok    [backtoback TARGET_SAMPLES/WARMUP export: "
                  f"measured/warmup = {want}]")
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    return errors


def check_plot_variant_gate() -> int:
    """plot.py's expected-series enumeration honors the pacing variant:
    the split leg is structurally unmeasurable under backtoback
    (bench.workspace_leg_skip_reason), so a strict backtoback render must
    not EXPECT its CSV, while quiescent expects both legs. Also pins
    resolve_variant: manifest value taken by default, contradiction
    refused, no manifest -> quiescent."""
    import json
    try:
        import plot
    except ImportError as e:   # matplotlib absent on this host
        print(f"skip  [plot variant gate: plot.py unimportable: {e}]")
        return 0
    errors = 0
    q = {s.csv_name for s in plot.workspace_series("0", "quiescent")}
    b = {s.csv_name for s in plot.workspace_series("0", "backtoback")}
    # Both legs × both type classes (METHODOLOGY §18): the variable rows
    # keep the pinned token-less prefixes; the pod twins carry `_pod`.
    quiescent_expected = {
        f"results_{bench.workspace_raw_name(leg, 0, msg)}.csv"
        for leg in bench.WORKSPACE_LEGS
        for msg in bench.WORKSPACE_MSG_CLASSES}
    backtoback_expected = {
        f"results_{bench.workspace_raw_name('mono', 0, msg)}.csv"
        for msg in bench.WORKSPACE_MSG_CLASSES}
    if q != quiescent_expected:
        print(f"FAIL  [quiescent workspace_series != both legs x both "
              f"classes: {sorted(q)}]")
        errors += 1
    if b != backtoback_expected:
        print(f"FAIL  [backtoback workspace_series should be mono only "
              f"(both classes): {sorted(b)}]")
        errors += 1
    if not errors:
        print("ok    [workspace_series: quiescent = split+mono x "
              "variable+pod, backtoback = mono only x variable+pod]")
    with tempfile.TemporaryDirectory(prefix="variant_") as td:
        d = Path(td)
        if plot.resolve_variant(d, None) != "quiescent":
            print("FAIL  [resolve_variant: no manifest should default to "
                  "quiescent]")
            errors += 1
        (d / "run.json").write_text(json.dumps(
            {"variant": "backtoback", "invocations": []}))
        if plot.resolve_variant(d, None) != "backtoback":
            print("FAIL  [resolve_variant: manifest variant not taken]")
            errors += 1
        if plot.resolve_variant(d, "backtoback") != "backtoback":
            print("FAIL  [resolve_variant: matching --variant refused]")
            errors += 1
        try:
            plot.resolve_variant(d, "quiescent")
        except SystemExit:
            print("ok    [resolve_variant: manifest default / match / "
                  "contradiction refused]")
        else:
            print("FAIL  [resolve_variant: --variant quiescent over a "
                  "backtoback manifest was NOT refused]")
            errors += 1
    return errors


_STUB_BENCH = """#!/bin/sh
# Stand-in for a native bench binary (check_usage_wiring): writes one .bin
# per size in CER_BENCH_PAYLOAD_SIZES and records what it saw.
#
# CER_STUB_SAMPLES is the u64 sample count to write. It is NOT cosmetic:
# run_native_bench gates each .bin's length against
# samples_for(variant, payload)[1], so a stub writing an arbitrary count
# would make the checker fail on the sample-count gate instead of on the
# wiring it is actually testing. The caller derives the value from the
# schedule and asserts it is uniform across the stub's sizes.
# CER_STUB_SIDECAR_WAIT_S: set ONLY for the usage-ON phase. run_native_bench
# spawns this stub FIRST and the usage sampler second, then SIGTERMs the
# sampler the moment the stub exits -- so a stub that just slept 0.1s was
# racing a fresh python interpreter's start-up, and under load the sampler
# died before writing its sidecar ("FAIL [missing per-size sidecar ...]").
# REPRODUCED on the pre-fix head: 3 concurrent runs, 2 of them red; widening
# this sleep to 3s made the same 3 runs green. Waiting on the sidecar itself
# makes the arm depend on the ORDERING it is testing rather than on how
# quickly this box starts an interpreter. Bounded, so a sampler that never
# writes still fails the arm -- loudly, and for its own reason.
set -eu
echo "invocation: ${CER_BENCH_PAYLOAD_SIZES:-UNSET}" >> "$CER_STUB_TRACE"
sizes=$(echo "$CER_BENCH_PAYLOAD_SIZES" | tr ',' ' ')
for size in $sizes; do
    dd if=/dev/zero bs=8 count="${CER_STUB_SAMPLES:-1}" \
        of="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${size}.bin" \
        2>/dev/null
done
if [ -n "${CER_STUB_SIDECAR_WAIT_S:-}" ]; then
    for size in $sizes; do
        sc="$CER_BENCH_RAW_DUMP_DIR/${CER_BENCH_RAW_NAME}_${size}.usage.csv"
        waited=0
        limit=$(( CER_STUB_SIDECAR_WAIT_S * 20 ))
        while [ ! -s "$sc" ] && [ "$waited" -lt "$limit" ]; do
            sleep 0.05
            waited=$(( waited + 1 ))
        done
        if [ ! -s "$sc" ]; then
            # Otherwise the only evidence is the downstream "missing
            # per-size sidecar", which reads the same whether the sampler
            # never wrote or this stub stopped waiting. The per-cell log
            # is asserted to exist, so this line is where it lands.
            echo "stub: sidecar wait of ${CER_STUB_SIDECAR_WAIT_S}s expired\
 for $sc" >&2
        fi
    done
else
    sleep 0.1
fi
"""

_STUB_SAMPLER = """#!/usr/bin/env python3
# Stand-in for usage_sampler.py (check_usage_wiring): platform-neutral,
# writes a minimal valid sidecar at --out so the wiring's naming and
# stale-guard behavior are observable without Linux /proc.
#
# CER_STUB_SAMPLER_DELAY_S stands in for a SLOW START. run_native_bench
# spawns the bench first and the sampler second, then SIGTERMs the sampler
# the instant the bench exits -- so how much of the sampler runs is decided
# by the bench's lifetime, and a real sampler's interpreter start-up is not
# instant. Without this delay the arm passed or failed according to how
# quickly the machine could start python (MEASURED: 2 of 3 concurrent runs red
# on the pre-fix head, all green once the bench waited), which is a coin
# toss, not a check. Be precise about WHAT it makes deterministic: the
# delay is a property of this STUB, and what it pins is this ARM -- the
# per-size sidecar naming and the stale guard are judged on a sidecar that
# is really there, rather than on whether the machine started an interpreter
# in under 100 ms. The production ordering (spawn bench, spawn sampler,
# SIGTERM the sampler when the bench exits) is unchanged and is not what
# this delay tests.
import argparse, os, sys, time
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--parent-pid", type=int)
ap.add_argument("--descend", action="store_true")
ap.add_argument("--docker-name")
a = ap.parse_args()
time.sleep(float(os.environ.get("CER_STUB_SAMPLER_DELAY_S", "0") or 0))
with open(a.out, "w") as f:
    f.write("# stub sampler (check_usage_wiring)\\n")
    f.write("ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb\\n")
    f.write(f"1,{a.parent_pid or 0},stub,1.0,100,90\\n")
    f.write("# sampler_self: cpu_s=0.0 wall_s=0.1 cpu_pct_of_one_core=0.1 "
            "rows=1 ticks=1 pss_denied=0 cgroup_rows=0 discovery=static\\n")
time.sleep(60)   # parent stops us via SIGTERM (stop_usage_sampler)
"""


def check_one_line_encoder() -> int:
    """usage_sampler.one_line: free text cannot forge a sidecar record.

    The sidecar is line-oriented and plot_usage accepts any non-`#` line
    with 6 comma-separated fields as a measurement, so a line break in
    `--label` / `--docker-name` / a process's `comm` emits a row nobody
    sampled (Principle #13). The encoder folds on `str.splitlines()`, the
    same function the reader splits on; these vectors pin that the fold
    covers the WHOLE of that set, not just CR/LF — the four exotic
    separators below are exactly what a hand-written escape would miss.
    The forgery case is driven end to end through the real reader."""
    try:
        import usage_sampler
        import plot_usage
    except ImportError as e:
        print(f"skip  [one_line encoder: unimportable: {e}]")
        return 0
    errors = 0
    vectors = [
        ("plain text untouched", "plain", "plain"),
        ("newline", "a\nb", "a\\nb"),
        ("CRLF is ONE break", "a\r\nb", "a\\nb"),
        ("vertical tab", "a\vb", "a\\nb"),
        ("form feed", "a\fb", "a\\nb"),
        ("NEL U+0085", "a\x85b", "a\\nb"),
        ("line separator U+2028", "a\u2028b", "a\\nb"),
        ("backslash doubled", "back\\slash", "back\\\\slash"),
        ("empty", "", ""),
        ("trailing break dropped", "trail\n", "trail"),
    ]
    for desc, src, want in vectors:
        got = usage_sampler.one_line(src)
        if got != want or len(got.splitlines()) > 1:
            print(f"FAIL  [one_line: {desc}]  {src!r} -> {got!r}, "
                  f"expected {want!r} on one line")
            errors += 1
    # End to end: a label crafted to look like a data row must not become
    # one. The oracle is the REAL reader's row list, not the encoder's
    # output, so a fold that stopped covering the reader's break set would
    # fail here even if the vectors above were relaxed.
    forged = "mylabel\n1000000,4242,ghost_proc,99.0,123456,123456"
    with tempfile.TemporaryDirectory(prefix="oneline_") as td:
        sidecar = Path(td) / "cell_64.usage.csv"
        sidecar.write_text(
            "# usage_sampler v1 scope=procs\n"
            f"# label: {usage_sampler.one_line(forged)}\n"
            "ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb\n"
            "1,7,real_proc,10.0,100,100\n"
            "2,7,real_proc,10.0,100,100\n")
        parsed = plot_usage.parse_sidecar(sidecar)
        comms = [r[2] for r in (parsed or {}).get("rows", [])]
        if comms != ["real_proc", "real_proc"]:
            print(f"FAIL  [one_line: a forged label became a record]  "
                  f"reader returned comms={comms}, expected two real_proc")
            errors += 1
    if not errors:
        print(f"ok    [one_line: {len(vectors)} vectors + the forged-label "
              f"round trip through plot_usage.parse_sidecar]")
    return errors


class _CaptureProceeded(Exception):
    """Raised from the stubbed `docker_available` to end a --capture-baseline
    probe at the first call PAST the no-baseline return. Reaching it is the
    pass condition; letting it continue would build and run real cells."""


def _stop_before_building():
    raise _CaptureProceeded


# ---------------------------------------------------------------------------
# Platform prerequisites for the run_sampler probe (check_pids_tokenizer)
# ---------------------------------------------------------------------------

# usage_sampler.run_sampler reads exactly these five `os` facilities. The
# probe below drives the REAL function, so each of them has to answer on the
# host running this checker -- which this file's own docstring says may be
# ANY host ("stdlib only -- runnable on any host"). Three are portable; two
# are Unix-only, and BOTH were classified as "the probe is malformed" when
# they were absent:
#
#   os.sysconf  run_sampler's FIRST statement, BEFORE `args.pids` is read.
#               Absent on Windows -> AttributeError -> the probe reported a
#               malformed-probe FAIL and then a SECOND FAIL for never
#               reaching the tokenizer, on a host the same arm's own comment
#               calls expected.
#   os.uname    read while writing the sidecar header, AFTER the parse. Same
#               AttributeError, same misclassification -- the sibling of the
#               same class, one statement further in, and the reason the fix
#               is a shim TABLE rather than one `if`.
#
# Absent facilities are stubbed (only where the host genuinely lacks them),
# which leaves AttributeError meaning what the probe's diagnostic claims it
# means: this checker built a bad Namespace.
#
# The table is not decoration: `_run_sampler_reaches_the_tokenizer` DERIVES
# the `os.*` reads from run_sampler's AST and refuses any that is not
# classified here, so a sixth facility -- portable or not -- is a loud FAIL
# naming it rather than a third quiet misclassification. It is defined below
# the stubs it stores, because it STORES them: the classification and the
# stand-in are one fact, and a table that only NAMED the Unix-only ones
# would be a claim about other code with nothing binding it to that code.

# The POSIX-conventional tick rate. Only run_sampler's cpu_pct arithmetic
# reads it, and this probe asserts nothing about cpu_pct, so the VALUE is
# immaterial here -- deliberately unasserted rather than gated: under the
# simulation the /proc readers answer "gone", so the division it feeds is
# never reached and any gate on it would be a claim nothing exercises.
_SC_CLK_TCK_FALLBACK = 100

# "the host did not have this attribute", distinct from any value it could
# legitimately hold. `None` served until now and was correct only by luck: a
# module-level `os` function is never None, but the restore path keys on it,
# so the day one IS, the restore DELETES a real attribute instead of putting
# it back.
_ABSENT = object()


# Resolved HERE, at import, before any stub can exist. `platform.system()`
# is itself implemented on top of `os.uname()` on Unix, so reading it from
# INSIDE the stub reaches through the very function being stubbed -- caught
# by the simulated arm below on its first run:
#
#   note  [pids tokenizer (platform-less simulation): run_sampler stopped at
#          RecursionError: maximum recursion depth exceeded ...]
#
# which is also why a note in the SIMULATED arm is a FAIL and not a note:
# there, every platform absence is modelled by this file, so anything that
# stops run_sampler is this file being wrong.
#
# What actually protects the shipped code is THESE TWO NAMES, not that
# guard: the stand-in reads frozen strings and never calls `platform` at
# all, so the cycle cannot form. Deleting them does not silently re-open it
# either -- the sidecar identity oracle names them, so the file stops with a
# loud NameError rather than quietly regrowing the hazard.
#
# The real scope of the guard, measured rather than assumed: a variant that
# resolves the identity inside the stub AND deletes these constants still
# PASSES, because `platform`'s own uname cache is already primed by then.
# TWO different things prime it, and they are worth keeping apart: in the
# SHIPPED file it is the `platform.system()` call on the line below; in that
# variant, where the line is gone, it is the adequacy check reading the
# stand-in's attributes while the real `os.uname` is still installed.
# Clearing `platform._uname_cache` at the moment of installation makes the
# variant fail as above -- so the recursion is unreachable through this
# file's CALL ORDER, which is an accident, not a property, and this comment
# does not claim the guard covers it. The guard's standing coverage
# is a run_sampler that raises a plain ValueError, which it does catch.
_HOST_SYSNAME = platform.system() or "unknown"
_HOST_MACHINE = platform.machine() or "unknown"


class _StubUname:
    """Stand-in for the ATTRIBUTES run_sampler reads off `os.uname()` --
    not for `os.uname()` itself (no nodename/release/version, no tuple
    behaviour). It carries the host's REAL sysname/machine, read portably
    at import, so a stubbed host still writes its own identity into the
    sidecar header instead of a fabricated one.

    Class attributes: the values are deliberately frozen at import, and
    `os.uname` is replaced by the CLASS, so `os.uname()` constructs one of
    these. `_run_sampler_reaches_the_tokenizer` derives the attribute set
    run_sampler actually reads off that call and refuses any an INSTANCE of
    this does not answer -- otherwise the stub would raise
    AttributeError and be reported as a malformed probe, which is the very
    misclassification this section exists to remove.
    """

    sysname = _HOST_SYSNAME
    machine = _HOST_MACHINE


# The `os` facilities `usage_sampler.run_sampler` reads, each with the
# stand-in installed where the host lacks it (`None` = portable, nothing to
# install) and the prose reason. ONE fact, ONE spelling: the prereq manager
# below installs these stand-ins, and BOTH the platform-less simulation and
# the anti-tautology assertion key on `_UNIX_ONLY_OS_FACILITIES`, which is
# DERIVED from the stand-in column -- so a facility given a stand-in is
# stubbed, simulated and asserted from ONE spelling, and a third Unix-only
# facility cannot be classified here while the simulation keeps modelling
# two.
#
# What is NOT enforced is the PROSE. The only machine-readable signal is
# "has a stand-in": a row whose text says Unix-only while its stand-in slot
# is None reads as portable, is never simulated, and its probe never runs.
# The column is the classification; the sentence is only a comment.
#
# The probe is a zero-argument callable rather than a `hasattr`: a facility
# can also be PRESENT and refuse the name we need (`os.sysconf` raising for
# an unsupported key), which is the same absence from run_sampler's point
# of view. It is late-bound, so a name missing at import is fine.
_OS_FACILITIES_RUN_SAMPLER_USES = {
    # name: (probe, stand-in or None if portable, prose)
    "cpu_count": (None, None, "portable to every platform CPython ships on"),
    "getpid": (None, None, "portable to every platform CPython ships on"),
    "times": (None, None, "portable to every platform CPython ships on"),
    # Reached only through `_ppid_map`, on the descend branch this probe
    # never takes. Classified anyway, because the walk that keeps a
    # Unix-only CALLEE classified necessarily sees the portable ones too.
    "listdir": (None, None, "portable API (the /proc PATH is Linux-only, "
                            "but the probe pins --pids and never descends)"),
    "sysconf": (lambda: os.sysconf("SC_CLK_TCK"),
                lambda _name: _SC_CLK_TCK_FALLBACK,
                "Unix-only -- stubbed where absent"),
    "uname": (lambda: os.uname(), _StubUname,
              "Unix-only -- stubbed where absent"),
}

_UNIX_ONLY_OS_FACILITIES = tuple(sorted(
    name for name, (_probe, stub, _prose)
    in _OS_FACILITIES_RUN_SAMPLER_USES.items() if stub is not None))

# The `read_*` helpers run_sampler calls, each with the value the REAL one
# returns when the file is unreadable (they swallow OSError and answer
# "gone"), so the simulation is what run_sampler actually sees off Linux
# rather than an invented failure. Like the `os` table above, the set is
# DERIVED from run_sampler's AST and anything unclassified is a loud FAIL --
# a fourth reader must not leave the simulation quietly modelling three
# while the `ok` line keeps claiming "no readable /proc".
#
# SCOPE: the derivation keys on the `read_` NAME PREFIX, which is
# this module's convention rather than a fact the AST can prove. A future
# reader named otherwise is not seen -- one rung weaker than the `os.` table,
# where the prefix IS the module.
_ABSENT_PROC_READERS = {
    "read_stat_cpu_ticks": None,
    "read_rss_kb": None,
    "read_pss_kb": (None, False),
    # Reachable only on the docker branch, which this probe never takes; a
    # host with no /proc cannot answer it either, so it is modelled rather
    # than left as the one reader the simulation forgets.
    "read_cgroup_mem_kb": None,
}


@contextlib.contextmanager
def _platform_prereqs_for_run_sampler():
    """Stub the Unix-only `os` facilities THIS host lacks, for the duration.

    Nothing the host provides is touched: on Linux and macOS this is a
    no-op and the probe keeps running against the real facilities, so the
    shim can never mask a regression on the hosts the sweep runs on. Yields
    the names it stubbed (the caller reports them) and restores every one in
    the `finally`, whether the probe returned or raised.
    """
    saved = {}
    why = {}
    # Every mutation sits INSIDE the try, so a failure between two stubs
    # cannot leave the first one installed for the rest of the process --
    # main() runs ~28 checks in one interpreter, and a leaked `os` stub
    # would be a fake platform under every arm that follows.
    try:
        for name in _UNIX_ONLY_OS_FACILITIES:
            probe, stub, _prose = _OS_FACILITIES_RUN_SAMPLER_USES[name]
            if probe is None:
                # The PROBE column has the same shape as the stand-in
                # column and a worse failure mode: `except Exception` below
                # cannot tell "None is not callable" from "the host lacks
                # it", so a row with a stand-in and no probe installs on
                # EVERY host -- silently falsifying the no-op promise on
                # exactly the hosts the sweep runs on. Measured: it shipped
                # green with `os.uname` replaced on a macOS desk.
                raise TypeError(
                    f"_OS_FACILITIES_RUN_SAMPLER_USES[{name!r}] carries a "
                    f"stand-in but no probe, so the shim would install it "
                    f"on every host and mask a real regression.")
            try:
                probe()
            except Exception as e:   # noqa: BLE001 (absent, or name refused)
                # The reason is CARRIED, not assumed: this probe also fires
                # when the facility EXISTS and refuses the name we need
                # (`os.sysconf` raising ValueError for an unsupported key),
                # and telling that operator their host "lacks os.sysconf"
                # would be a wrong diagnosis on a host that has it.
                saved[name] = getattr(os, name, _ABSENT)
                why[name] = type(e).__name__
                setattr(os, name, stub)
        yield why
    finally:
        for name, original in saved.items():
            if original is _ABSENT:
                delattr(os, name)
            else:
                setattr(os, name, original)


@contextlib.contextmanager
def _simulated_platformless_host():
    """Present this host as one with no Unix-only `os` facility and no
    readable `/proc` -- the shape that produced the two-FAIL report.

    The facilities removed are exactly `_UNIX_ONLY_OS_FACILITIES`, so this
    simulation cannot fall behind the table: classify a sixth facility as
    Unix-only and this removes it too.

    The /proc READERS are stubbed rather than the filesystem: they already
    swallow OSError and answer "gone", so stubbing them is what a /proc-less
    host really looks like from run_sampler, and it makes the simulation mean
    the same thing on Linux (where pids 1 and 2 exist and would mint rows) as
    on a desk where they do not.

    YIELDS the consultation witness: the names of the stubbed readers
    run_sampler actually asked. Without it the /proc half is an unbacked
    claim -- measured, deleting all three reader stubs left the arm green,
    because on a desk that has no /proc the real readers answer "gone"
    anyway and the simulation adds nothing there. Today only
    `read_stat_cpu_ticks` is reachable (a "gone" answer short-circuits the
    other two); the other two are stubbed because the simulation models the
    HOST, not this month's call graph, and the assertion is that the half
    was consulted AT ALL.
    """
    saved_os = {n: getattr(os, n) for n in _UNIX_ONLY_OS_FACILITIES
                if hasattr(os, n)}
    # `hasattr`-guarded: a reader renamed upstream must reach the report
    # below, not raise inside `contextlib.__enter__` -- which is BEFORE the
    # `with` yields, so `drive()` cannot wrap it and the derivation guards
    # built for exactly this class never run.
    saved_readers = {n: getattr(usage_sampler, n)
                     for n in _ABSENT_PROC_READERS
                     if hasattr(usage_sampler, n)}
    consulted = []

    def absent_reader(name, answer):
        def reader(_pid):
            # The pid is recorded because only a real CALL has one: a
            # witness appended in the FACTORY would record installation,
            # which the simulation always performs, making the assertion a
            # tautology (measured -- it shipped green).
            consulted.append((name, _pid))
            return answer
        return reader

    # Same rule as the shim above: capture first, mutate INSIDE the try, so
    # a partial setup is still fully undone.
    try:
        for name in saved_os:
            delattr(os, name)
        for name, answer in _ABSENT_PROC_READERS.items():
            if hasattr(usage_sampler, name):
                setattr(usage_sampler, name, absent_reader(name, answer))
        yield consulted
    finally:
        for name, original in saved_os.items():
            setattr(os, name, original)
        for name, original in saved_readers.items():
            setattr(usage_sampler, name, original)


# The probe's verdict. A NAMED container, not a bare positional tuple:
# three of its five members are lists, and two of those mean OPPOSITE
# things -- `problems`
# is "this checker is wrong" (always a FAIL) and `notes` is "the host
# stopped run_sampler" (tolerated on the real host, a FAIL under the
# simulation). Measured: transposing them at the call site leaves the arm
# printing `ok` at rc 0, i.e. it silently reinstates the very
# misclassification this section exists to remove, and the simulated arm
# cannot catch it because there both lists are FAILs. Position is too weak
# a place to keep the severity contract.
_ProbeResult = collections.namedtuple(
    "_ProbeResult", "problems reached notes stubbed sidecar_uname")

# The spec the probe feeds run_sampler: the whitespace-only slot that IS the
# original bug. Named once, because the caller asserts the tokenizer was
# handed exactly THIS -- a consumer that pre-strips before delegating is a
# second tokenizer, which is the same disagreement in a new place.
_PIDS_PROBE_SPEC = "1, ,2"


def _run_sampler_reaches_the_tokenizer() -> "_ProbeResult":
    """Drive the REAL run_sampler far enough to reach parse_pids_spec.

    Returns a `_ProbeResult`: `.problems` are message strings the CALLER
    prints (the caller knows which arm it is driving and what a failure
    there means), `.reached` is what the spied tokenizer saw, `.notes` are
    genuine platform stops -- noted, never swallowed -- `.stubbed` maps
    each facility this host needed stubbed to WHY, and `.sidecar_uname` is
    the `uname=` token run_sampler wrote into its own header (None if it
    never got that far) -- the only readback that can tell a stand-in
    carrying the host's real identity from one carrying anything at all.

    The Namespace is still HAND-LISTED, but it is now cross-checked against
    what run_sampler reads off `args`: a field the AST finds and the probe
    does not supply is reported as a problem instead of dying on
    AttributeError inside a broad `except`. The hand-listed version omitted
    two fields (`rescan_hz` and `descend`) and that `except Exception: pass`
    swallowed the result -- so this arm could report green having never
    reached the consumer at all.

    It had not yet: `args.pids` is handed to the tokenizer before the
    first omitted field the `--pids` branch actually reaches (`rescan_hz`,
    in the sampling loop -- NOT `descend`, which sits in the branch `--pids`
    never takes), so the tokenizer mutant really did die past the parse. But
    that is an ORDERING ACCIDENT, not a property -- move the parse below
    either read and the arm goes quietly inert. Two changes make it not
    an accident:
    every field is supplied, and reaching the consumer is asserted
    POSITIVELY by the caller instead of inferred from the absence of an
    exception.
    """
    problems = []
    notes = []
    sampler_src = Path(usage_sampler.__file__).read_text()
    sampler_module = ast.parse(sampler_src)
    run_sampler = next(
        (fn for fn in ast.walk(sampler_module)
         if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef))
         and fn.name == "run_sampler"), None)
    if run_sampler is None:
        # A bare StopIteration here would be an unattributable traceback in
        # the middle of a 28-check run.
        problems.append(
            "usage_sampler declares no `run_sampler` function, so every "
            "derivation below is empty and this arm tests nothing. It was "
            "renamed or restructured -- retarget the probe.")
        return _ProbeResult(problems, [], notes, {}, None)
    # The SCOPES differ on purpose. `args.*` is a question about what
    # run_sampler reads off its OWN parameter, so it is derived from that
    # function; walking the module would sweep `main`'s `args.self_test`
    # and demand a field the probe has no business supplying.
    #
    # `os.*` is a question about the whole CALL GRAPH, so it is derived
    # from the MODULE. Measured, and it matters: a Unix-only facility in a
    # CALLEE is just as fatal -- `Discovery()` is constructed three
    # statements before the parse, so an `os.getuid()` in its __init__
    # reproduces the
    # filed two-FAIL report verbatim while a function-scoped walk stays
    # silent. Module scope over-classifies by exactly one name today
    # (`os.listdir`, in the descend-branch /proc scan); over-classifying
    # costs a table row, under-scoping costs the guarantee -- and the
    # stale-entry check below keeps an over-broad table self-pruning.
    reads = {node.attr for node in ast.walk(run_sampler)
             if isinstance(node, ast.Attribute)
             and isinstance(node.value, ast.Name) and node.value.id == "args"}
    os_reads = {node.attr for node in ast.walk(sampler_module)
                if isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "os"}
    proc_reads = {node.func.id for node in ast.walk(sampler_module)
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Name)
                  and node.func.id.startswith("read_")}
    # A derivation that finds NOTHING is a broken derivation, not a clean
    # bill of health -- and it is the shape ordinary refactoring produces
    # (`import os as _os`, `from os import sysconf`, renaming the parameter).
    # Measured: with the body rewritten to `ns.` / `_os.`, both guards below
    # silently no-op and this arm still printed `ok`.
    for label, derived in (("args", reads), ("os", os_reads),
                           ("read_*", proc_reads)):
        if not derived:
            problems.append(
                f"the run_sampler derivation found NO `{label}` reads. An "
                f"empty derivation makes the guards below no-ops while they "
                f"keep reporting green -- run_sampler no longer reaches "
                f"those names in a form `ast` can see (an aliased import, a "
                f"renamed parameter, a `from os import ...`). Retarget the "
                f"derivation rather than accepting the silence.")
    unclassified = sorted(os_reads - set(_OS_FACILITIES_RUN_SAMPLER_USES))
    # An `os.path.join(...)` yields the attr `path`, which is a NAMESPACE,
    # not a facility: telling its author to "give it a stand-in" would send
    # them somewhere no fix lives. Failing loud is still right -- only the
    # remedy differs, so the two are separated rather than merged.
    namespaces = [n for n in unclassified
                  if type(getattr(os, n, None)).__name__ == "module"]
    facilities = [n for n in unclassified if n not in namespaces]
    if facilities:
        problems.append(
            f"run_sampler reads os.{{{', '.join(facilities)}}}, which "
            f"_OS_FACILITIES_RUN_SAMPLER_USES does not classify. If it is "
            f"portable, say so there; if it is Unix-only, give it a "
            f"stand-in there -- an unclassified Unix-only facility is "
            f"reported as a malformed probe on the hosts that lack it, "
            f"which is the bug this table closes.")
    if namespaces:
        problems.append(
            f"run_sampler reaches into the os.{{{', '.join(namespaces)}}} "
            f"NAMESPACE, which this derivation reports as a bare attribute "
            f"and cannot classify: a namespace takes no stand-in. Classify "
            f"it as portable if it is, or narrow the derivation to the "
            f"call it actually makes.")
    stale = sorted(set(_OS_FACILITIES_RUN_SAMPLER_USES) - os_reads)
    if stale:
        problems.append(
            f"_OS_FACILITIES_RUN_SAMPLER_USES classifies os.{{"
            f"{', '.join(stale)}}}, which run_sampler no longer reads. A "
            f"stale entry keeps the simulation removing a facility nothing "
            f"needs, so the anti-tautology assertion below passes on a "
            f"host shape that no longer exists -- prune it.")
    stale_readers = sorted(n for n in _ABSENT_PROC_READERS
                           if not hasattr(usage_sampler, n))
    if stale_readers:
        problems.append(
            f"_ABSENT_PROC_READERS classifies "
            f"{{{', '.join(stale_readers)}}}, which usage_sampler no longer "
            f"declares. The simulation cannot stub a name that is gone, so "
            f"the /proc half it claims to model is thinner than the ok line "
            f"says -- prune or retarget the entry. (The `os` table above has "
            f"had this counterpart since it was written; this one did not, "
            f"and a removed reader died entering the context manager rather "
            f"than reporting.)")
    unclassified_readers = sorted(proc_reads - set(_ABSENT_PROC_READERS))
    if unclassified_readers:
        problems.append(
            f"usage_sampler calls "
            f"{{{', '.join(unclassified_readers)}}} somewhere in "
            f"run_sampler's call graph, which _ABSENT_PROC_READERS does "
            f"not classify, so the "
            f"platform-less simulation leaves it reading the real /proc "
            f"while the arm keeps claiming it modelled a host without one.")
    uname_attrs = {node.attr for node in ast.walk(sampler_module)
                   if isinstance(node, ast.Attribute)
                   and isinstance(node.value, ast.Call)
                   and isinstance(node.value.func, ast.Attribute)
                   and node.value.func.attr == "uname"}
    # Asked of an INSTANCE, which is what run_sampler gets: reading the
    # class dict would reject a stand-in that sets the same attributes in
    # `__init__` -- a guard on the implementation rather than the contract.
    #
    # MODULE-scoped, for the reason the `os.*` walk above is: an attribute
    # read off `os.uname()` in a CALLEE is just as fatal, and measured, a
    # function-scoped walk let one through as three FAILs none of which
    # named the cause -- the "reported as a malformed probe" outcome this
    # guard exists to prevent.
    stub_uname = _StubUname()
    missing_uname = sorted(a for a in uname_attrs
                           if not hasattr(stub_uname, a))
    if missing_uname:
        problems.append(
            f"run_sampler reads os.uname().{{{', '.join(missing_uname)}}}, "
            f"which _StubUname does not carry -- on a stubbed host the "
            f"stand-in would raise AttributeError and be reported as a "
            f"malformed probe, which is exactly the misclassification this "
            f"section removes.")
    ns_fields = {
        "pids": _PIDS_PROBE_SPEC, "parent_pid": None, "docker_name": None,
        # duration_s must be POSITIVE, not 0.0: run_sampler reads it as
        # `deadline = now + duration_s if args.duration_s else None`, so a
        # falsy 0.0 means "sample until signalled" and the probe never
        # returns. Measured -- it hung this checker.
        "docker_wait_s": 0.0, "duration_s": 0.01, "stat_hz": 10.0,
        "pss_hz": 1.0, "rescan_hz": 1.0, "descend": False, "label": "probe",
    }
    # Discovery is NOT stubbed. Stubbing it to `None` made the probe die on
    # `disc.mode` a few lines past the parse -- caught by the AttributeError
    # arm below, which is that arm doing its job on its own author. The real
    # Discovery constructs fine on every host (verified), and the point of
    # this probe is to reach the consumer at line 381, not to complete a
    # sampling run.
    saved_parse = usage_sampler.parse_pids_spec
    reached = []

    def _spy(spec):
        reached.append(spec)
        return saved_parse(spec)

    usage_sampler.parse_pids_spec = _spy
    try:
        with tempfile.TemporaryDirectory(prefix="pids_") as td, \
                _platform_prereqs_for_run_sampler() as stubbed:
            probe_ns = argparse.Namespace(out=str(Path(td) / "u.csv"),
                                          **ns_fields)
            missing = sorted(f for f in reads if not hasattr(probe_ns, f))
            if missing:
                problems.append(
                    f"the run_sampler probe is missing {missing} -- "
                    f"usage_sampler reads them off `args`, so the probe "
                    f"would die on AttributeError before reaching the "
                    f"consumer, and this arm would report green having "
                    f"tested nothing.")
            try:
                with contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    usage_sampler.run_sampler(probe_ns)
            except ValueError as e:
                if "invalid literal for int" in str(e):
                    problems.append(
                        f"run_sampler does not use the shared tokenizer -- "
                        f"`--pids \"{_PIDS_PROBE_SPEC}\"` raised {e}. The "
                        f"validator accepts that spec, so a consumer that "
                        f"parses it differently is the original bug, not a "
                        f"variant of it.")
                else:
                    # Every OTHER ValueError is a stop like any other. It
                    # used to fall out of this clause reported as nothing at
                    # all -- measured: a run_sampler raising ValueError on
                    # every call left this arm printing `ok`, which is this
                    # file's own thesis one `except` clause over.
                    notes.append(f"run_sampler stopped at ValueError: "
                                 f"{str(e)[:70]}")
            except (AttributeError, TypeError) as e:
                # A malformed probe, never the host: every platform
                # facility in run_sampler's call graph is either portable or
                # stubbed above, and every attribute read off `os.uname()`
                # in that graph is checked against the stand-in, so this can
                # no longer be a platform gap.
                # Reporting a platform gap as one of these is exactly how
                # this arm failed on hosts its own comments call expected.
                problems.append(
                    f"the run_sampler probe could not be invoked "
                    f"({type(e).__name__}: {e}). That is this checker "
                    f"being wrong, not the host -- fix the probe rather "
                    f"than tolerating it.")
            except Exception as e:                   # noqa: BLE001
                # Genuine platform failures (an unwritable path, SIGTERM
                # plumbing) are possible -- but they are NOTED, not
                # swallowed, so an arm that stops testing anything says so
                # out loud.
                notes.append(f"run_sampler stopped at "
                             f"{type(e).__name__}: {str(e)[:70]}")
            sidecar_uname = None
            sidecar = Path(probe_ns.out)
            if sidecar.exists():
                for line in sidecar.read_text().splitlines():
                    for token in line.split():
                        if token.startswith("uname="):
                            sidecar_uname = token[len("uname="):]
                    if sidecar_uname is not None:
                        break
            return _ProbeResult(problems, reached, notes, stubbed,
                                sidecar_uname)
    finally:
        usage_sampler.parse_pids_spec = saved_parse


def check_pids_tokenizer() -> int:
    """`--pids` must mean the same thing to its validator and its consumer.

    Three sites used to split that string -- the argparse validator, the
    emptiness check, and run_sampler. Two stripped each token and the third
    filtered on the RAW token, so a whitespace-only entry cleared both guards
    and reached `int(" ")`:

        --pids "1, ,2"  -> ValueError: invalid literal for int() with base
                           10: ' '

    a bare traceback deep inside run_sampler, which is precisely the failure
    the validator's own comment claimed to prevent. A validator that
    disagrees with its consumer about what a token IS cannot guard it.

    The oracle is a hand-written table, not a round-trip: a round-trip
    against the tokenizer itself would agree with any tokenizer, including
    the broken pair this replaces.
    """
    errors = 0
    CASES = [
        # (spec, expected)  -- "expected" is a list, or ValueError.
        ("1,2,3", [1, 2, 3]),
        ("1, 2", [1, 2]),          # has always worked; int() strips too
        ("1,,2", [1, 2]),          # empty slot dropped
        ("1, ,2", [1, 2]),         # THE BUG: whitespace-only slot, same rule
        ("  7  ", [7]),
        ("0", [0]),
        ("", []),                  # emptiness is the caller's to report
        (" ", []),                 # ...and means the same as ""
        (",", []),
        ("1,x", ValueError),
        ("1,-2", ValueError),      # negative is not a pid
        ("1,2.0", ValueError),
        ("1,٢", ValueError),       # non-ASCII digit: isdigit() is True
    ]
    for spec, want in CASES:
        try:
            got = usage_sampler.parse_pids_spec(spec)
        except ValueError:
            got = ValueError
        except Exception as e:                       # noqa: BLE001
            got = f"{type(e).__name__}: {e}"
        if got != want:
            print(f"FAIL  [pids tokenizer: {spec!r} -> {got!r}, expected "
                  f"{want!r}]")
            errors += 1

    # The property that actually broke: whatever the validator ACCEPTS, the
    # consumer must be able to parse. Asserted over the same table rather
    # than trusting that one function is now used everywhere.
    for spec, want in CASES:
        if want is ValueError:
            continue
        try:
            usage_sampler.parse_pids_spec(spec)
        except Exception as e:                       # noqa: BLE001
            print(f"FAIL  [pids tokenizer: the validator accepts {spec!r} "
                  f"but parsing it raises {type(e).__name__}: {e} -- that "
                  f"disagreement IS the bug, reappearing.]")
            errors += 1

    # The property the table above CANNOT see: that run_sampler actually
    # USES the shared tokenizer. Measured -- reverting its line to the old
    # `[int(p) for p in args.pids.split(",") if p]` left every case above
    # green, because they call parse_pids_spec directly and the consumer was
    # the half that disagreed. A tokenizer nobody consumes fixes nothing.
    #
    # run_sampler is driven for real, not walked for a string. Reaching the
    # consumer is asserted POSITIVELY, never inferred from the absence of an
    # exception: absence-as-evidence fails one layer down, where a swallowed
    # exception would leave no trace.
    def drive(label):
        """Run the probe, converting an ESCAPING exception into an
        attributable FAIL. Nothing inside the probe is supposed to escape
        -- but a checker that dies with a traceback mid-run reports nothing
        about the other 27 checks, and "the probe cannot raise" is exactly
        the kind of claim this file declines to take on trust. Measured:
        narrowing the shim probe to `except AttributeError` let a
        name-refusing `os.sysconf` propagate out of the context manager and
        abort the whole run.
        """
        try:
            return _run_sampler_reaches_the_tokenizer()
        except Exception as e:                       # noqa: BLE001
            print(f"FAIL  [pids tokenizer: the {label} probe raised "
                  f"{type(e).__name__}: {str(e)[:90]} instead of reporting. "
                  f"Every failure this probe can have is supposed to come "
                  f"back as a problem or a note -- an escaping exception "
                  f"takes the rest of the checker with it.]")
            return None

    # This arm has twice now reached for a Unix-only facility DIRECTLY
    # (`saved_sysconf = os.sysconf`), which is fine on this desk and a bare
    # traceback on the host the whole section exists for. Both instances
    # passed a read-through and were found by RUNNING the reported shape end
    # to end, so the third is prevented structurally instead: no line in this
    # function may READ one of those facilities off `os`. Writing one is
    # allowed -- installing a stand-in is the point -- so the walk keeps
    # Store context and refuses only Load.
    # MODULE-scoped, not this one function. Measured: moving the refusing
    # arm into a helper -- which is this file's own habit, done twice in
    # this change -- carried a bare `os.sysconf` read out of a name-scoped
    # walk's sight and aborted the checker again on the target host. The
    # The table's own probe lambdas are Loads too, and they are the ONLY
    # legitimate direct reads -- by construction they run inside the `try`
    # that classifies them. So the exemption is scoped to calls written
    # INSIDE that table, not to every call in the module: exempting any
    # immediately-called Load let an unrelated `os.uname()` added elsewhere
    # walk straight past the guard and abort the checker on a host lacking
    # it, which is the failure this guard exists to prevent.
    # try/except, not a falsy check: `read_text()` RAISES on an unreadable
    # or undecodable file and `ast.parse` raises SyntaxError -- neither ever
    # returns None -- so the falsy form could only fire on a zero-byte self,
    # while every real failure escaped uncaught from a block that sits
    # OUTSIDE `drive()`. A dead guard beside an unhandled path is the exact
    # shape this change exists to remove.
    try:
        _own_tree = ast.parse(Path(__file__).read_text())
    except Exception as e:                           # noqa: BLE001
        _own_tree = None
        _own_why = f"{type(e).__name__}: {str(e)[:70]}"
    if _own_tree is None:
        print(f"FAIL  [pids tokenizer: this file could not read or parse "
              f"ITSELF ({_own_why}), so the unguarded-facility walk below "
              f"is inert -- it reports nothing about a read this function "
              f"may be doing.]")
        errors += 1
    else:
        _own_fn = _own_tree
        _table = next(
            (n for n in ast.walk(_own_tree)
             if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name)
                     and t.id == "_OS_FACILITIES_RUN_SAMPLER_USES"
                     for t in n.targets)), None)
        if _table is None:
            print("FAIL  [pids tokenizer: the facility table could not be "
                  "located in this file's own AST, so the walk below would "
                  "exempt nothing and report the table's own probes as "
                  "unguarded reads.]")
            errors += 1
        _called = {id(n.func) for n in ast.walk(_table)
                   if isinstance(n, ast.Call)} if _table else set()
        bare = {node.attr for node in ast.walk(_own_fn)
                if isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "os"
                and isinstance(node.ctx, ast.Load)
                and id(node) not in _called
                and node.attr in _UNIX_ONLY_OS_FACILITIES}
        # `getattr(os, "sysconf")` with NO default is the same read, and it
        # is the remedy below minus one argument -- the spelling a reader
        # copying that advice is likeliest to typo. Measured: it walked
        # straight past an Attribute-only guard and still crashed the
        # checker on a host without the facility.
        bare |= {node.args[1].value for node in ast.walk(_own_fn)
                 if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name)
                 and node.func.id == "getattr"
                 and len(node.args) == 2
                 and isinstance(node.args[0], ast.Name)
                 and node.args[0].id == "os"
                 and isinstance(node.args[1], ast.Constant)
                 and node.args[1].value in _UNIX_ONLY_OS_FACILITIES}
        bare = sorted(bare)
        if bare:
            print(f"FAIL  [pids tokenizer: this file READS os."
                  f"{{{', '.join(bare)}}} directly. On a host that lacks "
                  f"it -- the host this whole section exists for -- that is "
                  f"an AttributeError traceback that takes the rest of the "
                  f"checker with it. Read it as "
                  f"`getattr(os, NAME, _ABSENT)` -- WITH the default -- and "
                  f"restore with `delattr` when it was absent.]")
            errors += 1

    probe = drive("real-host")
    if probe is None:
        errors += 1
        probe = _ProbeResult([], [], [], {}, None)
    # The shim's central promise is that it isolates a platform GAP and
    # touches nothing that answers -- which is what lets it be a no-op on
    # the hosts the sweep actually runs on. Asserted directly, by asking
    # each facility it stubbed whether it was really unusable: an
    # unconditional stubber otherwise ships green here and masks a genuine
    # usage_sampler regression on exactly those hosts. (The refuses-the-name
    # arm below used to pin this as a side effect of an exact-set oracle;
    # that oracle had to go, because it was wrong on a host lacking
    # os.uname -- so the pin is made explicit rather than incidental.)
    for _name, _reason in sorted(probe.stubbed.items()):
        _probe_fn = _OS_FACILITIES_RUN_SAMPLER_USES[_name][0]
        try:
            _probe_fn()
        except Exception:                            # noqa: BLE001
            continue                                 # genuinely unusable
        print(f"FAIL  [pids tokenizer: the shim stubbed os.{_name} on a "
              f"host that can USE it (it reported {_reason}). The shim "
              f"isolates a platform GAP; stubbing a working facility hides "
              f"a real regression on the hosts the sweep runs on.]")
        errors += 1
    for problem in probe.problems:
        print(f"FAIL  [pids tokenizer (real host): {problem}]")
        errors += 1
    for note in probe.notes:
        print(f"note  [pids tokenizer: {note}"
              + (" -- the consumer was still reached; this arm's oracle is "
                 "narrow on purpose." if probe.reached
                 else " -- and the consumer was NOT reached; see the FAIL "
                      "below.") + "]")
    if probe.stubbed:
        print("note  [pids tokenizer: "
              + ", ".join(f"os.{n} unusable here ({r})"
                          for n, r in sorted(probe.stubbed.items()))
              + " -- stubbed for the probe so a platform gap is not "
                "reported as a broken probe.]")
    # NOT `if not reached`: emptiness says the tokenizer was CALLED, never
    # with what. Measured -- a consumer that keeps its own strip rule and
    # delegates a pre-mangled spec (`args.pids.replace(" ", "")`) leaves an
    # emptiness check green while being a second tokenizer, which is the
    # original "two sites disagree about what a token IS" in a new place.
    if probe.reached != [_PIDS_PROBE_SPEC]:
        print(f"FAIL  [pids tokenizer: run_sampler handed the shared "
              f"tokenizer {probe.reached!r}, not the raw spec "
              f"[{_PIDS_PROBE_SPEC!r}] exactly once. Every case above calls "
              f"the tokenizer DIRECTLY, so the consumer -- the half that "
              f"actually disagreed -- is covered by nothing else; and a "
              f"consumer that pre-mangles the spec before delegating IS a "
              f"second tokenizer.]")
        errors += 1

    # ...and that the probe survives a host with NEITHER Unix-only facility
    # nor a readable /proc. Windows was the reported shape: `os.sysconf`
    # raises AttributeError as run_sampler's first statement, which the
    # probe classified as a malformed probe and then counted AGAIN as
    # "never reached the tokenizer" -- two FAILs on a host this arm's own
    # comments call expected, for a file whose docstring promises it runs
    # anywhere. Simulated rather than assumed: the desk cannot run Windows,
    # but it can be presented with the same absences, and a shim asserted
    # only on the host that already has the facility asserts nothing.
    with _simulated_platformless_host() as proc_reads:
        sim = drive("platform-less simulation")
        # Still INSIDE the simulation, where no Unix-only facility exists:
        # anything present now is a stand-in the shim failed to remove.
        # Nothing else asserts the restore -- measured, a `finally: pass`
        # and a restore that wrote the sentinel INTO `os` both shipped
        # green, because the enclosing scopes happen to overwrite the
        # damage on a Unix desk. On a host that genuinely lacks these, that
        # leak is a fabricated platform for every check that follows. This
        # is also the only thing that pins the `_ABSENT` sentinel.
        leaked = sorted(n for n in _UNIX_ONLY_OS_FACILITIES
                        if hasattr(os, n))
    if sim is None:
        errors += 1
        sim = _ProbeResult(["the probe raised instead of reporting"], [],
                           [], {}, None)
    # A stop the REAL-HOST arm above already saw is this host's, not this
    # file's: the same unrelated OSError (a full disk while writing the
    # sidecar) is TOLERATED there and was a hard FAIL in both arms below,
    # one of them blaming "a probe that only checks for the ATTRIBUTE" for
    # something with no connection to sysconf. Measured: injecting one
    # produced a note plus FOUR misattributed FAILs. Only stops UNIQUE to an
    # arm can be that arm's evidence -- which is what keeps the rule that
    # motivated this (a stop under the simulation is the simulation being
    # incomplete) pointed at stops the simulation itself caused.
    sim_new_notes = [n for n in sim.notes if n not in probe.notes]
    if leaked:
        print(f"FAIL  [pids tokenizer: the shim left os."
              f"{{{', '.join(leaked)}}} installed on a host that has no "
              f"such facility -- a leaked stand-in is a fabricated platform "
              f"for every arm and every check that follows.]")
        errors += 1
    sim_problems, sim_reached = sim.problems, sim.reached
    sim_notes, sim_stubbed = sim.notes, sim.stubbed
    if sim_problems or sim_reached != [_PIDS_PROBE_SPEC]:
        # The STOPS are printed here too. Without them this branch reported
        # `Problems: none` and told the operator to go isolate a platform
        # facility, while the real cause sat unprinted in `notes` -- the
        # same misattribution this section removes, pointing the other way.
        print(f"FAIL  [pids tokenizer: on a host with no Unix-only os "
              f"facility and no readable /proc the probe reported "
              f"{len(sim_problems)} problem(s) and handed the tokenizer "
              f"{sim_reached!r}. A missing platform facility is not a "
              f"contract violation -- isolate it in "
              f"_platform_prereqs_for_run_sampler instead of failing the "
              f"checker. Problems: {sim_problems or 'none'}; stops: "
              f"{sim_notes or 'none'}]")
        errors += 1
    elif sim_new_notes:
        # A note is tolerated on the REAL host (its quirks are not this
        # file's to model) and is a FAIL here, because every absence the
        # simulated host has is one this file installed: anything that
        # stops run_sampler under it is an incomplete simulation, and a
        # simulation that stops early stops testing. Measured on its first
        # run -- `platform.system()` inside the uname stub recursed into
        # the stub, and only this arm would have said so.
        print(f"FAIL  [pids tokenizer: run_sampler did not complete under "
              f"the platform-less simulation, for a reason the real host "
              f"did NOT hit ({'; '.join(sim_new_notes)}). "
              f"Every absence there is one this file installs, so the "
              f"simulation is incomplete -- model the missing facility "
              f"rather than noting it.]")
        errors += 1
    # The two anti-tautology checks below are INDEPENDENT facts about the
    # simulation's setup, not about the run's outcome, so they are their own
    # `if`s: a simulation that both failed to bite AND produced problems
    # would otherwise print only "isolate the platform facility" and never
    # say that it proved nothing.
    if sorted(sim_stubbed) != list(_UNIX_ONLY_OS_FACILITIES):
        # Anti-tautology: the simulation must actually have BITTEN. If the
        # shim reports it stubbed nothing, the arm above passed because the
        # host still had those facilities, not because the isolation works.
        # Compared against the TABLE, never a literal pair. Measured both
        # ways: pre-fix, with the simulation AND this assertion each
        # carrying a hardcoded pair, classifying a THIRD Unix-only facility
        # left the arm green; with only the simulation hardcoded, this
        # assertion catches it. Both sides derive from one tuple now, so
        # hardcoding either is caught by the other.
        print(f"FAIL  [pids tokenizer: the platform-less simulation stubbed "
              f"{sorted(sim_stubbed)}, expected "
              f"{list(_UNIX_ONLY_OS_FACILITIES)} -- the arm above proved "
              f"nothing, because the facilities it removes were still "
              f"present when the probe ran.]")
        errors += 1
    if not [n for n, pid in proc_reads if isinstance(pid, int)]:
        # The other half of the same anti-tautology. Measured: deleting all
        # the /proc reader stubs left this arm green, because a desk with no
        # /proc has the real readers answering "gone" anyway -- so the `ok`
        # line's "no readable /proc" was unbacked. The witness is what
        # run_sampler ACTUALLY asked, so the claim is only made when the
        # simulated half was really consulted.
        print(f"FAIL  [pids tokenizer: the platform-less simulation's /proc "
              f"half was never consulted -- run_sampler asked none of "
              f"{sorted(_ABSENT_PROC_READERS)}, so nothing about a host "
              f"with no readable /proc was exercised and the ok line below "
              f"would claim it anyway.]")
        errors += 1
    # The INDEPENDENT half of the identity oracle, and the one that is not
    # a self-compare. `want_uname` below is built from the same two
    # constants `_StubUname` reads, so it can only see a stand-in that
    # departs from them -- measured, fabricating the CONSTANTS instead left
    # it green while every sidecar a stubbed host writes carried a false
    # provenance line. That is reachable with no edit at all: where
    # `platform.system()` returns "", both sides become "unknown" and agree.
    # So the stubbed run is also compared against what the REAL `os.uname()`
    # wrote on this same host, one probe earlier, which shares no input with
    # the stand-in. Only when this host needed no stubbing is that real
    # reading available.
    if not probe.stubbed and probe.sidecar_uname != sim.sidecar_uname:
        print(f"FAIL  [pids tokenizer: the stand-in wrote uname="
              f"{sim.sidecar_uname!r} into the sidecar where the REAL "
              f"os.uname() on this same host wrote "
              f"{probe.sidecar_uname!r}. The want_uname check below reads "
              f"the same two constants the stand-in does, so it cannot see "
              f"an identity fabricated at that one point.]")
        errors += 1
    want_uname = f"{_HOST_SYSNAME}-{_HOST_MACHINE}"
    if sim.sidecar_uname != want_uname:
        # _StubUname promises the stubbed host still writes its OWN
        # identity. Measured: with both values set to a fabricated string
        # the arm stayed green, because the sidecar is written into a temp
        # dir nothing read back. It is read back now -- which also proves
        # the probe got PAST the parse and into the body, where os.uname()
        # is the read that used to end it.
        print(f"FAIL  [pids tokenizer: under the platform-less simulation "
              f"run_sampler wrote uname={sim.sidecar_uname!r} into its "
              f"sidecar header, not this host's own {want_uname!r}. A "
              f"stand-in that fabricates the identity puts a wrong "
              f"provenance line in every sidecar a stubbed host records.]")
        errors += 1

    # ...and the OTHER shape of the same absence, which the deletion
    # simulation above cannot reach: a host that HAS the facility and
    # refuses the name we need. POSIX lets `os.sysconf` raise for an
    # unrecognised key, and from run_sampler's point of view that is the
    # same absence -- which is why the probe classifies on the CALL rather
    # than on `hasattr`. Measured: narrowing that probe to
    # `except AttributeError` passed every other arm in this file.
    # Saved through the ABSENT sentinel, not `os.sysconf` directly: on the
    # host this whole section is about there IS no `os.sysconf`, and reading
    # it here killed the checker with a traceback -- this arm's own version
    # of the bug it exists to close (found by running the reported shape
    # end to end rather than by reading). Installing the raiser works on a
    # host that lacks the facility too, so the arm runs everywhere.
    saved_sysconf = getattr(os, "sysconf", _ABSENT)

    def _refuses_the_name(name):
        raise ValueError(f"unrecognized configuration name {name!r}")

    os.sysconf = _refuses_the_name
    try:
        refused = drive("name-refusing os.sysconf")
    finally:
        if saved_sysconf is _ABSENT:
            delattr(os, "sysconf")
        else:
            os.sysconf = saved_sysconf
    if refused is None:
        errors += 1
        refused = _ProbeResult(["the probe raised instead of reporting"],
                               [], [], {}, None)
    # `sysconf` must be IN the stubbed set, not BE it: on a host that also
    # lacks `os.uname` -- the exact host this section is about -- uname is
    # legitimately stubbed alongside. Demanding the pair was the same
    # hardcoded-host-shape mistake the derived table fixed elsewhere, and it
    # failed the arm on the shape the finding reported.
    refused_new_notes = [n for n in refused.notes if n not in probe.notes]
    if (refused.problems or refused_new_notes
            or refused.reached != [_PIDS_PROBE_SPEC]
            or (sorted(refused.stubbed)
                != sorted(set(probe.stubbed) | {"sysconf"}))):
        print(f"FAIL  [pids tokenizer: a host whose os.sysconf EXISTS but "
              f"refuses SC_CLK_TCK reported {len(refused.problems)} "
              f"problem(s), stops {refused_new_notes or 'none'} that the "
              f"real host did not hit, handed the "
              f"tokenizer {refused.reached!r} and stubbed "
              f"{sorted(refused.stubbed)}, where this host's own stubbed "
              f"set plus 'sysconf' is "
              f"{sorted(set(probe.stubbed) | {'sysconf'})}. A facility "
              f"that refuses the name we need is absent as far as "
              f"run_sampler is concerned, and a probe that only checks for "
              f"the ATTRIBUTE cannot see it.]  Problems: "
              f"{refused.problems or 'none'}")
        errors += 1

    if not errors:
        print("ok    [pids tokenizer: one tokenizer for the validator, the "
              "emptiness check and run_sampler; empty and whitespace-only "
              "slots mean the same thing, every accepted spec parses, and "
              "the probe reaches the consumer on a host with neither "
              "Unix-only os facility nor a readable /proc]")
    return errors


def check_preflight_order() -> int:
    """The no-baseline classification must answer BEFORE any preflight that
    only matters if a cell is going to run, and every leg command must refuse
    before it mutates anything.

    Two contracts, one function, because they are the same rule at two
    levels: a command that cannot proceed must say so before it changes the
    machine or claims an exit code that means something else.

    Contract A (cmd_smoke). The README, the wrapper and cmd_smoke's own
    docstring all promise 4 for every no-baseline shape. cmd_smoke ran the
    posture banner, the payload-restriction refusal and the memlock refusal
    first, and all three return 3 -- so a fresh clone on a host with a low
    memlock limit got 3, and run_benchmarks.sh ABORTED instead of skipping a
    gate it has no baseline for. Those preflights are about the quality of a
    measurement that, in these shapes, is never taken. They keep their 3 for
    every run that DOES proceed, which is the other half asserted here.

    Contract B (the leg commands). A refusal must not arrive after the run
    directory, the manifest, or -- worst -- after cleanup_iceoryx() has
    unlinked SHM segments shared with every other tenant on the machine.
    """
    errors = 0
    saved = (bench.RANGES_PATH, bench.RESULTS_ROOT,
             bench.ensure_memlock_for_zenoh, bench.ambient_payload_restriction,
             bench.cleanup_iceoryx, bench.shutil.which)

    # The DMA posture gate is a HOST property, not part of any contract this
    # function asserts, and it raises. On Linux CER_BENCH_DMA_LOCK defaults
    # to 1, so a machine without /dev/cpu_dma_latency exits through
    # require_dma_posture_deliverable before ever reaching the mocked memlock
    # refusal below -- the arm would report a contract failure that is really
    # a missing device. Stubbed for the duration, restored in the finally.
    #
    # Stubbing it does not weaken the arms: the memlock refusal, the bash
    # refusal and the docker gate are each a DIFFERENT gate, and leaving a
    # second refusal armed lets one arm pass for the wrong reason. One gate
    # per arm.
    saved_posture = (bench.require_dma_posture_deliverable,
                     bench.announce_posture)
    bench.require_dma_posture_deliverable = lambda *a, **k: None
    bench.announce_posture = lambda *a, **k: None
    try:
        return _check_preflight_order_body(errors, saved, saved_posture)
    finally:
        (bench.require_dma_posture_deliverable,
         bench.announce_posture) = saved_posture


def _check_preflight_order_body(errors, saved, saved_posture_real) -> int:
    """The arms themselves. Split out only so the DMA-posture stub in
    check_preflight_order has one `finally` covering every early return."""
    try:
        mine = bench.compute_machine_hash()
    except Exception as e:
        print(f"FAIL  [preflight order: cannot be probed — the machine hash "
              f"is unavailable ({e}). The contract is NOT implicated.]")
        return 1

    def host_block(h, body):
        return (f"schema_version: {bench.RANGES_SCHEMA_VERSION}\n"
                f"hosts:\n  {h}:\n"
                f"    machine_hash: {h}\n"
                f"    measured_on_git_sha: {'0' * 40}\n"
                f"    notes: \"fixture\"\n{body}")

    foreign = "f" * 16
    QUIESCENT = ("    rtt_p50_ns:\n      quiescent:\n"
                 "        iox2_chrt0:\n          64: [1000, 9000]\n")

    # --- Contract A ---------------------------------------------------
    # A host that CANNOT run a cell (low memlock, and a payload restriction
    # exported) still owes 4 for a shape where no cell was going to run.
    bench.ensure_memlock_for_zenoh = lambda: False
    bench.ambient_payload_restriction = lambda: [64]
    with tempfile.TemporaryDirectory(prefix="preflight_") as td:
        bench.RESULTS_ROOT = Path(td) / "results"
        shapes = [
            ("file absent", None),
            ("hosts: {}", f"schema_version: {bench.RANGES_SCHEMA_VERSION}\n"
                          f"hosts: {{}}\n"),
            ("another host on file", host_block(foreign, QUIESCENT)),
            ("own entry, no variant",
             host_block(mine, "    rtt_p50_ns:\n      quiescent: {}\n")),
        ]
        for i, (label, text) in enumerate(shapes):
            path = Path(td) / f"a{i}.yaml"
            if text is not None:
                path.write_text(text)
            bench.RANGES_PATH = path
            try:
                with contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    rc = bench.cmd_smoke(argparse.Namespace(
                        variant="quiescent", capture_baseline=False))
            except SystemExit as e:
                print(f"FAIL  [preflight order: {label} could not be probed "
                      f"— the environment refused: {e.code}]")
                errors += 1
                break
            finally:
                bench.RANGES_PATH = saved[0]
            if rc != bench.SMOKE_RC_NO_BASELINE:
                print(f"FAIL  [preflight order: with a low memlock limit and "
                      f"CER_BENCH_PAYLOAD_SIZES exported, {label} returned "
                      f"{rc}, expected {bench.SMOKE_RC_NO_BASELINE}. Those "
                      f"preflights gate the QUALITY of a measurement that is "
                      f"never taken in this shape; returning 3 makes "
                      f"run_benchmarks.sh abort where the documented "
                      f"contract says skip.]")
                errors += 1

        # The other half: once a baseline EXISTS, a cell IS eligible, and the
        # memlock refusal is a real setup failure again. Without this arm the
        # fix above is satisfied by deleting the preflight altogether.
        path = Path(td) / "have.yaml"
        path.write_text(host_block(mine, QUIESCENT))
        bench.RANGES_PATH = path
        # ISOLATE the memlock refusal. Inheriting the payload-restriction stub
        # made this arm pass for the wrong reason: that refusal also returns
        # 3, so a variant that disarms the memlock check keeps the arm green.
        bench.ambient_payload_restriction = lambda: None
        # And stop a disarmed preflight from proceeding into a real build:
        # the first call past the preflights is the build/run section.
        saved_docker = bench.docker_available
        bench.docker_available = lambda: (_ for _ in ()).throw(
            _CaptureProceeded())
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                rc = bench.cmd_smoke(argparse.Namespace(
                    variant="quiescent", capture_baseline=False))
        except _CaptureProceeded:
            rc = "ran past the preflights into the build section"
        except SystemExit as e:
            rc = f"SystemExit({e.code})"
        finally:
            bench.RANGES_PATH = saved[0]
            bench.docker_available = saved_docker
            bench.ambient_payload_restriction = lambda: [64]
        if rc != 3:
            print(f"FAIL  [preflight order: with a baseline ON FILE and a low "
                  f"memlock limit, cmd_smoke returned {rc}, expected 3. A "
                  f"cell was eligible to run, so the memlock refusal is a "
                  f"genuine setup failure — hoisting the classification must "
                  f"not disarm the preflight for runs that DO proceed.]")
            errors += 1

    bench.ensure_memlock_for_zenoh = saved[2]
    bench.ambient_payload_restriction = saved[3]
    bench.RESULTS_ROOT = saved[1]

    # --- Contract B ---------------------------------------------------
    swept = []
    bench.cleanup_iceoryx = lambda: swept.append(1)
    real_which = saved[5]
    bench.shutil.which = (lambda n, *a, **k:
                          None if n == "bash" else real_which(n, *a, **k))
    with tempfile.TemporaryDirectory(prefix="preflight_b_") as td:
        run_dir = Path(td) / "run"
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                rc = bench.cmd_workspace(argparse.Namespace(
                    variant="quiescent", rep=1, run_dir=str(run_dir),
                    chrt="0", leg=None, sizes=None, reps=1))
        except SystemExit:
            rc = "SystemExit"
        except Exception as e:                       # noqa: BLE001
            rc = f"{type(e).__name__}: {e}"
        finally:
            bench.shutil.which = real_which
            bench.cleanup_iceoryx = saved[4]
        created = sorted(x.name for x in Path(td).rglob("*"))
        if rc != 3:
            print(f"FAIL  [preflight order: bash-less `workspace --run-dir X` "
                  f"returned {rc!r}, expected 3.]")
            errors += 1
        if created:
            print(f"FAIL  [preflight order: bash-less `workspace --run-dir X` "
                  f"created {created} before refusing. resolve_run_dir returns "
                  f"early on --run-dir, so nothing upstream checks bash; the "
                  f"artifacts describe a run that never happened.]")
            errors += 1
        if swept:
            print(f"FAIL  [preflight order: bash-less `workspace --run-dir X` "
                  f"ran cleanup_iceoryx() before refusing — that unlinks SHM "
                  f"segments shared with every other tenant on the host, and "
                  f"they are not ours to destroy on the way to failing.]")
            errors += 1

    # The DIRECT caller. cmd_workspace now refuses on its own, so the guard
    # inside run_workspace_leg is invisible from the arm above -- and that
    # guard is the one `_run_smoke_cells` depends on, since it calls the leg
    # helper without going through cmd_workspace at all. Driven separately,
    # or the leg-level refusal is pinned by nothing.
    swept_direct = []
    bench.cleanup_iceoryx = lambda: swept_direct.append(1)
    bench.shutil.which = (lambda n, *a, **k:
                          None if n == "bash" else real_which(n, *a, **k))
    with tempfile.TemporaryDirectory(prefix="preflight_c_") as td:
        raw = Path(td) / "raw"
        raw.mkdir()
        refused = None
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                bench.run_workspace_leg("iox2", 0, "quiescent", raw,
                                        raw / "_logs", 1)
        except SystemExit as e:
            refused = str(e.code)
        except Exception as e:                       # noqa: BLE001
            refused = f"{type(e).__name__}: {e}"
        finally:
            bench.shutil.which = real_which
            bench.cleanup_iceoryx = saved[4]
        if refused is None or "bash" not in refused:
            print(f"FAIL  [preflight order: a DIRECT run_workspace_leg call "
                  f"on a bash-less host did not refuse for bash "
                  f"({refused!r}). _run_smoke_cells calls this helper "
                  f"directly, bypassing cmd_workspace's preflight.]")
            errors += 1
        if swept_direct:
            print(f"FAIL  [preflight order: a DIRECT run_workspace_leg call "
                  f"ran cleanup_iceoryx() before refusing for bash — the SHM "
                  f"sweep destroys state shared with every other tenant on "
                  f"the machine.]")
            errors += 1

    # cmd_ros2, the third leg. The sweep for item 2's class found the same
    # defect here -- ensure_docker_image ran AFTER resolve_run_dir,
    # raw_dir.mkdir and the RunManifest -- and no arm covered it.
    saved_docker = bench.docker_available
    bench.docker_available = lambda: False
    with tempfile.TemporaryDirectory(prefix="preflight_d_") as td:
        run_dir = Path(td) / "run"
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                rc = bench.cmd_ros2(argparse.Namespace(
                    variant="quiescent", rep=1, run_dir=str(run_dir),
                    chrt="0", distro=None, cells=None, sizes=None,
                    build_image=False, reps=1))
        except SystemExit as e:
            rc = f"SystemExit({e.code})"
        except Exception as e:                       # noqa: BLE001
            rc = f"{type(e).__name__}: {e}"
        finally:
            bench.docker_available = saved_docker
        created = sorted(x.name for x in Path(td).rglob("*"))
        if rc != 3:
            print(f"FAIL  [preflight order: `ros2` on a docker-less host "
                  f"returned {rc!r}, expected 3.]")
            errors += 1
        if created:
            print(f"FAIL  [preflight order: `ros2` on a docker-less host "
                  f"created {created} before refusing — ensure_docker_image "
                  f"refuses only after the run dir and the manifest exist.]")
            errors += 1

    # The env-contract class: announce_posture and ambient_payload_restriction
    # sit below the classification, and their PARSING moves with them, so on
    # a no-baseline host a malformed
    # CER_BENCH_DMA_LOCK or CER_BENCH_PAYLOAD_SIZES stopped being an exit-1
    # violation and became a 4 -- the code run_benchmarks.sh deliberately
    # SWALLOWS, so a typo in an exported variable silently continued a run.
    # Three cases, three answers, and no arm covered any of them.
    with tempfile.TemporaryDirectory(prefix="envcontract_") as td:
        bench.RESULTS_ROOT = Path(td) / "results"
        absent = Path(td) / "absent.yaml"
        have = Path(td) / "have.yaml"
        have.write_text(host_block(mine, QUIESCENT))
        bench.ensure_memlock_for_zenoh = lambda: True
        bench.ambient_payload_restriction = saved[3]

        def drive(env, path):
            for k, v in env.items():
                os.environ[k] = v
            bench.RANGES_PATH = path
            saved_docker = bench.docker_available
            bench.docker_available = lambda: (_ for _ in ()).throw(
                _CaptureProceeded())
            try:
                with contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    return bench.cmd_smoke(argparse.Namespace(
                        variant="quiescent", capture_baseline=False))
            except _CaptureProceeded:
                return "proceeded to the build section"
            except SystemExit:
                return 1          # Python's code for SystemExit(<message>)
            except Exception as e:                   # noqa: BLE001
                return f"{type(e).__name__}: {e}"
            finally:
                for k in env:
                    os.environ.pop(k, None)
                bench.RANGES_PATH = saved[0]
                bench.docker_available = saved_docker

        # A malformed value is a malformed ASK -- same answer on every path.
        for var in ("CER_BENCH_PACING", "CER_BENCH_DMA_LOCK",
                    "CER_BENCH_PAYLOAD_SIZES"):
            for shape, path in (("no baseline", absent),
                                ("baseline on file", have)):
                rc = drive({var: "bogus"}, path)
                if rc != 1:
                    print(f"FAIL  [preflight order: a malformed {var} with "
                          f"{shape} returned {rc!r}, expected 1. A malformed "
                          f"value is a contract violation on every path; "
                          f"answering 4 hands it to run_benchmarks.sh, which "
                          f"SWALLOWS 4 as 'no baseline, skip the gate' -- so "
                          f"a typo in an exported variable would silently "
                          f"continue the run instead of aborting it.]")
                    errors += 1

        # ...and a WELL-FORMED but restrictive value keeps that fix's answer,
        # which is the distinction the fix must not flatten: 4 where no cell
        # was going to run, 3 where one was.
        for shape, path, want in (("no baseline", absent,
                                   bench.SMOKE_RC_NO_BASELINE),
                                  ("baseline on file", have, 3)):
            rc = drive({"CER_BENCH_PAYLOAD_SIZES": "64"}, path)
            if rc != want:
                print(f"FAIL  [preflight order: a WELL-FORMED "
                      f"CER_BENCH_PAYLOAD_SIZES=64 with {shape} returned "
                      f"{rc!r}, expected {want}. Validating the env early "
                      f"must not drag the curated-subset REFUSAL up with it "
                      f"-- that refusal is about a measurement, and in the "
                      f"no-baseline shape none is taken.]")
                errors += 1

        # cmd_full, the dispatcher. It gates CER_BENCH_DMA_LOCK before
        # resolve_run_dir (via require_dma_posture_deliverable) but did not
        # gate CER_BENCH_PAYLOAD_SIZES, so a malformed one NAMED a run
        # directory and only then exited 1 from the first sweep it
        # dispatched into. Same class as the leg commands, one level up.
        # The posture gate stays STUBBED for these arms, deliberately: cmd_full
        # now validates DMA posture and payload size itself, through the
        # explicit dma_lock_enabled()/ambient_payload_restriction() pair,
        # which is what these arms exist to pin.
        #
        # Restoring the gate is now actively WRONG on the host that matters.
        # require_dma_posture_deliverable opens `if not IS_LINUX: return`, so
        # on this desk it is inert and the arms reach the validators either
        # way -- but on a Linux host without /dev/cpu_dma_latency it RAISES,
        # and it raises before ambient_payload_restriction() is ever called.
        # Both arms would then pass on the gate rather than on the validator,
        # and cmd_full could lose its payload validation with this checker
        # green. A green that depends on the host having a device is not a
        # green.
        #
        # So: stub the gate, and keep ensure_built stubbed as the backstop
        # that made restoring it look necessary in the first place.
        saved_full = (bench.require_dma_posture_deliverable,
                      bench.announce_posture, bench.ensure_built)
        bench.ensure_built = lambda *a, **k: (_ for _ in ()).throw(
            _CaptureProceeded())
        _ = saved_posture_real   # the real pair, deliberately NOT restored
        for var in ("CER_BENCH_DMA_LOCK", "CER_BENCH_PAYLOAD_SIZES"):
            os.environ[var] = "bogus"
            full_dir = Path(td) / f"full-{var}"
            # The observable is the BANNER, not the filesystem: with
            # --run-dir, resolve_run_dir returns the path without creating
            # it, and the first leg cmd_full dispatches into validates the
            # same variable a moment later -- so an exit code alone cannot
            # tell cmd_full's gate from the leg's, and an arm that checked
            # only the code passed with cmd_full's gate deleted (measured).
            # cmd_full's own comment states the contract exactly: "a refused
            # `full` touches nothing and NAMES nothing".
            full_out = io.StringIO()
            try:
                with contextlib.redirect_stdout(full_out), \
                        contextlib.redirect_stderr(io.StringIO()):
                    rc = bench.cmd_full(argparse.Namespace(
                        variant="quiescent", reps=1, run_dir=str(full_dir),
                        chrt="0", sizes=None, leg=None, distro=None,
                        cells=None, build_image=False, rep=1))
            except SystemExit:
                rc = 1
            except _CaptureProceeded:
                rc = "reached the build section"
            except Exception as e:                   # noqa: BLE001
                rc = f"{type(e).__name__}: {e}"
            finally:
                os.environ.pop(var, None)
            if rc != 1:
                print(f"FAIL  [preflight order: `full` with a malformed "
                      f"{var} returned {rc!r}, expected 1.]")
                errors += 1
            if full_dir.exists() or full_dir.name in full_out.getvalue():
                print(f"FAIL  [preflight order: `full` with a malformed "
                      f"{var} NAMED {full_dir.name} before refusing. Its own "
                      f"comment promises a refused `full` touches nothing "
                      f"and names nothing; without the gate it resolves and "
                      f"announces a run directory, and only the sweep it "
                      f"dispatches into then exits 1 on the same variable.]")
                errors += 1

        (bench.require_dma_posture_deliverable, bench.announce_posture,
         bench.ensure_built) = saved_full

    bench.ensure_memlock_for_zenoh = saved[2]
    bench.RESULTS_ROOT = saved[1]

    if not errors:
        print("ok    [preflight order: the four no-baseline shapes answer 4 "
              "even when the memlock limit and an exported payload "
              "restriction would refuse a run; a host WITH a baseline still "
              "gets 3 from that same memlock refusal; and a bash-less "
              "workspace leg refuses without creating a run dir, a manifest, "
              "or wiping shared memory; a MALFORMED env var is exit 1 on "
              "every path while a well-formed restrictive one keeps 4/3]")
    return errors


def check_smoke_exit_contract() -> int:
    """`bench.py smoke`'s no-baseline status is ONE number, in two files.

    A baseline is per (machine_hash, variant) and is captured on the
    machine, so a fresh clone cannot have one — a normal first-run state,
    not a failure. It has its own exit code precisely so
    `scripts/run_benchmarks.sh` can SKIP the smoke gate without also
    tolerating a build failure. That makes the number a contract spanning a
    Python module and a shell script, written as a literal in each; a
    one-sided edit is silent and total, so the two literals are compared.

    Every arm drives a SYNTHETIC ranges file in a temp dir and never the
    live `RANGES_PATH`. That is not tidiness: `expected-ranges.yaml` is
    designed to accumulate host entries, and on any machine that has run
    `--capture-baseline` — which is every machine where the gate means
    anything — this host's entry is present, `host_ranges` is truthy, and
    `cmd_smoke` falls past BOTH returns into `shutil.rmtree(results/smoke)`,
    a full cargo build, the real cell subset, and `kill_stragglers`, with
    stdout and stderr swallowed by the redirects below. A contract check
    must not delete a prior run's raws or kill a concurrent campaign.

    ONE gate ahead of the baseline lookup can still return 3: the machine
    hash (a malformed ranges file is the other, and these arms write their
    own). The ambient-payload refusal and the memlock raise used to sit there
    too and no longer do -- a review fix moved both below the classification,
    because they gate the quality of a measurement that these shapes never
    take. What DID stay ahead of it is the env-contract VALIDATION, and that
    exits 1, never 3.

    So the first arm still doubles as a probe -- it MUST answer 4, and a 3
    means the arms below cannot be attributed -- but the set of things a 3
    can mean is now much smaller, and the check says which rather than
    blaming the contract."""
    errors = 0
    wrapper = REPO_ROOT / "tools" / "scripts" / "run_benchmarks.sh"
    m = re.search(r"^SMOKE_RC_NO_BASELINE=(\d+)$", wrapper.read_text(),
                  re.MULTILINE)
    if m is None:
        print(f"FAIL  [smoke exit contract: no SMOKE_RC_NO_BASELINE=<n> line "
              f"in {wrapper.name} — the wrapper cannot be checked against "
              f"bench.py]")
        return 1
    if int(m.group(1)) != bench.SMOKE_RC_NO_BASELINE:
        print(f"FAIL  [smoke exit contract: {wrapper.name} says "
              f"{m.group(1)}, bench.py says {bench.SMOKE_RC_NO_BASELINE} — "
              f"the wrapper's skip branch will never fire]")
        errors += 1
    if bench.SMOKE_RC_NO_BASELINE in (0, 2, 3):
        print(f"FAIL  [smoke exit contract: the no-baseline code is "
              f"{bench.SMOKE_RC_NO_BASELINE}, which already means something "
              f"else (0=pass, 2=regression, 3=setup failure)]")
        errors += 1

    # `reproduce.sh` asks the same question before offering a capture, with
    # its own copy of the predicate. The gate treats a present-but-EMPTY
    # variant map as no baseline (falsy, not `is None`); a mirror that used
    # `is None` would suppress the offer and then be refused by the gate.
    repro = REPO_ROOT / "tools" / "scripts" / "benchmarks" / "reproduce.sh"
    if 'bool((host.get("rtt_p50_ns") or {}).get(variant))' not in \
            repro.read_text():
        print(f"FAIL  [smoke exit contract: {repro.name}::baseline_present no "
              f"longer mirrors cmd_smoke's falsy baseline predicate — the "
              f"capture offer and the gate can now disagree]")
        errors += 1

    foreign = "0" * 16
    try:
        mine = bench.compute_machine_hash()
    except Exception as e:
        # Same wording as the in-loop blocker probe, and for the same reason:
        # a checker that cannot run must say the contract is NOT implicated,
        # or an environment fault reads as a contract regression. A traceback
        # here also swallows main()'s own "FAIL: N divergence(s)" line.
        print(f"FAIL  [smoke exit contract: cannot be probed — the machine "
              f"hash is unavailable ({e}), and that gate answers before the "
              f"baseline lookup. The contract is NOT implicated; fix the "
              f"environment and re-run.]")
        return errors + 1

    def host_block(h, body):
        return (f"schema_version: {bench.RANGES_SCHEMA_VERSION}\n"
                f"hosts:\n  {h}:\n"
                f"    machine_hash: {h}\n"
                f"    measured_on_git_sha: {'0' * 40}\n"
                f"    notes: \"fixture\"\n{body}")

    # (label, ranges-file text or None, expected rc, expected signals).
    # The signals are named per arm rather than keyed on a position. A bare
    # index (`i == 2`) inverts into a false failure the moment an arm is
    # added or reordered -- and it fails in the WRONG direction: the text
    # blames the production contract for what is a fixture edit. Measured:
    # swapping two arms with bench.py untouched reported the drift warning
    # as both missing and spurious.
    #
    # The signals are asserted because with ONE exit code for every
    # no-baseline shape they are the only thing separating a possibly-drifted
    # machine from a host that simply never captured.
    #
    # The POSTURE line is the one the old fixture could not see: it goes to
    # STDOUT while every other line in that branch is stderr. Measured on the
    # parent commit — deleting print_governor_state_loudly() left the WHOLE
    # suite green while its own summary still claimed the shared-file arm
    # printed the posture. The drift SENTENCE was already pinned even then,
    # so what capturing stdout adds is the posture line, not the warning as a
    # whole; the earlier wording here over-claimed that and was corrected.
    arms = [
        ("file absent", None, bench.SMOKE_RC_NO_BASELINE,
         {"governor", "notfound"}),
        ("hosts: {} (what ships)",
         f"schema_version: {bench.RANGES_SCHEMA_VERSION}\nhosts: {{}}\n",
         bench.SMOKE_RC_NO_BASELINE, {"governor", "fresh"}),
        # A shared file holding only OTHER hosts is the file's DESIGNED
        # steady state ("Multiple hosts coexist under `hosts:`"), so it takes
        # the skippable code like every other shape — otherwise the
        # documented default command fails on every machine except the one
        # that captured. Drift is POSSIBLE here (this host's key is absent),
        # so this is the one arm that gets the drift text and the posture.
        ("another host on file",
         host_block(foreign, "    rtt_p50_ns:\n      quiescent:\n"
                             "        iox2_chrt0:\n          64: [1000, 9000]\n"),
         bench.SMOKE_RC_NO_BASELINE, {"drift", "governor"}),
        # This host's OWN key is present, so identity drift is excluded by
        # construction and the fault is in the YAML. Claiming drift here
        # sends the operator to check a governor that is not the problem.
        ("own entry, empty variant map",
         host_block(mine, "    rtt_p50_ns:\n      quiescent: {}\n"),
         bench.SMOKE_RC_NO_BASELINE, {"own_entry", "governor"}),
        ("own entry, other variant only",
         host_block(mine, "    rtt_p50_ns:\n      fixed100:\n"
                          "        iox2_chrt0:\n          64: [1, 2]\n"),
         bench.SMOKE_RC_NO_BASELINE, {"own_entry", "governor"}),
        # The file's REAL steady state on a shared bench fleet: this host's
        # entry is on file AND other hosts are too. `others` is empty in both
        # arms above, so neither can tell a drift gate keyed on `host_entry`
        # from one that also demands `not others` -- and the latter re-prints
        # the identity-drift claim on exactly the operator this PR fixed.
        ("own entry AND other hosts on file",
         host_block(mine, "    rtt_p50_ns:\n      quiescent: {}\n")
         + f"  {foreign}:\n"
           f"    machine_hash: {foreign}\n"
           f"    measured_on_git_sha: {'0' * 40}\n"
           f"    notes: \"fixture\"\n"
           f"    rtt_p50_ns:\n      quiescent:\n"
           f"        iox2_chrt0:\n          64: [1000, 9000]\n",
         bench.SMOKE_RC_NO_BASELINE, {"own_entry", "governor"}),
    ]
    # Each arm below catches a defect no other arm catches — measured, not
    # assumed: arm "file absent" is the sole cover for the FIRST return site;
    # "hosts: {}" for a spurious drift claim on a valid empty file; "own
    # entry, empty variant map" for the falsy-not-`is None` predicate ({} is
    # the only shape where those differ); "own entry, other variant only" for
    # per-VARIANT keying; "own entry AND other hosts" for the branch
    # precedence. Only two of them carry a signal no other arm carries, so the
    # coverage guard below cannot notice the other three going missing —
    # hence an explicit count, so pruning an apparent duplicate is deliberate.
    EXPECTED_ARMS = 6
    if len(arms) != EXPECTED_ARMS:
        print(f"FAIL  [smoke exit contract: {len(arms)} arms, expected "
              f"{EXPECTED_ARMS}. Four arms carry no unique signal, so the "
              f"coverage loop below cannot see them deleted — each is the "
              f"sole cover for its own production mutant. Change the count "
              f"deliberately, with the reason.]")
        errors += 1

    MARKERS = {
        "drift": ("its identity", "stderr"),
        "own_entry": ("entry IS on file", "stderr"),
        "fresh": ("no host entries at all", "stderr"),
        "notfound": ("not found — nothing has been captured", "stderr"),
        # The posture rides EVERY no-baseline exit: the shapes whose remedy
        # is "re-capture" are precisely the ones that must not capture on a
        # powersave governor, and cmd_smoke prints no posture anywhere else.
        "governor": ("[env] cpu governor:", "stdout"),
    }
    for sig in MARKERS:
        if not any(sig in a[3] for a in arms):
            print(f"FAIL  [smoke exit contract: no arm expects the {sig!r} "
                  f"signal, so its absence is checked and its PRESENCE is "
                  f"not -- a bench.py that prints nothing would pass. "
                  f"Deleting an arm must not silently delete a check.]")
            errors += 1

    ns = argparse.Namespace(variant="quiescent", capture_baseline=False)
    saved = bench.RANGES_PATH
    saved_results = bench.RESULTS_ROOT
    # Same host-property hazard as check_preflight_order: the
    # --capture-baseline arms below deliberately proceed PAST the
    # classification, so on Linux (CER_BENCH_DMA_LOCK defaults to 1) a machine
    # without /dev/cpu_dma_latency would exit through
    # require_dma_posture_deliverable and report a contract failure that is
    # really a missing device.
    saved_posture = (bench.require_dma_posture_deliverable,
                     bench.announce_posture)
    # `cmd_smoke` sets bench's PER-INVOCATION state (`set_dma_basis`, and
    # the refusal args / probe resolved beside it). `main()` clears those
    # per invocation; nothing clears them for an IN-PROCESS caller, and
    # MEASURED, a healthy run of this scenario leaves `_DMA_BASIS` at
    # "host". `set_dma_basis` REFUSES a host->container transition, so a
    # later container-basis check in the same process would die on state
    # this scenario installed.
    saved_invocation = (bench._DMA_BASIS, bench._REFUSAL_ARGS,
                        bench._DMA_PROBE)
    bench.require_dma_posture_deliverable = lambda *a, **k: None
    bench.announce_posture = lambda *a, **k: None
    # Every mutation above is undone in a `finally`, like the other
    # scenarios in this file. Without it an unexpected throw anywhere in
    # the arms below leaves bench.RESULTS_ROOT pointing at a deleted temp
    # directory and both posture gates stubbed out for EVERY later check
    # in this process — a cascade the report would then attribute to
    # whichever check happened to run next.
    try:
        with tempfile.TemporaryDirectory(prefix="smokerc_") as td:
            # Redirect the RESULTS root as well, not just the ranges file. Every
            # arm is supposed to return before any build -- but a regression that
            # breaks exactly that is what this function exists to catch, and on
            # such a regression cmd_smoke reaches `rmtree(RESULTS_ROOT/"smoke")`
            # and then cargo. The check must survive being right about a bug.
            bench.RESULTS_ROOT = Path(td) / "results"
            for i, (label, text, want, want_signals) in enumerate(arms):
                path = Path(td) / f"ranges-{i}.yaml"
                if text is not None:
                    path.write_text(text)
                bench.RANGES_PATH = path
                arm_out, arm_err = io.StringIO(), io.StringIO()
                fell_through = []
                saved_docker = bench.docker_available
                bench.docker_available = lambda: (fell_through.append(label),
                                                  _stop_before_building())[0]
                try:
                    with contextlib.redirect_stdout(arm_out), \
                            contextlib.redirect_stderr(arm_err):
                        rc = bench.cmd_smoke(ns)
                except _CaptureProceeded:
                    # The arm fell past the no-baseline return. Recorded below;
                    # raising is what CONTAINS it -- a guard that only records
                    # lets the arm continue into ensure_built and a real cargo
                    # build, which is how this check first ran for minutes
                    # instead of milliseconds when the guard falls through.
                    #
                    # rc is DELIBERATELY set to a sentinel rather than left
                    # unbound. Leaving it unbound had two failure modes, and the
                    # second is the dangerous one: on the FIRST arm the next
                    # comparison raises UnboundLocalError (loud, but it reports a
                    # harness crash where a contract violation was found); on
                    # EVERY LATER arm `rc` is still bound from the PREVIOUS
                    # iteration, so the rc oracle silently compares this arm's
                    # contract against the last arm's exit code. That is why the
                    # fall-through variant looked like it reported
                    # cleanly -- it did, by reading a stale value that happened
                    # to match. The arm is skipped outright below.
                    rc = None
                except SystemExit as e:
                    # Reachable, not defensive: check_ambient_pacing and
                    # dma_lock_enabled raise SystemExit(<str>) on a contradictory
                    # CER_BENCH_PACING / CER_BENCH_DMA_LOCK export — the normal
                    # shell state mid-campaign. e.code is then a STRING.
                    print(f"FAIL  [smoke exit contract: {label} could not be "
                          f"probed — the environment refused the run: {e.code}]")
                    errors += 1
                    break
                finally:
                    bench.RANGES_PATH = saved
                    bench.docker_available = saved_docker
                if fell_through:
                    print(f"FAIL  [smoke exit contract: {label} ran PAST the "
                          f"no-baseline return into the build/run section. The "
                          f"docstring's 'costs seconds, not minutes' promise is "
                          f"broken, and on a real box that path rmtree's "
                          f"results/smoke and shells out to cargo.]")
                    errors += 1
                    # No exit code was produced and the streams stop wherever the
                    # guard fired, so every downstream oracle here would be
                    # judging a run that did not finish. Skip the arm; the
                    # failure above is the finding.
                    continue
                if label == "file absent" and rc != want:
                    # DO NOT blame the environment without checking it: a
                    # checker that guessed would be the very defect this PR kept
                    # finding, a message asserting a cause it cannot know. The
                    # ambiguity is much narrower than it was -- since that fix
                    # only the machine hash and a malformed ranges file return 3
                    # ahead of the classification, and the env-contract gates
                    # that used to sit there now exit 1 instead. The probes below
                    # are kept anyway: they are cheap, they name a cause instead
                    # of asserting one, and a future reorder that puts a
                    # 3-returning gate back in front would otherwise silently
                    # turn this arm into a mystery. (ensure_memlock_for_zenoh is
                    # the only one with side effects, and cmd_smoke has already
                    # run it on this same call, so asking costs nothing new.)
                    blocker = None
                    if ambient := bench.ambient_payload_restriction():
                        blocker = f"CER_BENCH_PAYLOAD_SIZES is exported ({ambient})"
                    elif not bench.ensure_memlock_for_zenoh():
                        blocker = "the memlock limit is too low for zenoh-SHM"
                    else:
                        try:
                            bench.compute_machine_hash()
                        except Exception as e:
                            blocker = f"the machine hash is unavailable ({e})"
                    if blocker is not None:
                        print(f"FAIL  [smoke exit contract: cannot be probed — "
                              f"{blocker}, and that gate answers before the "
                              f"baseline lookup. The contract is NOT implicated; "
                              f"fix the environment and re-run.]")
                    else:
                        print(f"FAIL  [smoke exit contract: the {label} site "
                              f"returned {rc}, expected {want} — and every gate "
                              f"ahead of it is clear, so this IS the contract: a "
                              f"no-baseline state is being reported as a setup "
                              f"failure again.]")
                    errors += 1
                    break
                if rc != want:
                    print(f"FAIL  [smoke exit contract: {label} returned {rc}, "
                          f"expected {want}]")
                    errors += 1
                streams = {"stdout": arm_out.getvalue(), "stderr": arm_err.getvalue()}
                for name, (marker, stream) in MARKERS.items():
                    seen = marker in streams[stream]
                    if seen != (name in want_signals):
                        print(f"FAIL  [smoke exit contract: {label} "
                              f"{'emitted' if seen else 'did NOT emit'} the "
                              f"{name!r} signal on {stream} — with one exit code "
                              f"for every no-baseline shape these lines are the "
                              f"only thing telling them apart]")
                        errors += 1
            # The mirror of every arm above. A capture must NOT take the
            # no-baseline early return -- if it did, `--capture-baseline` would
            # report success while writing nothing, and the operator would
            # believe they had a baseline they do not have. Driven on the
            # file-absent shape, the one a fresh clone actually hits.
            for cap_label, cap_text in (("file absent", None),
                                        ("hosts: {} (what ships)",
                                         f"schema_version: "
                                         f"{bench.RANGES_SCHEMA_VERSION}\n"
                                         f"hosts: {{}}\n")):
                cap_path = Path(td) / f"cap-{len(cap_label)}.yaml"
                if cap_text is not None:
                    cap_path.write_text(cap_text)
                bench.RANGES_PATH = cap_path
                proceeded = []
                # Stub every gate the capture path meets BEFORE docker, not just
                # docker. cmd_smoke reaches ensure_memlock_for_zenoh and
                # ambient_payload_restriction first, and both return 3, so on a
                # low-memlock host (or one with CER_BENCH_PAYLOAD_SIZES exported)
                # `proceeded` stayed empty and this arm reported a contract
                # failure that was really the host. The gate under test here is
                # the `and not args.capture_baseline` conjunct -- nothing else --
                # and leaving a second refusal armed lets an arm pass for the
                # wrong reason. One gate per
                # arm; the production gates are untouched.
                saved_cap = (bench.docker_available,
                             bench.ensure_memlock_for_zenoh,
                             bench.ambient_payload_restriction)
                bench.ensure_memlock_for_zenoh = lambda: True
                bench.ambient_payload_restriction = lambda: None
                bench.docker_available = lambda: (proceeded.append(True),
                                                  _stop_before_building())[0]
                try:
                    with contextlib.redirect_stdout(io.StringIO()), \
                            contextlib.redirect_stderr(io.StringIO()):
                        bench.cmd_smoke(argparse.Namespace(
                            variant="quiescent", capture_baseline=True))
                except _CaptureProceeded:
                    pass
                except Exception as e:                   # noqa: BLE001
                    # Kept broad -- the capture path is long and this arm cares
                    # about ONE thing, whether it got past the early return --
                    # but no longer SILENT. With the gates above stubbed, a
                    # throw here is a real surprise, and reporting it as "took
                    # the early return" (which is what `pass` did) would blame
                    # the contract for something else entirely.
                    cap_surprise = f"{type(e).__name__}: {e}"
                    print(f"note  [smoke exit contract: --capture-baseline on "
                          f"'{cap_label}' raised {cap_surprise[:90]} -- not the "
                          f"early return this arm tests; read it before the "
                          f"verdict below.]")
                finally:
                    (bench.docker_available, bench.ensure_memlock_for_zenoh,
                     bench.ambient_payload_restriction) = saved_cap
                    bench.RANGES_PATH = saved
                if not proceeded:
                    print(f"FAIL  [smoke exit contract: --capture-baseline on "
                          f"'{cap_label}' took the no-baseline early return "
                          f"instead of proceeding to the capture. That return is "
                          f"guarded by `and not args.capture_baseline` on BOTH "
                          f"sites; without it the one remedy the gate, the "
                          f"wrapper and reproduce.sh all name is inert, and a "
                          f"capture reports success having written nothing.]")
                    errors += 1

    finally:
        bench.RANGES_PATH = saved
        bench.RESULTS_ROOT = saved_results
        (bench.require_dma_posture_deliverable,
         bench.announce_posture) = saved_posture
        (bench._DMA_BASIS, bench._REFUSAL_ARGS,
         bench._DMA_PROBE) = saved_invocation

    if not errors:
        print(f"ok    [smoke exit contract: bench.py and run_benchmarks.sh "
              f"both say {bench.SMOKE_RC_NO_BASELINE}; reproduce.sh mirrors "
              f"the predicate; all four no-baseline shapes get it across "
              f"{len(arms)} arms, each naming which shape it hit and each "
              f"carrying the CPU posture; only the shared-file one warns "
              f"about drift; an own-entry gap says so instead, with or "
              f"without other hosts on file; and --capture-baseline "
              f"proceeds rather than skipping]")
    return errors


# `bench` attributes that legitimately differ across a call because they
# are CACHES the scenario may legitimately fill. Named, so the identity
# diff below can be total over everything else rather than a hand list of
# four names that goes stale the day a fifth global is stubbed.
# Each entry is a MEMOISED value whose only effect is to skip recomputing
# itself -- verified by reading its one writer in bench.py, not assumed
# from the name. A stubbed function or a redirected path does NOT belong
# here: those change what later checks measure, which is the whole point
# of this arm.
_SMOKE_RESTORE_MUTABLE = {
    "_CLI_SOURCE_FLOOR_CACHE",     # memoised source-floor mtime
    "_MACHINE_HASH_CACHE",         # memoised host identity (bench.py:1543)
}


def _check_smoke_allowlist_is_minimal() -> int:
    """Every allowlisted name must be a name `bench` actually declares.

    The allowlist is the one place this arm can be weakened without looking
    weakened: a name added to it stops being watched, and a name left in it
    after the global is renamed silently covers nothing. Both show up as a
    name `bench` does not have."""
    stale = sorted(n for n in _SMOKE_RESTORE_MUTABLE if not hasattr(bench, n))
    if stale:
        print(f"FAIL  [smoke contract restore: the mutable-state allowlist "
              f"names {stale}, which `bench` does not declare — an entry "
              f"that matches nothing watches nothing, and the next global "
              f"to be stubbed would go unnoticed]")
        return 1
    return 0


def _bench_identity_snapshot() -> dict:
    """Every `bench` module attribute, by IDENTITY, minus the declared
    caches. Derived rather than listed: the check below has to notice a
    global the scenario stubs TOMORROW, not only the four it stubs today."""
    return {n: v for n, v in vars(bench).items()
            if n not in _SMOKE_RESTORE_MUTABLE}


def _bench_identity_drift(before: dict) -> list:
    after = _bench_identity_snapshot()
    drifted = [n for n, v in before.items()
               if n not in after or after[n] is not v]
    return sorted(drifted + [n for n in after if n not in before])


def check_smoke_contract_restores_on_a_throw() -> int:
    """`check_smoke_exit_contract` must hand the module back unchanged even
    when something inside it throws.

    It rebinds `bench` globals — RANGES_PATH, RESULTS_ROOT, the two posture
    gates, and three more inside its `--capture-baseline` phase — and
    before this arm existed the outer ones were restored by plain
    statements after the `with`, which an escaping exception skips. What
    ships then is not one failed check: RESULTS_ROOT points at a DELETED
    temp directory and the posture gates stay stubbed for every later check
    in the process, so the report blames whichever check ran next.

    TWO phases, because the scenario has two `finally`s and one throw
    cannot reach both:

    (a) a sentinel raised from the FIRST `bench.cmd_smoke` call, which
        escapes before the capture phase is ever entered;
    (b) a `SystemExit` raised only when `capture_baseline` is set — the
        class that actually escapes there, since the capture loop's own
        handler catches `Exception` and `SystemExit` is not one. That is
        not hypothetical: `cmd_smoke` raises `SystemExit` on a
        contradictory `CER_BENCH_PACING` / `CER_BENCH_DMA_LOCK` export,
        which is ordinary shell state mid-campaign.

    ...and the HEALTHY run, which neither throw phase can see.

    The oracle is an IDENTITY snapshot of the whole `bench` namespace taken
    before each phase — not a list of names, which is the shape this file's
    own comments call the failure mode this tree keeps paying for.

    COST, stated because it is deliberate: this drives
    `check_smoke_exit_contract` THREE times (plus `main()`'s own call).
    That scenario is arms-only — every one of them returns before any
    build, which is the property it exists to assert — so the four runs
    together are a few hundred milliseconds, not the minutes a
    fall-through would cost."""
    errors = _check_smoke_allowlist_is_minimal()
    real_cmd_smoke = bench.cmd_smoke
    sentinel = RuntimeError("restore probe")

    def _throw_always(*_a, **_k):
        raise sentinel

    def _throw_on_capture(ns):
        if getattr(ns, "capture_baseline", False):
            raise SystemExit("capture probe")
        return real_cmd_smoke(ns)

    for label, stub, caught in (
        ("a throw before the capture phase", _throw_always, RuntimeError),
        ("a SystemExit inside the capture phase", _throw_on_capture,
         SystemExit),
    ):
        before = _bench_identity_snapshot()
        threw = False
        returned = None
        probe_out, probe_err = io.StringIO(), io.StringIO()
        try:
            bench.cmd_smoke = stub
            # The scenario prints its own progress; captured so a PASSING
            # arm does not bury the report, and printed below if it FAILS.
            with contextlib.redirect_stdout(probe_out), \
                    contextlib.redirect_stderr(probe_err):
                try:
                    returned = check_smoke_exit_contract()
                except caught as e:
                    if caught is RuntimeError and e is not sentinel:
                        raise
                    threw = True
        finally:
            bench.cmd_smoke = real_cmd_smoke

        if not threw:
            print(f"FAIL  [smoke contract restore: {label} — the probe never "
                  f"threw (the scenario returned {returned!r}), so this arm "
                  f"asserted nothing. Read its output:\n"
                  f"{probe_out.getvalue()[-600:]}")
            errors += 1
            continue
        drifted = _bench_identity_drift(before)
        if drifted:
            # Put it back before returning. The message is right about the
            # consequence, and leaving the consequence in place would make
            # the REST of this report the mis-attributed cascade it warns
            # about. `delattr` for names the scenario ADDED, and a MISSING
            # sentinel rather than `getattr(..., None)`: a global whose
            # legitimate value is None would otherwise look like a drift
            # that could not be repaired.
            missing = object()
            for name, value in before.items():
                if getattr(bench, name, missing) is not value:
                    setattr(bench, name, value)
            for name in list(vars(bench)):
                if name not in before and name not in _SMOKE_RESTORE_MUTABLE:
                    delattr(bench, name)
            print(f"FAIL  [smoke contract restore: {label} left "
                  f"{', '.join(drifted)} mutated. Every later check in this "
                  f"process would run against that state, and the FAIL it "
                  f"prints would name the wrong check. Restored here so the "
                  f"rest of this report is readable.]")
            errors += 1

    # ...and the HEALTHY run, which neither throw phase can see. Phase (a)
    # throws before `cmd_smoke` ever runs, so it never reaches
    # `set_dma_basis`; phase (b) snapshots after phase (a), by which point
    # the state is already whatever the previous phase left. A scenario
    # that returns 0 and quietly leaves `bench._DMA_BASIS` set is the
    # ordinary case, and it was invisible to both.
    before = _bench_identity_snapshot()
    probe_out = io.StringIO()
    with contextlib.redirect_stdout(probe_out), \
            contextlib.redirect_stderr(io.StringIO()):
        healthy = check_smoke_exit_contract()
    drifted = _bench_identity_drift(before)
    if drifted:
        missing = object()
        for name, value in before.items():
            if getattr(bench, name, missing) is not value:
                setattr(bench, name, value)
        for name in list(vars(bench)):
            if name not in before and name not in _SMOKE_RESTORE_MUTABLE:
                delattr(bench, name)
        print(f"FAIL  [smoke contract restore: a HEALTHY run (errors="
              f"{healthy}) left {', '.join(drifted)} mutated. `main()` "
              f"clears bench's per-invocation state between invocations and "
              f"nothing clears it for an in-process caller, so the next "
              f"check inherits a posture it never chose. Restored here so "
              f"the rest of this report is readable.]")
        errors += 1

    if errors == 0:
        print(f"ok    [smoke contract restore: a throw before the capture "
              f"phase, a SystemExit inside it, and a healthy run each hand "
              f"the whole `bench` namespace back unchanged]")
    return errors


def check_compile_csv_raw_dir_out_default() -> int:
    """`compile-csv --raw-dir X` writes to X's PARENT, not a run dir.

    compile_csv.py documents and implements the raw-dir output default as
    the raw dir's parent, and reads the manifest variant from that same
    parent — that directory IS the run dir for this legacy single-rep
    interface. bench.py used to resolve the default
    results/<machine-hash>-<date>-<variant>/ path and pass it as --out-dir
    in this mode too, so the CSVs landed somewhere unrelated to the raw dir
    the caller named, while the variant came from that raw dir's parent.
    Both halves of one invocation have to agree about where the run is.

    Drives the real bench.py entry point (which subprocesses the real
    compile_csv.py), so a regression in either half fails here."""
    errors = 0
    size = 64
    n = bench.samples_for("quiescent", size)[1]
    with tempfile.TemporaryDirectory(prefix="rawdir_") as td:
        # Point the module's results root INTO the temp tree for the
        # duration. The second reason is the one that matters: a regression
        # here sends the CSVs to the DEFAULT run dir, which lives under the
        # REPO — so without this the failing case would litter
        # benches/latency/results/ (measured: it does), and the stray sweep
        # below, which walks only the temp tree, could not see the very
        # files that prove the regression.
        saved_root = bench.RESULTS_ROOT
        bench.RESULTS_ROOT = Path(td) / "would-be-run-root"
        try:
            errors += _raw_dir_out_default_arms(td, size, n)
        finally:
            bench.RESULTS_ROOT = saved_root
    if not errors:
        print("ok    [compile-csv --raw-dir: CSVs land in the raw dir's "
              "parent and nowhere else under the run roots this check can "
              "reach; an explicit --out-dir still wins]")
    return errors


def _raw_dir_out_default_arms(td: str, size: int, n: int) -> int:
    """The arms of check_compile_csv_raw_dir_out_default, split out only so
    the caller can own the RESULTS_ROOT redirect in a short try/finally."""
    errors = 0
    legacy_run = Path(td) / "some-legacy-run"
    raw = legacy_run / "raw"
    raw.mkdir(parents=True)
    (legacy_run / "run.json").write_text('{"variant": "quiescent"}')
    (raw / f"probe_chrt0_{size}.bin").write_bytes(
        struct.pack(f"<{n}Q", *range(1000, 1000 + n)))

    rc = bench.cmd_compile_csv(argparse.Namespace(
        variant=None, run_dir=None, raw_dir=str(raw),
        out_dir=None, allow_partial=True))
    landed = sorted(q.name for q in legacy_run.glob("results_*.csv"))
    if rc != 0 or landed != ["results_probe_chrt0.csv"]:
        print(f"FAIL  [compile-csv --raw-dir: CSVs did not land in the "
              f"raw dir's parent]  rc={rc} found={landed}")
        errors += 1
    # ...and nothing was written anywhere else under the temp root.
    strays = sorted(str(q.relative_to(td))
                    for q in Path(td).rglob("results_*.csv")
                    if q.parent != legacy_run)
    if strays:
        print(f"FAIL  [compile-csv --raw-dir: CSVs also written outside "
              f"the raw dir's parent]  {strays}")
        errors += 1
    # An explicit --out-dir still wins: the default is a default.
    explicit = Path(td) / "elsewhere"
    rc2 = bench.cmd_compile_csv(argparse.Namespace(
        variant=None, run_dir=None, raw_dir=str(raw),
        out_dir=str(explicit), allow_partial=True))
    if rc2 != 0 or not (explicit / "results_probe_chrt0.csv").exists():
        print(f"FAIL  [compile-csv --raw-dir --out-dir: explicit out dir "
              f"not honoured, or the run failed]  rc={rc2}")
        errors += 1
    # An explicit --variant selects nothing in this mode, so it is
    # CHECKED instead: the variant decides the exact-sample-count gate,
    # and compiling a run against the wrong schedule is the mislabel
    # this seam exists to prevent. Four shapes, because the guard has to
    # agree with the reader it guards (compile_csv.read_manifest_variant)
    # rather than merely be stricter than it.
    variant_arms = [
        ("matching", "quiescent", 0, True),
        ("contradicting", "fixed100", 2, False),
        # A manifest the READER rejects cannot contradict anything: it
        # degrades that run to warn-if-matches-neither-schedule, so a
        # refusal here would block a run compile_csv would have taken.
        ("manifest variant unknown to the reader", "quiescent", 0, True),
    ]
    for label, variant, want_rc, want_csv in variant_arms:
        probe = Path(td) / f"variant-{variant}-{label.split()[0]}"
        praw = probe / "raw"
        praw.mkdir(parents=True)
        (probe / "run.json").write_text(
            '{"variant": "quiescant"}' if "unknown" in label
            else '{"variant": "quiescent"}')
        (praw / f"probe_chrt0_{size}.bin").write_bytes(
            struct.pack(f"<{n}Q", *range(1000, 1000 + n)))
        rcv = bench.cmd_compile_csv(argparse.Namespace(
            variant=variant, run_dir=None, raw_dir=str(praw),
            out_dir=None, allow_partial=True))
        got_csv = (probe / "results_probe_chrt0.csv").exists()
        if rcv != want_rc or got_csv != want_csv:
            print(f"FAIL  [compile-csv --raw-dir --variant {label}]  "
                  f"rc={rcv} (want {want_rc}), csv={got_csv} "
                  f"(want {want_csv})")
            errors += 1
    return errors


def _compile_dns_fixture(run_dir: Path, out_name: str, *extra) -> "tuple":
    """`(rc, stderr, body rows)` of one compile of a did-not-sustain fixture.

    ONE helper for BOTH scenarios below. The row filter is the thing the two
    arms compare against each other (`rows3 != rows`, and a `startswith`
    oracle on the exhausted row), so a second copy is a place for them to
    drift into disagreeing about what a "row" IS while both still print ok.

    `rows` is None when no CSV was written at all. compile_csv has several
    early returns BEFORE it writes anything -- one of them reachable straight
    from the operator's shell, since an ambient `CER_BENCH_SMOKE_N` makes it
    refuse at once -- and reading the CSV unconditionally turned that into a
    FileNotFoundError traceback that took every check after this one with it,
    while the one line explaining it died inside this StringIO. Captured
    stderr is returned on EVERY path for the same reason: a diagnosis
    surfaced only when nothing went wrong is no diagnosis.
    """
    err = io.StringIO()
    with contextlib.redirect_stderr(err), \
            contextlib.redirect_stdout(io.StringIO()):
        rc = compile_csv.main(["--run-dir", str(run_dir), "--out-dir",
                               str(run_dir / out_name), *extra])
    csv_path = run_dir / out_name / "results_probe_chrt0.csv"
    if not csv_path.exists():
        return rc, err.getvalue(), None
    return rc, err.getvalue(), [
        ln for ln in csv_path.read_text().splitlines()
        if ln and not ln.startswith("#")
        and not ln.startswith("payload_bytes")]


# The per-rep detection's own words, spelled ONCE. Both did-not-sustain
# scenarios key on it -- one requires it, the other requires its ABSENCE to
# prove isolation -- and a second copy silently stops matching when
# compile_csv rewords, which is a fail-open in the arm that needs absence.
_PER_REP_REASON = ("carries samples while its own rep declares "
                   "did_not_sustain")


def _cell_level_contradiction_arm(td: str, dns_size: int) -> int:
    """The CELL-level contradiction: every rate sidecar says the ladder was
    exhausted, and a rep with NO sidecar left samples anyway.

    A second SCENARIO rather than a stricter assertion on the first one: on
    a ONE-rep fixture the per-rep arm consumes the contradiction first (an
    aggregate can only read did_not_sustain when that single rep's own
    sidecar says so), so the cell-level arm is unreachable there and
    deleting it is green either way.

    Its blast radius is why that gap is closed here rather than noted.
    Measured on this isolating fixture: deleting the cell-level DETECTION
    in compile_csv (not this arm -- deleting a test cannot move a compile's
    rc) takes the STRICT compile from rc 1 to rc 0 and ships a row whose
    achieved_rate_hz column reads did_not_sustain -- fabricated latency at
    a payload the ladder declared exhausted, in the DEFAULT mode, at exit 0
    (Principle #13). Not the --allow-partial-only hazard the sibling arm's
    comments describe.

    rep1 is the real exhaustion (sidecar, no `.bin`); rep2 carries the
    samples with NO sidecar of its own, which is the only shape that
    reaches the aggregate check.
    """
    errors = 0
    run_dir = Path(td) / "cell_level"
    reps = {}
    for rep in ("rep1", "rep2"):
        raw = run_dir / rep / "raw"
        raw.mkdir(parents=True)
        reps[rep] = raw
    (run_dir / "run.json").write_text('{"variant": "fixed100"}')
    for size in bench.PAYLOAD_SIZES:
        n = bench.samples_for("fixed100", size)[1]
        blob = struct.pack(f"<{n}Q", *range(1000, 1000 + n))
        for rep, raw in reps.items():
            if size == dns_size:
                if rep == "rep1":
                    (raw / f"probe_chrt0_{size}.rate").write_text(
                        f"{compile_csv.DID_NOT_SUSTAIN}\n")
                else:
                    # Samples, and NO sidecar: the rep the cell-level arm
                    # names. A sidecar here would be caught per-rep instead.
                    (raw / f"probe_chrt0_{size}.bin").write_bytes(blob)
                continue
            (raw / f"probe_chrt0_{size}.rate").write_text("100\n")
            (raw / f"probe_chrt0_{size}.bin").write_bytes(blob)

    CELL_REASON = ("every rate sidecar says did_not_sustain but a rep with "
                   "no sidecar left samples")
    rc, err, _ = _compile_dns_fixture(run_dir, "csv_strict")
    if rc == 0:
        print("FAIL  [did_not_sustain (cell level): a contradictory cell "
              "passed the strict compile -- a full percentile row stamped "
              "did_not_sustain would ship at exit 0.]")
        errors += 1
    if CELL_REASON not in err:
        print(f"FAIL  [did_not_sustain (cell level): the strict compile "
              f"refused, but no line named {CELL_REASON!r}.]  "
              f"stderr={err!r}")
        errors += 1
    if _PER_REP_REASON in err or "missing .bin" in err:
        # The isolation IS the arm: with either of those present this
        # fixture is exercising the sibling detection or an ambient gap,
        # and its verdict would say nothing about the cell-level one.
        print(f"FAIL  [did_not_sustain (cell level): the fixture is not "
              f"isolating -- stderr also carries the per-rep reason or a "
              f"missing .bin, so a nonzero exit here proves nothing about "
              f"the cell-level arm.]  stderr={err!r}")
        errors += 1
    rc2, err2, rows2 = _compile_dns_fixture(
        run_dir, "csv_partial", "--allow-partial")
    dns_rows = [] if rows2 is None else [
        r for r in rows2 if r.startswith(f"{dns_size},")]
    if (rc2 != 0 or len(dns_rows) != 1
            or not dns_rows[0].startswith(f"{dns_size},0,,")
            or compile_csv.DID_NOT_SUSTAIN not in dns_rows[0]):
        print(f"FAIL  [did_not_sustain (cell level): --allow-partial did "
              f"not reduce the payload to the correct empty row]  rc={rc2} "
              f"{dns_rows}  stderr={err2!r}")
        errors += 1
    if CELL_REASON not in err2:
        # The same loudness contract the per-rep scenario asserts, which
        # this one was missing: suppressing the warning still exits 0 with
        # the identical empty row, so without this the arm passes while the
        # operator loses the only explanation of what was withheld.
        print(f"FAIL  [did_not_sustain (cell level): --allow-partial "
              f"dropped the contradictory rep silently -- no line named "
              f"{CELL_REASON!r}. A partial artifact that ships at exit 0 "
              f"with no record of what was withheld is the outcome this "
              f"whole check exists to prevent.]  stderr={err2!r}")
        errors += 1
    if not errors:
        print("ok    [did_not_sustain (cell level): a rep with no sidecar "
              "leaving samples beside an exhausted verdict is refused "
              "strictly BY NAME, in isolation from the per-rep arm, and "
              "mints no latency under --allow-partial]")
    return errors


def check_did_not_sustain_exclusion() -> int:
    """compile_csv refuses to build a row from contradictory artifacts.

    A fixed100 ladder exhaustion mints NO latency: the runner writes a
    did_not_sustain sidecar and removes the `.bin`. If a stale `.bin`
    survives anyway, the pair is contradictory — and aggregating it emits
    a full percentile row computed from a previous run's samples, which
    under --allow-partial ships at exit 0 while plot.py reads the same
    row's did_not_sustain and renders the payload as having produced no
    latency. So the row is flagged AND excluded, the same way the
    wrong-sample-count path excludes its file. Both arms are pinned: the
    correct empty row for a real exhaustion (sidecar, no `.bin`) must
    still be written.

    The fixture carries the COMPLETE pinned sweep — every pinned payload
    but the exhausted one, sustained — for a reason the earlier one-payload
    version made plain. With the rest absent, a strict compile exits nonzero
    on their `missing .bin` rows no matter what it thinks of the
    contradiction,
    so a bare `rc != 0` assertion was VACUOUS: deleting the per-rep
    detection outright left this arm printing `ok` (measured — the
    `--allow-partial` half is rescued by the cell-level arm, which fires on
    the same fixture, so nothing observed the per-rep half at all). A
    complete fixture makes the strict verdict flip 0 → nonzero on the
    contradiction ALONE, and the reason is asserted by name rather than
    inferred from the exit code.
    """
    errors = 0
    dns_size = bench.PAYLOAD_SIZES[0]
    # The per-rep detection's own words. Asserted verbatim because the
    # cell-level arm ("every rate sidecar says did_not_sustain but a rep
    # with no sidecar left samples") also refuses this fixture: an exit code
    # cannot tell the two apart, and only one of them is under test here.
    #
    # SCOPE: with the per-rep detection present, the CELL-level arm never
    # fires on a one-rep fixture, so deleting it alone leaves THIS check
    # green (measured). That arm needs a second rep carrying samples with
    # no sidecar of its own -- a different scenario, not a stricter
    # assertion about this one -- and it is covered by
    # `_cell_level_contradiction_arm`, driven from the end of this check.
    with tempfile.TemporaryDirectory(prefix="dns_") as td:
        run_dir = Path(td) / "run"
        raw = run_dir / "rep1" / "raw"
        raw.mkdir(parents=True)
        (run_dir / "run.json").write_text('{"variant": "fixed100"}')
        for size in bench.PAYLOAD_SIZES:
            rate = raw / f"probe_chrt0_{size}.rate"
            if size == dns_size:
                rate.write_text(f"{compile_csv.DID_NOT_SUSTAIN}\n")
                continue
            n = bench.samples_for("fixed100", size)[1]
            rate.write_text("100\n")
            (raw / f"probe_chrt0_{size}.bin").write_bytes(
                struct.pack(f"<{n}Q", *range(1000, 1000 + n)))

        # The consistent fixture, compiled STRICTLY: the exhausted payload gets
        # its empty row and the other nine are ordinary percentile rows, so
        # a clean strict compile is the baseline the contradiction below is
        # measured against. Without this arm, "strict refused" says nothing.
        rc, err, rows = _compile_dns_fixture(run_dir, "csv_honest")
        dns_rows = [] if rows is None else [
            r for r in rows if r.startswith(f"{dns_size},")
            and compile_csv.DID_NOT_SUSTAIN in r]
        honest_row = dns_rows[0] if len(dns_rows) == 1 else None
        if (rc != 0 or rows is None
                or len(rows) != len(bench.PAYLOAD_SIZES)
                or honest_row is None or not honest_row.startswith(
                    f"{dns_size},0,,")):
            print(f"FAIL  [did_not_sustain: the correct empty row]  rc={rc} "
                  f"rows="
                  f"{'NO CSV WRITTEN' if rows is None else len(rows)}"
                  f"/{len(bench.PAYLOAD_SIZES)} dns_rows={dns_rows} "
                  f"stderr={err!r}")
            errors += 1
        # The SUSTAINED rows are checked for CONTENT, not merely for
        # existence. `rows3 != rows` further down otherwise means "those
        # rows, whatever they are, are unchanged" -- measured: blanking
        # every percentile cell in compile_csv left this arm green, so the
        # baseline the contradiction is measured against would accept nine
        # hollow rows and "strict refused" would again say very little.
        want_iters = {str(s): str(bench.samples_for("fixed100", s)[1])
                      for s in bench.PAYLOAD_SIZES if s != dns_size}
        hollow = []
        judged = 0
        for row in (rows or []):
            cells = row.split(",")
            want = want_iters.get(cells[0])
            if want is None:                      # the dns row, checked above
                continue
            judged += 1
            if (cells[1] != want or not cells[2]
                    or compile_csv.DID_NOT_SUSTAIN in row):
                hollow.append(row)
        if hollow:
            print(f"FAIL  [did_not_sustain: a sustained payload compiled to "
                  f"a hollow row -- iterations must be the schedule's "
                  f"count, round_trip_p50_ns must be present, and no "
                  f"sustained row may carry the "
                  f"{compile_csv.DID_NOT_SUSTAIN} token.]  {hollow}")
            errors += 1
        if judged != len(want_iters):
            # `want is None` means "the exhausted row" AND "no expectation
            # for this row", and nothing distinguished them: measured, a
            # payload-column format drift made the loop skip every row while
            # reporting green, and it is the only cover for the hollow-row
            # defect it catches.
            print(f"FAIL  [did_not_sustain: the sustained-row content check "
                  f"judged {judged} of {len(want_iters)} rows -- the "
                  f"payload column no longer matches bench.PAYLOAD_SIZES, "
                  f"so the loop skipped rows it should have judged while "
                  f"reporting green.]")
            errors += 1
        # The "missing .bin" guard below matches compile_csv's wording, and
        # a wording guard fails OPEN: reword it there and the guard matches
        # nothing, silently restoring the ambient-nonzero vacuity this arm
        # was rewritten to close. So prove the literal can still fire, on
        # this same fixture, by removing one payload and putting it back.
        gap_size = bench.PAYLOAD_SIZES[1]
        gap = raw / f"probe_chrt0_{gap_size}.bin"
        if not gap.exists():
            # The fixture loop above writes a .bin for every size but the
            # exhausted one, so this cannot happen -- and saying so beats
            # the FileNotFoundError traceback it would otherwise be.
            print(f"FAIL  [did_not_sustain: the fixture never wrote "
                  f"{gap.name}, so the completeness control below cannot "
                  f"run and the fixture is not the complete sweep this arm "
                  f"depends on.]")
            errors += 1
            gap_bytes = None
        else:
            gap_bytes = gap.read_bytes()
            gap.unlink()
            _, err_gap, _ = _compile_dns_fixture(run_dir, "csv_gap")
            if "missing .bin" not in err_gap:
                print(f"FAIL  [did_not_sustain: the 'missing .bin' literal "
                      f"no longer matches compile_csv's wording, so the "
                      f"fixture-completeness guard below can never fire "
                      f"and the ambient-nonzero vacuity is back.]  stderr="
                      f"{err_gap!r}")
                errors += 1
            gap.write_bytes(gap_bytes)
        # Now the contradiction: the exhausted payload's sidecar with a
        # full-length .bin beside it. Strict must refuse, FOR THIS REASON;
        # --allow-partial must warn and write the empty row rather than
        # percentiles from those samples.
        n_dns = bench.samples_for("fixed100", dns_size)[1]
        (raw / f"probe_chrt0_{dns_size}.bin").write_bytes(
            struct.pack(f"<{n_dns}Q", *range(1000, 1000 + n_dns)))
        rc2, err2, _ = _compile_dns_fixture(run_dir, "csv_strict")
        if rc2 == 0:
            print("FAIL  [did_not_sustain: a contradictory cell passed "
                  "the strict compile]")
            errors += 1
        if _PER_REP_REASON not in err2:
            print(f"FAIL  [did_not_sustain: the strict compile refused, but "
                  f"not for the contradiction — no line named "
                  f"{_PER_REP_REASON!r}. An exit code alone cannot say WHY a "
                  f"strict compile refused, which is how this arm passed "
                  f"with the per-rep detection deleted.]  stderr={err2!r}")
            errors += 1
        if "missing .bin" in err2:
            print(f"FAIL  [did_not_sustain: the fixture is incomplete — "
                  f"strict is refusing missing payloads as well, so its "
                  f"nonzero exit is ambient and proves nothing about the "
                  f"contradiction.]  stderr={err2!r}")
            errors += 1
        rc3, err3, rows3 = _compile_dns_fixture(
            run_dir, "csv_partial", "--allow-partial")
        # The contradictory rep's samples are EXCLUDED, so what survives at
        # that payload is the correct empty row — iterations 0 and every stat
        # cell blank, byte-identical to a real exhaustion — and the nine
        # sustained rows are untouched. Never a percentile row there: that
        # is the outcome under test, because a percentile here would be
        # computed from a previous run's samples and would ship at exit 0.
        if rc3 != 0 or rows3 is None or rows3 != rows:
            print(f"FAIL  [did_not_sustain: --allow-partial did not "
                  f"reproduce the strict rows]  rc={rc3} "
                  f"{'NO CSV WRITTEN' if rows3 is None else rows3}, "
                  f"expected "
                  f"{'NO CSV WRITTEN' if rows is None else rows}  "
                  f"stderr={err3!r}")
            errors += 1
        if _PER_REP_REASON not in err3:
            print(f"FAIL  [did_not_sustain: --allow-partial dropped the "
                  f"contradictory rep silently — the exclusion must be "
                  f"LOUD, or a partial artifact ships at exit 0 with no "
                  f"record of what was withheld.]  stderr={err3!r}")
            errors += 1
        errors += _cell_level_contradiction_arm(td, dns_size)
    if not errors:
        print("ok    [did_not_sustain: correct empty row written beside a "
              "complete sweep; the stale-.bin contradiction refused "
              "strictly BY NAME and reduced to the same empty row, loudly, "
              "under --allow-partial]")
    return errors


def check_usage_wiring() -> int:
    """Runtime-exercise bench.py's CER_BENCH_USAGE native path — the
    per-size invocation split, the per-size CER_BENCH_PAYLOAD_SIZES env
    override, the sidecar naming + stale-sidecar guard, and the
    exact-set gate under per-size invocations — against a stub bench
    binary and a stub sampler (platform-neutral; the REAL sampler +
    binaries get their box exercise in the campaign, see README).

    cleanup_iceoryx / kill_stragglers are stubbed to no-ops: SHM hygiene
    is not under test, and this host may be running real iceoryx2 work
    that a /tmp/iceoryx2 rmtree would destroy.

    The arm owns a PRIVATE root it creates and removes ITSELF. The
    delta over the `tempfile.TemporaryDirectory` it replaces
    is narrow and worth stating plainly: both are private and both are
    removed, but a cleanup failure is now a `note` naming the leftover
    instead of an exception thrown out of the arm. What actually protects
    the oracles below -- every one of which names its paths by GLOB -- is
    the EXACT-SET assertion at the end of the usage-ON leg: the raw dir
    must hold exactly the files this invocation wrote, so a stray file
    from anywhere (a shared root, a stale-guard regression, a second
    invocation) is a FAIL rather than something a glob quietly absorbs."""
    errors = 0
    saved = {n: getattr(bench, n) for n in
             ("usage_enabled", "USAGE_SAMPLER", "NATIVE_BIN_DIR",
              "cleanup_iceoryx", "kill_stragglers")}
    saved_env = {k: os.environ.get(k)
                 for k in ("CER_BENCH_PAYLOAD_SIZES", "CER_STUB_TRACE",
                           "CER_STUB_SAMPLES", "CER_STUB_SIDECAR_WAIT_S",
                           "CER_STUB_SAMPLER_DELAY_S")}
    root = Path(tempfile.mkdtemp(prefix="usage_wire_"))
    try:
        try:
            d = root
            bin_dir = d / "bins"; bin_dir.mkdir()
            stub_bin = bin_dir / "stub_native_bench"
            stub_bin.write_text(_STUB_BENCH)
            stub_bin.chmod(0o755)
            stub_sampler = d / "stub_sampler.py"
            stub_sampler.write_text(_STUB_SAMPLER)
            trace = d / "invocations.txt"
            raw_dir = d / "raw"; raw_dir.mkdir()
            log_dir = d / "logs"
            os.environ["CER_BENCH_PAYLOAD_SIZES"] = "64 4096"
            os.environ["CER_STUB_TRACE"] = str(trace)
            # The stub must write the count run_native_bench's gate expects
            # for THIS variant, or the checker fails on the sample-count
            # gate rather than on the wiring under test. fixed100 is uniform
            # across sizes; assert that rather than assuming it.
            stub_counts = {bench.samples_for("fixed100", s)[1]
                           for s in (64, 4096)}
            if len(stub_counts) != 1:
                print(f"FAIL  [the fixed100 schedule is no longer uniform "
                      f"across the stub's sizes ({sorted(stub_counts)}) — "
                      f"the stub needs a per-size count]")
                errors += 1
            os.environ["CER_STUB_SAMPLES"] = str(max(stub_counts))
            bench.NATIVE_BIN_DIR = bin_dir
            bench.USAGE_SAMPLER = stub_sampler
            bench.cleanup_iceoryx = lambda: None
            bench.kill_stragglers = lambda: None
            nb = bench.NativeBench("stub_native_bench", "stubx",
                                   chrt_on=False)
            raw_name = "stubx_chrt0"

            # -- usage mode: per-size invocations + sidecars ------------
            # The sampler is spawned AFTER the stub and killed the moment
            # the stub exits, so the stub waits for the sidecar rather than
            # for a fixed 0.1s (see _STUB_BENCH). 10s is ~100x a cold
            # interpreter start; exceeding it means the sampler never wrote,
            # which is the failure the oracles below are supposed to report.
            os.environ["CER_STUB_SIDECAR_WAIT_S"] = "10"
            # The sampler is made DELIBERATELY slow to start, so this arm
            # asserts the ordering instead of racing it (see _STUB_SAMPLER).
            # Comfortably longer than the 0.1s the stub used to sleep, and
            # comfortably inside the 10s budget above.
            os.environ["CER_STUB_SAMPLER_DELAY_S"] = "0.5"
            bench.usage_enabled = lambda: True
            stale = raw_dir / f"{raw_name}_64.usage.csv"
            stale.write_text("STALE — a prior run's sidecar\n")
            ok = bench.run_native_bench(nb, 0, "fixed100", raw_dir, log_dir)
            if not ok:
                print("FAIL  [usage-mode run_native_bench returned False "
                      "(exact-set gate should pass on the stub's .bins)]")
                errors += 1
            invocations = (trace.read_text().splitlines()
                           if trace.exists() else [])
            if invocations != ["invocation: 64", "invocation: 4096"]:
                print(f"FAIL  [usage mode should invoke once per size with "
                      f"a single-size CER_BENCH_PAYLOAD_SIZES override: "
                      f"{invocations}]")
                errors += 1
            for size in (64, 4096):
                sc = raw_dir / f"{raw_name}_{size}.usage.csv"
                log = log_dir / f"{raw_name}_{size}.log"
                if not sc.exists():
                    # The stub records an expired sidecar wait on its own
                    # stderr, which lands in this per-cell log — and the arm
                    # deletes its root on the way out, so the line has to be
                    # carried here or it is gone before anyone reads it.
                    why = ""
                    if log.exists():
                        tail = [ln for ln in
                                log.read_text(errors="replace").splitlines()
                                if "stub:" in ln]
                        if tail:
                            why = f" — the stub said: {tail[-1].strip()}"
                    print(f"FAIL  [missing per-size sidecar {sc.name}"
                          f"{why}]")
                    errors += 1
                elif "STALE" in sc.read_text():
                    print(f"FAIL  [stale-sidecar guard did not clear "
                          f"{sc.name}]")
                    errors += 1
                if not log.exists():
                    print(f"FAIL  [missing per-size log {log.name}]")
                    errors += 1
            # EXACT SET, not "the files I asked about exist". Every
            # oracle above globs this directory, so an extra file is as
            # much a defect as a missing one -- it is how a shared root,
            # or a stale-guard regression, reads as success.
            want_raw = {f"{raw_name}_{s}{suffix}"
                        for s in (64, 4096)
                        for suffix in (".bin", ".usage.csv")}
            got_raw = {q.name for q in raw_dir.iterdir()}
            if got_raw != want_raw:
                print(f"FAIL  [usage wiring: after the usage-ON leg the raw "
                      f"dir holds {sorted(got_raw)}, want exactly "
                      f"{sorted(want_raw)} — every oracle above globs this "
                      f"directory, so anything else here was read as this "
                      f"run's output]")
                errors += 1
            if not errors:
                print("ok    [usage mode: one invocation + sidecar + log "
                      "per size; stale sidecar cleared; the raw dir holds "
                      "exactly this run's files; gate passed]")

            # -- usage off: single invocation, sidecars swept -----------
            # No sampler runs on this path, so the wait must be DISARMED or
            # the stub would burn its whole budget waiting for a file
            # nothing is going to write.
            trace.unlink(missing_ok=True)
            os.environ.pop("CER_STUB_SIDECAR_WAIT_S", None)
            os.environ.pop("CER_STUB_SAMPLER_DELAY_S", None)
            bench.usage_enabled = lambda: False
            before = errors
            ok = bench.run_native_bench(nb, 0, "fixed100", raw_dir, log_dir)
            if not ok:
                print("FAIL  [usage-off run_native_bench returned False]")
                errors += 1
            invocations = (trace.read_text().splitlines()
                           if trace.exists() else [])
            if invocations != ["invocation: 64 4096"]:
                print(f"FAIL  [usage off should be ONE invocation with the "
                      f"ambient restriction intact: {invocations}]")
                errors += 1
            leftover = sorted(p.name for p in raw_dir.glob("*.usage.csv"))
            if leftover:
                print(f"FAIL  [usage-off run left prior sidecars beside its "
                      f"fresh .bins: {leftover} — the stale guard must "
                      f"clear them]")
                errors += 1
            if not (log_dir / f"{raw_name}.log").exists():
                print("FAIL  [usage-off run missing the single-sweep log]")
                errors += 1
            if errors == before:
                print("ok    [usage off: one sweep invocation, prior "
                      "sidecars swept, gate passed]")
        finally:
            # The arm removes its OWN root, on every exit path. A note, not
            # a FAIL: the next run mkdtemps a fresh one either way, so a
            # leftover is litter rather than a wrong verdict — but a silent
            # `ignore_errors` would hide a stub process still holding it.
            try:
                shutil.rmtree(root)
            except OSError as e:
                print(f"note  [usage wiring: could not remove its private "
                      f"root {root}: {e}]", file=sys.stderr)
    finally:
        for n, v in saved.items():
            setattr(bench, n, v)
        for k, v in saved_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    return errors


# ---------------------------------------------------------------------------
# Type-class axis (METHODOLOGY §18). Each arm below is tied to the fix
# it names: reverting that fix makes exactly that arm fail. The
# arms that drive a real runner or a real render say so in their own
# docstrings; the rest are pure.
# ---------------------------------------------------------------------------

_RUN_BENCH = Path(__file__).resolve().parent / "ros2" / "run_bench.sh"

# The type-class gate markers run_bench.sh prints (substrings of its
# stderr), and the FIRST post-gate check every arm below stops at.
_GATE_MARKERS = {
    "shm": "image is enumerated for SHM_MODE=shm only",
    "qos": "image is enumerated for CER_BENCH_QOS=be1 only",
    "loan": "image × loan is structurally unmeasurable",
    "zc": "image × zc is structurally unmeasurable",
    "lanes": "cannot ride the usage lanes",
}
_ROS_MARKER = "ROS setup not found"


def _drive_run_bench(raw_dir: Path, axes: dict) -> tuple:
    """Run the REAL ros2/run_bench.sh under a minimal env with the given
    axes and return (rc, stderr). ROS_SETUP_BASH points at a file that
    does not exist, so the first check AFTER the type-class gates
    refuses deterministically on every host — a ROS box would otherwise
    start a real cell — and reaching that refusal is the proof an arm
    PASSED the gates."""
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": os.environ.get("HOME", "/"),
        "CER_BENCH_RAW_DUMP_DIR": str(raw_dir),
        "CER_BENCH_RAW_NAME": "parity_probe",
        "ROS_SETUP_BASH": str(raw_dir / "no-such-ros-setup.bash"),
    }
    env.update(axes)
    r = subprocess.run(["bash", str(_RUN_BENCH)], env=env, cwd=str(raw_dir),
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=60)
    return r.returncode, r.stderr


_RUN_WORKSPACE = Path(__file__).resolve().parent / "workspace" / "run_workspace.sh"
_POD_ALIGN_MARKER = "pod-class payload size"
_NOT_DECIMAL_MARKER = "is not a decimal integer"
_TOO_MANY_DIGITS_MARKER = "has too many digits"
# DERIVED, not typed: the refusal interpolates the runner's ceiling, so a
# hard-typed number is a marker that breaks the day the pinned sweep grows
# — the same fragility that made the two stale markers this round. The
# ceiling IS the largest pinned size, which is where both runners get it.
_OUT_OF_RANGE_MARKER = f"is outside 1..{bench.PAYLOAD_SIZES[-1]}"
# The CONDITION's noun, not the sentence: the base rewrote this refusal's
# wording ("unsupported payload size 'N' in …" -> "payload size 'N' is not
# in the suite's TEN PINNED SIZES …") and three arms failed on the change,
# exactly as the SMOKE_N marker did a round earlier. Paired with the quoted
# VALUE each arm plants, which is what actually identifies the refusal —
# the alignment refusal names its size UNquoted, so the two cannot collide.
_NOT_PINNED_MARKER = "payload size"
_EMPTY_SIZES_MARKER = "splits to no tokens"
# Both runners enumerate the pinned set in their refusals; anchor on its
# first three, derived from the same list they print.
_VALID_SET_PREFIX = "valid: " + " ".join(str(n) for n in bench.PAYLOAD_SIZES[:3])
# The post-gate tripwire every workspace-runner arm carries: a bogus
# CER_BENCH_SMOKE_N is refused by the runner AFTER the payload-size parse
# and BEFORE its release rebuilds, so an arm that passes (or, under a
# broken variant, fails) the size gate stops here — never in a cargo build.
# Matched on the `abc` this harness plants (the runner's own message
# quotes the offending value), not on the refusal's wording: the wording
# is the runner's to change, and it has, once.
_SMOKE_N_MARKER = "CER_BENCH_SMOKE_N='abc'"


def _drive_run_workspace(raw_dir: Path, leg: str, axes: dict) -> tuple:
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": os.environ.get("HOME", "/"),
        "CER_BENCH_RAW_DUMP_DIR": str(raw_dir),
        "CER_BENCH_RAW_NAME": "parity_probe",
        "CER_BENCH_SMOKE_N": "abc",
    }
    env.update(axes)
    r = subprocess.run(["bash", str(_RUN_WORKSPACE), leg], env=env,
                       cwd=str(raw_dir), capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=60)
    return r.returncode, r.stderr


def check_workspace_size_contract() -> int:
    """workspace/run_workspace.sh's payload-size contract, driven through
    the REAL runner: (1) every token is a decimal integer of at most 10
    digits — bash's 64-bit `10#` WRAPS on overflow (18446744073709551680
    read as 64 pre-fix and would have minted a 64-byte row under that
    label), so the length is bounded BEFORE any arithmetic; (2) the
    surviving value lies in 1..16777216, the pinned sweep's ceiling
    (schedule + SHM provisioning end there; zero bytes is no sweep
    point); (3) an override that word-splits to NOTHING is refused rather
    than run as a zero-size sweep that would exit 0 having measured
    nothing; (4) the pod class refuses a size it cannot realize EXACTLY:
    PodPayloadShm is
    #[repr(C)] with three u32 fields, so its wire size is a multiple of
    4 (measured: N=65 bakes size_of 68) and, below the 13-byte floor
    (12 fixed-field bytes + a non-empty data array), no size is
    realizable at all; the matched-quantity rule (exactly N bytes)
    cannot hold for anything else. Pre-fix the runner
    accepted 65, rebuilt three crates in release, and the nodes' init
    guard failed the size at RUN time; a rounded size would be a
    mislabeled row, so the contract is REFUSE (exit 2 with the reason),
    never round; and (5) whatever survives all of that must be one of
    the TEN PINNED sweep sizes, for BOTH classes — an off-sweep size is
    realizable on this stack (PodPayload is generated per size, and the
    variable class takes any length) but would run under schedule_for's
    `*)` fallback, i.e. a schedule no other stack shares. The
    pod-alignment gate runs BEFORE the membership gate and its refusal
    names the pinned set as well, so a misaligned size gets one complete
    answer rather than two. Driven through the REAL runner on any host:
    refusals stop at the size gate; the controls (pinned sizes, both
    classes) pass it and stop at the SMOKE_N tripwire instead. The
    build-time twin (pod_codegen.rs's `is_multiple_of(4)` assert) is exercised by the
    round commit's `CER_BENCH_POD_BYTES=65 cargo build` evidence, not
    here — a cargo build is too heavy for this gate."""
    errors = 0
    # (marker, must-contain, must-NOT-contain) per refusal: the alignment
    # refusals must name the DECIMAL size (bash's $(( )) reads a padded
    # token as octal — '0000030' as 24, which is divisible by 4 and slipped
    # the gate; '08' made bash abort with "value too great for base"), and
    # non-digit tokens are refused as non-integers under BOTH classes.
    refusals = [
        ("pod x 65 beside a pinned size", {"CER_BENCH_MSG": "pod",
                                           "CER_BENCH_PAYLOAD_SIZES": "64 65"},
         _POD_ALIGN_MARKER, "size 65 ", ()),
        ("pod x 66 (even but not a multiple of 4)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "66"},
         _POD_ALIGN_MARKER, "size 66 ", ()),
        ("pod x 12 (aligned but below the 13-byte floor)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "12"},
         _POD_ALIGN_MARKER, "size 12 ", ()),
        ("pod x 0000030 (zero-padded: octal 24 slipped the gate pre-fix)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "0000030"},
         _POD_ALIGN_MARKER, "size 30 ", ("size 24", "size 0000030")),
        ("pod x 08 (bash arithmetic refuses 08 as octal)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "08"},
         _POD_ALIGN_MARKER, "size 8 ", ("value too great for base",)),
        ("pod x 1e3 (exponent is not a byte count)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "1e3"},
         _NOT_DECIMAL_MARKER, "'1e3'", ()),
        ("pod x 0x40 (hex literal refused)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "0x40"},
         _NOT_DECIMAL_MARKER, "'0x40'", ()),
        ("variable x 0x40 (the decimal contract is class-independent)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "0x40"},
         _NOT_DECIMAL_MARKER, "'0x40'", ()),
        ("variable x -64 (a sign is not a byte count)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "-64"},
         _NOT_DECIMAL_MARKER, "'-64'", ()),
        # Overflow class: bash's 64-bit `10#` WRAPS — 18446744073709551680
        # read as 64 pre-fix and would have minted a 64-byte row under
        # that label. The LENGTH bound fires before any arithmetic, so
        # the refusal must never mention the wrapped value.
        ("variable x 18446744073709551680 (20 digits: wrapped to 64 pre-fix)",
         {"CER_BENCH_MSG": "variable",
          "CER_BENCH_PAYLOAD_SIZES": "18446744073709551680"},
         _TOO_MANY_DIGITS_MARKER, "(20 > 10)", ("= 64", "size 64 ")),
        ("pod x 99999999999 (11 digits: fits 64-bit, still over the length bound)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "99999999999"},
         _TOO_MANY_DIGITS_MARKER, "(11 > 10)", ()),
        # Range class: the value that survives the length bound is compared
        # numerically against the pinned ceiling (and a zero-byte floor).
        ("variable x 9999999999 (10 digits: inside the length bound, over the ceiling)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "9999999999"},
         _OUT_OF_RANGE_MARKER, "(= 9999999999)", ()),
        ("pod x 16777217 (ceiling + 1)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "16777217"},
         _OUT_OF_RANGE_MARKER, "(= 16777217)", ()),
        ("variable x 0 (a zero-byte payload is not a sweep point)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "0"},
         _OUT_OF_RANGE_MARKER, "(= 0)", ()),
        # Sweep-point membership, class-INDEPENDENT: an off-sweep size is
        # realizable here (PodPayload is generated per size, and the
        # variable class takes any length) but would run under
        # schedule_for's `*)` fallback — a schedule no other stack shares.
        # 100 is a multiple of 4 and above the pod floor, so it kills an
        # "alignment instead of the exact set" implementation.
        ("pod x 100 (a multiple of 4, still not a pinned sweep size)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "64 100"},
         _NOT_PINNED_MARKER, "'100'", ()),
        ("variable x 65 (the pinned set is class-independent)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "64 65"},
         _NOT_PINNED_MARKER, "'65'", ()),
        # A whitespace-only override splits to ZERO tokens: every
        # per-token loop is skipped, so nothing is validated and the sweep
        # would measure nothing and still exit 0.
        ("variable x '   ' (whitespace only: a zero-size sweep)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "   "},
         _EMPTY_SIZES_MARKER, _VALID_SET_PREFIX, ()),
        ("variable x tabs and a newline (whitespace too)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "\t\n "},
         _EMPTY_SIZES_MARKER, _VALID_SET_PREFIX, ()),
        # NEWLINE-separated list: `read -r -a <<<` stops at the first
        # newline, so `CER_BENCH_PAYLOAD_SIZES=$(cat sizes.txt)` used to
        # validate line 1 and silently DROP the rest — a sweep that
        # measured one size while its caller asked for three, at exit 0.
        # The off-sweep token on line 3 is what proves the later lines are
        # reaching the validator at all.
        ("variable x a newline-separated list (later lines must be read)",
         {"CER_BENCH_MSG": "variable",
          "CER_BENCH_PAYLOAD_SIZES": "64\n1024\n65"},
         _NOT_PINNED_MARKER, "'65'", ()),
        # `run_*` and not `6*`: run_workspace.sh `cd`s to its OWN directory
        # before parsing, so the expansion happens there — where no
        # digit-named file exists, but `run_workspace.sh` itself does.
        # Without `set -f` the token becomes that filename and the refusal
        # names it; with the guard the token stays literal. The must_not
        # is what makes this a `set -f` pin rather than a digit-gate arm.
        ("variable x a glob matching a real file (pathname expansion off)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "run_*"},
         _NOT_DECIMAL_MARKER, "'run_*'", ("'run_workspace.sh'",)),
    ]
    # CER_BENCH_FORCE_RATE_HZ — the shell twin of the pod ping node's rate
    # guard this round added (strict parse + a 1 GHz ceiling), and until
    # now driven by nothing. Its own comment calls the spelling
    # "load-bearing on both sides": a LEADING ZERO is octal to the
    # runner's arithmetic and decimal to the node's parse (one label, two
    # rates), and an over-long value makes `test` return 2, which `if`
    # reads as FALSE — so a numeric range check alone is bypassed by
    # exactly the values it exists to stop.
    rate_refusals = [
        ("010 (octal to the runner, decimal 10 to the node)", "010",
         "is not a"),
        ("0 (a zero rate divides)", "0", "is not a"),
        ("abc", "abc", "is not a"),
        ("1.5", "1.5", "is not a"),
        ("-5", "-5", "is not a"),
        ("1e9 (an exponent is not a rate)", "1e9", "is not a"),
        ("99999999999 (11 digits: over the length bound)", "99999999999",
         "is not a"),
        ("1000000001 (ceiling + 1)", "1000000001", "out of range"),
    ]
    rate_controls = [("1", "1"), ("100", "100"),
                     ("1000000000 (exactly the ceiling)", "1000000000")]
    # (must-contain) per control: a zero-padded ALIGNED size is accepted
    # as its decimal value with a loud note, then stops at the tripwire.
    controls = [
        ("pod x a pinned subset 64 1024",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "64 1024"}, ()),
        ("pod x the pinned default sweep", {"CER_BENCH_MSG": "pod"}, ()),
        ("variable x the pinned default sweep",
         {"CER_BENCH_MSG": "variable"}, ()),
        # An EMPTY value is not the whitespace case: `${VAR:-default}`
        # treats empty as unset, so it means "no override" and runs the
        # full pinned sweep (the ROS 2 runner's `[ -z ]` reads it the same
        # way). Pinned as a control so a later `:-` -> `-` edit, which
        # would turn "unset me" into a zero-size sweep, fails here.
        ("variable x '' (empty means unset: the default sweep runs)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": ""}, ()),
        ("pod x 0000064 (zero-padded, aligned: read as decimal 64, noted)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "0000064"},
         ("'0000064' read as decimal 64",)),
        ("variable x 0000064 (same canonicalization on the variable class)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "0000064"},
         ("'0000064' read as decimal 64",)),
        ("variable x 16777216 (exactly the ceiling is admissible)",
         {"CER_BENCH_MSG": "variable", "CER_BENCH_PAYLOAD_SIZES": "64 16777216"},
         ()),
        ("pod x 16777216 (exactly the ceiling, aligned)",
         {"CER_BENCH_MSG": "pod", "CER_BENCH_PAYLOAD_SIZES": "16777216"}, ()),
    ]
    with tempfile.TemporaryDirectory(prefix="wssize_") as td:
        raw = Path(td)
        for desc, axes, marker, must, must_not in refusals:
            rc, err = _drive_run_workspace(raw, "split", axes)
            bad = [s for s in must_not if s in err]
            # The pod-alignment refusal claims (in its own comment) to name
            # the pinned set as well, so a misaligned size gets ONE complete
            # answer instead of being sent to a second refusal. Asserted,
            # not assumed.
            if marker == _POD_ALIGN_MARKER and "the pinned sweep sizes: " not in err:
                bad = bad + ["<missing: the pinned sweep sizes>"]
            if (rc != 2 or marker not in err or must not in err or bad
                    or _SMOKE_N_MARKER in err):
                print(f"FAIL  [size gate: {desc}] want rc=2 + {marker!r} "
                      f"naming {must!r} (never {list(must_not)}) before the "
                      f"SMOKE_N tripwire; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [size gate refuses: {desc}] rc={rc}")
        for desc, value, marker in rate_refusals:
            rc, err = _drive_run_workspace(
                raw, "split", {"CER_BENCH_FORCE_RATE_HZ": value})
            if rc != 2 or marker not in err or _SMOKE_N_MARKER in err:
                print(f"FAIL  [force-rate gate: {desc}] want rc=2 + "
                      f"{marker!r} before the SMOKE_N tripwire; got "
                      f"rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [force-rate gate refuses: {desc}] rc={rc}")
        for desc, value in rate_controls:
            rc, err = _drive_run_workspace(
                raw, "split", {"CER_BENCH_FORCE_RATE_HZ": value})
            if (rc != 2 or _SMOKE_N_MARKER not in err
                    or "CER_BENCH_FORCE_RATE_HZ" in err):
                print(f"FAIL  [force-rate gate control: {desc}] expected to "
                      f"pass the rate gate silently and stop at the SMOKE_N "
                      f"tripwire; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [force-rate gate control passes: {desc}]")
        for desc, axes, must in controls:
            rc, err = _drive_run_workspace(raw, "split", axes)
            missing = [s for s in must if s not in err]
            if (rc != 2 or _SMOKE_N_MARKER not in err or _POD_ALIGN_MARKER in err
                    or _NOT_DECIMAL_MARKER in err or _TOO_MANY_DIGITS_MARKER in err
                    or _OUT_OF_RANGE_MARKER in err or _NOT_PINNED_MARKER in err
                    or _EMPTY_SIZES_MARKER in err or missing):
                print(f"FAIL  [size gate control: {desc}] expected to pass "
                      f"the size gate (with {list(must)}) and stop at the "
                      f"SMOKE_N tripwire; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [size gate control passes: {desc}]")
    return errors


def check_ros2_sizes_decimal() -> int:
    """ros2/run_bench.sh applies the same size contract to its SIZES env
    (its per-size schedule is a string match that would route a
    zero-padded token to the fallback tuple, and the .bin names must be
    what bench.py expects): a non-digit, over-length (> 10 digits — the
    64-bit wrap class) or out-of-range (outside 1..16777216) token is
    refused before ROS is sourced; a zero-padded token is read as decimal
    with a loud note and the run proceeds to the ROS-setup check; exactly
    the ceiling is admissible. Any size outside the ten pinned sweep
    sizes is refused for BOTH classes (pod: Pod<N> is a fixed generated
    set; image: the in-container dispatch validates the sweep point)."""
    errors = 0
    with tempfile.TemporaryDirectory(prefix="ros2sizes_") as td:
        raw = Path(td)
        for desc, sizes, want_marker in (
                ("0x40", "0x40", "SIZES entry '0x40' is not a decimal integer"),
                ("1e3", "1e3", "SIZES entry '1e3' is not a decimal integer"),
                ("18446744073709551680 (20 digits: wrapped to 64 pre-fix)",
                 "18446744073709551680",
                 "SIZES entry '18446744073709551680' has too many digits (20 > 10)"),
                ("16777217 (ceiling + 1)", "16777217",
                 "SIZES entry '16777217' (= 16777217) is outside 1..16777216"),
                ("0 (zero-byte payload)", "64 0",
                 "SIZES entry '0' (= 0) is outside 1..16777216")):
            rc, err = _drive_run_bench(raw, {"SIZES": sizes})
            if rc != 2 or want_marker not in err or _ROS_MARKER in err:
                print(f"FAIL  [ros2 SIZES decimal: {desc}] want rc=2 + "
                      f"{want_marker!r} before any ROS check; got rc={rc}:\n"
                      f"{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [ros2 SIZES decimal refuses: {desc}] rc={rc}")
        rc, err = _drive_run_bench(raw, {"SIZES": "0000064 256"})
        note = "SIZES entry '0000064' read as decimal 64"
        if rc != 2 or note not in err or _ROS_MARKER not in err:
            print(f"FAIL  [ros2 SIZES decimal control: 0000064 256] want the "
                  f"note {note!r} then the ROS-setup stop; got rc={rc}:\n"
                  f"{err.rstrip()}")
            errors += 1
        else:
            print("ok    [ros2 SIZES decimal control: '0000064 256' read as "
                  "decimal 64 (noted), run proceeds to the ROS-setup check]")
        rc, err = _drive_run_bench(raw, {"SIZES": "64 16777216"})
        if (rc != 2 or _ROS_MARKER not in err or "too many digits" in err
                or "is outside 1.." in err):
            print(f"FAIL  [ros2 SIZES range control: 64 16777216] exactly the "
                  f"ceiling must be admissible; got rc={rc}:\n{err.rstrip()}")
            errors += 1
        else:
            print("ok    [ros2 SIZES range control: '64 16777216' (exactly the "
                  "ceiling) admitted, run proceeds to the ROS-setup check]")
        # Sweep-point realizability, class-INDEPENDENT: every ROS 2 cell
        # takes exactly the ten pinned sizes (pod: Pod<N> is a fixed set
        # of generated types; image: msg_class_dispatch.hpp's
        # valid_sweep_size rejects an off-sweep N in the container), so
        # any other size is refused before ROS is sourced — even 100, a
        # multiple of 4 (kills an "alignment instead of the exact set"
        # implementation), and under the image class too. SHM_MODE=shm on
        # the image arms: the existing image gates refuse the default
        # WITH_SHM sweep (it carries no_shm), so the enumerated shape is
        # pinned to isolate THIS gate from the image axis gates.
        set_marker = "is not a pinned sweep size"
        for desc, axes in (
                ("pod x 65", {"SIZES": "65"}),
                ("pod x 100 (a multiple of 4, still not a Pod<N> type)",
                 {"CER_BENCH_MSG": "pod", "SIZES": "64 100"}),
                ("image x 65 (the image arm validates the sweep point too)",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm", "SIZES": "65"})):
            rc, err = _drive_run_bench(raw, axes)
            if (rc != 2 or set_marker not in err or _ROS_MARKER in err
                    or _VALID_SET_PREFIX not in err):
                print(f"FAIL  [ros2 sweep-point set: {desc}] want rc=2 + "
                      f"{set_marker!r} naming the set, before any ROS check; "
                      f"got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [ros2 sweep-point set refuses: {desc}] rc={rc}")
        # A whitespace-only SIZES splits to ZERO tokens: the per-token
        # validation loop never runs, so nothing is checked and the sweep
        # would measure nothing and still exit 0. An EMPTY value is a
        # different thing — `[ -z ]` reads it as "no override" and the
        # default sweep runs — so both are pinned, one as a refusal and
        # one as a control.
        for desc, sizes in (("'   ' (spaces)", "   "),
                            ("tabs and a newline", "\t\n ")):
            rc, err = _drive_run_bench(raw, {"SIZES": sizes})
            if (rc != 2 or "splits to no tokens" not in err
                    or _ROS_MARKER in err or _VALID_SET_PREFIX not in err):
                print(f"FAIL  [ros2 SIZES empty split: {desc}] want rc=2 + "
                      f"'splits to no tokens' naming the set, before any ROS "
                      f"check; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [ros2 SIZES refuses a zero-size sweep: {desc}] "
                      f"rc={rc}")
        # run_bench.sh does not `cd`, so it parses SIZES with the caller's
        # cwd live — which is what makes the `set -f` guard observable
        # here: plant a DIGIT-named file the glob would expand to. Without
        # the guard, `6*` becomes `64`, a perfectly valid pinned size the
        # caller never typed, and the sweep runs one cell under it.
        (raw / "64").write_text("", encoding="utf-8")
        for desc, sizes, marker, must in (
                ("a newline-separated list (later lines must be read)",
                 "64\n1024\n65", "is not a pinned sweep size", "'65'"),
                ("a glob beside a matching file (pathname expansion off)",
                 "6*", "is not a decimal integer", "'6*'")):
            rc, err = _drive_run_bench(raw, {"SIZES": sizes})
            if rc != 2 or marker not in err or must not in err or _ROS_MARKER in err:
                print(f"FAIL  [ros2 SIZES {desc}] want rc=2 + {marker!r} "
                      f"naming {must!r} before any ROS check; got rc={rc}:\n"
                      f"{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [ros2 SIZES refuses: {desc}] rc={rc}")
        (raw / "64").unlink()
        rc, err = _drive_run_bench(raw, {"SIZES": ""})
        if rc != 2 or _ROS_MARKER not in err or "splits to no tokens" in err:
            print(f"FAIL  [ros2 SIZES empty control: ''] an empty value means "
                  f"'no override' and must run the default sweep; got "
                  f"rc={rc}:\n{err.rstrip()}")
            errors += 1
        else:
            print("ok    [ros2 SIZES empty control: '' means unset, the "
                  "default sweep runs]")
        rc, err = _drive_run_bench(raw, {"CER_BENCH_MSG": "image",
                                         "SHM_MODE": "shm", "SIZES": "64 1024"})
        if rc != 2 or _ROS_MARKER not in err or set_marker in err:
            print(f"FAIL  [ros2 sweep-point set control: image x 64 1024] pinned "
                  f"sizes must be admitted under the image class; got "
                  f"rc={rc}:\n{err.rstrip()}")
            errors += 1
        else:
            print("ok    [ros2 sweep-point set control: image x '64 1024' "
                  "admitted, run proceeds to the ROS-setup check]")
    return errors


def check_ros2_image_axis_gate() -> int:
    """ros2/run_bench.sh refuses every CER_BENCH_MSG=image axis bench.py
    does not enumerate (image = matrix rmws × shm × rclcpp × be1 ONLY):
    no_shm and rel10 are exit 2 (unenumerated — not structural, so not a
    77 skip), loan and zc stay the exit-77 structural skips, the usage
    lanes stay exit 2. Driven through the REAL script on a ROS-less host
    (the gates are pure env parsing hoisted above ROS sourcing), with
    two anti-tautology controls: the enumerated image shape and a pod
    cell on the very axes image refuses both PASS the gates and stop at
    the ROS-setup check instead."""
    errors = 0
    refusals = [
        ("image x no_shm (pinned)",
         {"CER_BENCH_MSG": "image", "SHM_MODE": "no_shm"}, 2, "shm"),
        ("image, SHM_MODE unset (the manual WITH_SHM sweep carries no_shm)",
         {"CER_BENCH_MSG": "image"}, 2, "shm"),
        ("image x shm x rel10",
         {"CER_BENCH_MSG": "image", "SHM_MODE": "shm",
          "CER_BENCH_QOS": "rel10"}, 2, "qos"),
        ("image x zc (structural skip)",
         {"CER_BENCH_MSG": "image", "SHM_MODE": "zc"}, 77, "zc"),
        ("image x loan (structural skip)",
         {"CER_BENCH_MSG": "image", "SHM_MODE": "shm",
          "RECV_PATH": "loan"}, 77, "loan"),
        ("image x stock lane",
         {"CER_BENCH_MSG": "image", "RMWS": "stock", "SHM_MODE": "stock",
          "CER_BENCH_QOS": "stock"}, 2, "lanes"),
    ]
    controls = [
        ("image x shm x be1 x rclcpp — the enumerated shape passes",
         {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"}),
        ("pod x no_shm x rel10 — the gates are image-scoped",
         {"CER_BENCH_MSG": "pod", "SHM_MODE": "no_shm",
          "CER_BENCH_QOS": "rel10"}),
    ]
    with tempfile.TemporaryDirectory(prefix="imggate_") as td:
        raw = Path(td)
        for desc, axes, want_rc, key in refusals:
            rc, err = _drive_run_bench(raw, axes)
            marker = _GATE_MARKERS[key]
            if rc != want_rc or marker not in err or _ROS_MARKER in err:
                print(f"FAIL  [image gate: {desc}] want rc={want_rc} + "
                      f"{marker!r} before any ROS check; got rc={rc}:\n"
                      f"{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [image gate refuses: {desc}] rc={rc}")
        for desc, axes in controls:
            rc, err = _drive_run_bench(raw, axes)
            fired = [k for k, m in _GATE_MARKERS.items() if m in err]
            if rc != 2 or _ROS_MARKER not in err or fired:
                print(f"FAIL  [image gate control: {desc}] expected to pass "
                      f"the type-class gates and stop at the ROS-setup "
                      f"check (rc=2); got rc={rc}, gates fired={fired}:\n"
                      f"{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [image gate control passes: {desc}]")
    return errors


def check_type_class_visuals() -> int:
    """plot.py gives a ROS 2 image twin its OWN visual assignment: the
    image cell's color_key carries the class token, so _assign_visuals
    (keyed by color_key, first-seen wins) no longer hands it the pod
    cell's color. Pins (a) the incumbent pod keys byte-identical, (b)
    pod vs image colors DIFFER on every matrix rmw, (c) the image color
    IS the documented same-hue tint of its pod twin's color (the
    1-hue-2-shades pairing, _role_for's _image_ branch), same tier, and
    (d) the workspace pod/variable pair differ too."""
    try:
        import plot
    except ImportError as e:   # matplotlib absent on this host
        print(f"skip  [type-class visuals: plot.py unimportable: {e}]")
        return 0
    errors = 0
    series = plot.ros2_series("jazzy", "0")
    vis = plot._assign_visuals(series)
    by_label = {s.label: s for s in series}
    for rmw in bench.STOCK_RMWS:
        pod = bench.Ros2Cell("jazzy", rmw, "shm", "rclcpp", "be1", 0)
        img = bench.Ros2Cell("jazzy", rmw, "shm", "rclcpp", "be1", 0,
                             msg="image")
        s_pod, s_img = by_label.get(pod.name), by_label.get(img.name)
        if s_pod is None or s_img is None:
            print(f"FAIL  [type-class visuals: {rmw} pod/image twins not "
                  f"both enumerated: {pod.name} / {img.name}]")
            errors += 1
            continue
        if s_pod.color_key != f"jazzy_{rmw}_shm_rclcpp":
            print(f"FAIL  [type-class visuals: incumbent pod color_key "
                  f"moved: {s_pod.color_key!r}]")
            errors += 1
        (c_pod, t_pod), (c_img, t_img) = (vis[s_pod.color_key],
                                          vis[s_img.color_key])
        want_img = plot._tint(c_pod, 0.28)
        if c_pod == c_img or c_img != want_img or t_pod != t_img:
            print(f"FAIL  [type-class visuals: {rmw} pod={c_pod}/{t_pod} "
                  f"image={c_img}/{t_img}; want image == tint(pod, 0.28) "
                  f"= {want_img}, same tier]")
            errors += 1
        else:
            print(f"ok    [type-class visuals: {rmw} pod {c_pod} vs image "
                  f"{c_img} (same-hue tint, tier {t_pod})]")
    ws = plot.workspace_series("0", "quiescent")
    wvis = plot._assign_visuals(ws)
    for leg in bench.WORKSPACE_LEGS:
        colors = {msg: wvis[next(s for s in ws if s.label ==
                                 bench.workspace_raw_name(leg, 0, msg))
                            .color_key][0]
                  for msg in bench.WORKSPACE_MSG_CLASSES}
        if len(set(colors.values())) != len(colors):
            print(f"FAIL  [type-class visuals: workspace {leg} classes "
                  f"share a color: {colors}]")
            errors += 1
        else:
            print(f"ok    [type-class visuals: workspace {leg} {colors}]")
    # Anti-vacuity, the discipline the footnote and loan-lane arms already
    # carry: both loops above iterate a bench.py tuple, so a shrunk
    # STOCK_RMWS / WORKSPACE_LEGS / WORKSPACE_MSG_CLASSES would run them
    # fewer (or zero) times and return 0 errors without a word.
    if (len(bench.STOCK_RMWS) < 3 or len(bench.WORKSPACE_LEGS) < 2
            or len(bench.WORKSPACE_MSG_CLASSES) < 2):
        print(f"FAIL  [type-class visuals: the loops above are driven by "
              f"bench.py's own tuples and one of them shrank "
              f"(STOCK_RMWS={bench.STOCK_RMWS}, "
              f"WORKSPACE_LEGS={bench.WORKSPACE_LEGS}, "
              f"WORKSPACE_MSG_CLASSES={bench.WORKSPACE_MSG_CLASSES}) — the "
              f"arms above proved less than they claim]")
        errors += 1
    return errors


def _series_for_stem(stem: str, label=None, csv=None):
    import plot
    return plot.Series(label or stem, csv or f"results_{stem}.csv", "-",
                       label or stem)


def _write_synthetic_results(results_dir: Path, series) -> None:
    """Hand-shaped results CSVs (the parse_results_csv column contract)
    so plot_group can render on a host with no real sweep — the
    NUMBERS are throwaway geometry, never a measurement (Principle
    #13); the render exists only to observe the footnote block."""
    for s in series:
        rows = ["payload_bytes,iterations,round_trip_p50_ns,"
                "round_trip_p99_ns,achieved_rate_hz"]
        for i, size in enumerate(bench.PAYLOAD_SIZES):
            p50 = 5_000 + 10 * i * (2 if "_image_" in s.csv_name else 1)
            rows.append(f"{size},1000,{p50},{p50 * 2},")
        (results_dir / s.csv_name).write_text("\n".join(rows) + "\n")


def check_type_class_footnote() -> int:
    """The matched-quantity footnote appears only when BOTH classes of
    one stack share the figure (plot._paired_class_stacks): an image
    cell beside a ROS 2 pod cell, or a workspace pod leg beside a
    variable leg. A lone tokened row — a --skip-missing render whose
    twin CSV is absent, a single image --series overlay — claims no
    pairing, and the incumbent (token-less) figures stay byte-identical.
    Pinned on the pure helper AND through a real render pair (the
    footnote text greppable in SVG output), so a caption-site revert to
    the old any(token) test fails here, not just a helper revert.

    The helper half is stdlib (plot.py imports matplotlib LAZILY, inside
    plot_group, so `import plot` succeeds without it); only the render
    half needs matplotlib and SKIPS loudly where it is absent — pinned
    by check_plot_arms_skip_without_matplotlib."""
    try:
        import plot
    except ImportError as e:
        print(f"skip  [type-class footnote: plot.py unimportable: {e}]")
        return 0
    errors = 0
    S = _series_for_stem
    img = S("jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0")
    pod = S("jazzy_cyclonedds_shm_rclcpp_be1_chrt0")
    pod_other = S("jazzy_fastdds_no_shm_rclcpp_be1_chrt0")
    zc_pod = S("jazzy_fastdds_zc_rclcpp_be1_chrt0")
    stock = S("jazzy_stock_rclcpp_chrt0")
    ws_pod = S("cerulion_workspace_split_pod_chrt0")
    ws_var = S("cerulion_workspace_split_chrt0")
    ws_mono = S("cerulion_workspace_mono_chrt0")
    iox2 = S("iox2_chrt0")
    overlay_img = S("jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0",
                    label="Image cell (pretty)",
                    csv="/elsewhere/results_jazzy_cyclonedds_shm_image_"
                        "rclcpp_be1_chrt0.csv")
    overlay_pod = S("jazzy_cyclonedds_shm_rclcpp_be1_chrt0",
                    label="Pod cell (pretty)",
                    csv="/elsewhere/results_jazzy_cyclonedds_shm_rclcpp_"
                        "be1_chrt0.csv")
    cases = [
        ("image + its pod twin", [img, pod], True),
        ("image + another rmw's pod cell (same stack)", [img, pod_other],
         True),
        ("image + the stock lane (pod class)", [img, stock], True),
        ("image + a zc pod row (FastDDS DataSharing lane)", [img, zc_pod],
         True),
        ("zc pod row alone", [zc_pod], False),
        ("image alone (single --series overlay)", [img], False),
        ("image + the native floor only", [img, iox2], False),
        ("workspace pod + its variable twin", [ws_pod, ws_var], True),
        ("workspace pod + the other leg's variable row", [ws_pod, ws_mono],
         True),
        ("workspace pod alone (--skip-missing, twin CSV absent)", [ws_pod],
         False),
        ("incumbent hero shape (no tokened row)",
         [iox2, ws_var, ws_mono, pod, stock], False),
        ("cross-stack only: workspace pod + ROS 2 image, no same-stack "
         "twin", [ws_pod, img], False),
        ("pretty-labelled overlays resolve via the csv stem",
         [overlay_img, overlay_pod], True),
    ]
    for desc, series, want in cases:
        got = bool(plot._paired_class_stacks(series))
        if got != want:
            print(f"FAIL  [type-class footnote gate: {desc}] want {want}, "
                  f"got {got}")
            errors += 1
        else:
            print(f"ok    [type-class footnote gate: {desc}] -> {got}")
    # Cross-pin the detector against the REAL enumeration: every ROS 2
    # cell bench.py can mint classifies — image rows variable, every
    # other matrix row (shm / no_shm / zc, both recv paths, both qos) and
    # both usage lanes pod — so the detector's stem alternation cannot
    # drift from bench.py's (a zc row read as None is how this arm was
    # born). Native lines classify None (no typed schema).
    unclassified = []
    for distro in bench.ROS2_DISTROS:
        for chrt in (0, 1):
            cells, _skips = bench.enumerate_ros2_cells(distro, chrt)
            for c in cells:
                want_cls = ("ros2", "variable" if c.msg == "image" else "pod")
                if plot._type_class_of(S(c.name)) != want_cls:
                    unclassified.append((c.name, want_cls))
    shm_modes = {c.shm for d in bench.ROS2_DISTROS
                 for c in bench.enumerate_ros2_cells(d, 0)[0]}
    if unclassified:
        print(f"FAIL  [type-class detector vs enumeration: "
              f"{len(unclassified)} cell(s) misclassified: "
              f"{unclassified[:6]}]")
        errors += 1
    elif "zc" not in shm_modes:
        print("FAIL  [type-class detector vs enumeration: the enumeration "
              "minted no zc cell, so the zc arm above proved nothing]")
        errors += 1
    else:
        print(f"ok    [type-class detector classifies every enumerated ROS 2 "
              f"cell (shm modes {sorted(shm_modes)}); native lines -> "
              f"{plot._type_class_of(iox2)}]")
    # Through the real render: the caption site must consult the gate.
    # plot_group imports matplotlib at CALL time, so probe it here — an
    # absent matplotlib is a loud skip of the render half, never a
    # traceback out of the gate.
    try:
        import matplotlib  # noqa: F401
    except ImportError as e:
        print(f"skip  [type-class footnote render: matplotlib absent on "
              f"this host — the caption-site render pin did not run: {e}]")
        return errors
    marker = "type classes: fixed array"
    with tempfile.TemporaryDirectory(prefix="footnote_") as td:
        d = Path(td)
        for desc, series, want in (("pair render", [img, pod], True),
                                   ("lone-image render", [img], False)):
            rd = d / desc.replace(" ", "_"); rd.mkdir()
            _write_synthetic_results(rd, series)
            out = rd / "probe.svg"
            plot.plot_group(series, rd, out, title="parity probe",
                            variant="quiescent")
            got = marker in out.read_text(encoding="utf-8")
            if got != want:
                print(f"FAIL  [type-class footnote render: {desc}] footnote "
                      f"{'present' if got else 'absent'}, want "
                      f"{'present' if want else 'absent'}")
                errors += 1
            else:
                print(f"ok    [type-class footnote render: {desc}] footnote "
                      f"{'present' if got else 'absent'}")
    return errors


def _extract_bash_block(path: Path, start: str, end_contains: str) -> str:
    """The SHIPPED text from the line equal to `start` through the first
    later line containing `end_contains`. Used to drive a block that is not
    a function — the backup/restore prologue — rather than re-typing it."""
    lines = path.read_text(encoding="utf-8").splitlines()
    try:
        i = next(k for k, ln in enumerate(lines) if ln.strip() == start.strip())
    except StopIteration:
        raise RuntimeError(f"{path.name}: no line equal to {start!r}")
    try:
        j = next(k for k in range(i, len(lines)) if end_contains in lines[k])
    except StopIteration:
        raise RuntimeError(f"{path.name}: no line containing {end_contains!r} "
                           f"after {start!r}")
    return "\n".join(lines[i:j + 1])


def _extract_bash_function(path: Path, name: str) -> str:
    """The SHIPPED text of one bash function, from `name() {` to the
    closing brace at column 0. Extracted rather than re-typed so the arms
    below drive the real thing; a rename or a reshaped definition fails
    here loudly instead of testing a stale copy."""
    lines = path.read_text(encoding="utf-8").splitlines()
    start = next((i for i, ln in enumerate(lines)
                  if ln.startswith(f"{name}() {{")), None)
    if start is None:
        raise RuntimeError(f"{path.name}: no `{name}() {{` definition found")
    end = next((i for i in range(start + 1, len(lines))
                if lines[i] == "}"), None)
    if end is None:
        raise RuntimeError(f"{path.name}: `{name}` has no closing brace at "
                           f"column 0")
    return "\n".join(lines[start:end + 1])


def check_loaned_column_is_first_match() -> int:
    """`compile_csv.loaned_from_log` takes the FIRST `loaned=` match in a
    cell's node log — the premise `latency_node.cpp` cites for forcing
    `loaned_ = false` on the variable class.

    `run_bench.sh` appends all three nodes into ONE `<cell>_<size>_node.log`
    concurrently, so which node writes first is a race. With `re.search`
    a single disagreeing node decides the whole cell's `loaned` column,
    which is why the override belongs in ALL THREE binaries and not only
    in the two that publish. The C++ cannot be driven on a host with no
    ROS 2 distro; the consumer contract it leans on can, and it is the
    half that would silently invalidate the fix if anyone "improved"
    `re.search` into a last-match or majority read."""
    errors = 0
    with tempfile.TemporaryDirectory(prefix="loaned_") as td:
        raw = Path(td)
        cell = "jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0"
        cases = [
            ("all three agree on 0", 64,
             "ping_node ready (msg=image payload=64 loaned=0 qos=be1)\n"
             "latency_node ready (msg=image payload=64 loaned=0 qos=be1)\n"
             "pong_node ready (msg=image payload=64 loaned=0 qos=be1)\n",
             "0"),
            ("a disagreeing FIRST line decides the column", 256,
             "latency_node ready (msg=image payload=256 loaned=1 qos=be1)\n"
             "ping_node ready (msg=image payload=256 loaned=0 qos=be1)\n",
             "1"),
            ("the same disagreement, other interleaving", 1024,
             "ping_node ready (msg=image payload=1024 loaned=0 qos=be1)\n"
             "latency_node ready (msg=image payload=1024 loaned=1 qos=be1)\n",
             "0"),
            ("a log with no loaned= line", 4096,
             "latency_node ready (payload=4096)\n", ""),
        ]
        for desc, size, text, want in cases:
            (raw / f"{cell}_{size}_node.log").write_text(text,
                                                         encoding="utf-8")
            got = compile_csv.loaned_from_log(raw, cell, size)
            if got != want:
                print(f"FAIL  [loaned column: {desc}] want {want!r}, got "
                      f"{got!r}")
                errors += 1
            else:
                print(f"ok    [loaned column: {desc}] -> {got!r}")
        got = compile_csv.loaned_from_log(raw, cell, 16384)   # no such log
        if got != "":
            print(f"FAIL  [loaned column: a missing log] want '', got {got!r}")
            errors += 1
        else:
            print("ok    [loaned column: a missing log -> '']")
    return errors


_WORKSPACE_DIR = Path(__file__).resolve().parent / "workspace"
_GRAPHS_DIR = _WORKSPACE_DIR / "graphs"
_SCHEMAS_DIR = _WORKSPACE_DIR / "schemas"
# 12 bytes of fixed fields (prep + stamp_hi + stamp_lo) precede the baked
# array, so a pod payload of N total bytes carries `uint8[N-12] data`.
# Hand-written here on purpose: this is the CONTRACT, and both the build
# script and the runner are checked against it rather than against each
# other.
_POD_FIXED_BYTES = 12


def _graph_schema_declarations() -> "list":
    """(graph path, line no, schema value) for every `schema:` a bench graph
    declares. A hand parse, not a YAML load: the harness is stdlib-only and
    the shape is one key per line."""
    out = []
    for g in sorted(_GRAPHS_DIR.glob("*.yaml")):
        for i, line in enumerate(g.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if stripped.startswith("#") or not stripped.startswith("schema:"):
                continue
            out.append((g, i, stripped[len("schema:"):].strip()))
    return out


def _registered_msg_store_schemas() -> "set":
    """The QUALIFIED names the workspace `.msg` store registers —
    `schemas/<pkg>/msg/<Type>.msg` registers `<pkg>/<Type>`, the package
    coming from the DIRECTORY. That package is what `parse_rosmsg` folds
    into the wire `SCHEMA_HASH`, which is why the fixed-POD type is
    registered here and not as a bare workspace YAML."""
    return {f"{d.parent.name}/{m.stem}"
            for d in _SCHEMAS_DIR.glob("*/msg") if d.is_dir()
            for m in d.glob("*.msg")}


def _registered_workspace_schemas() -> "set":
    """The bare schema names `schemas/*.yaml` registers — the keys under the
    top-level `schemas:` map, which is how the CLI resolver reads them."""
    names = set()
    for y in sorted(_SCHEMAS_DIR.glob("*.yaml")):
        in_map = False
        for line in y.read_text(encoding="utf-8").splitlines():
            if line.startswith("schemas:"):
                in_map = True
                continue
            if in_map:
                if line[:1] not in (" ", "\t", ""):
                    in_map = False
                    continue
                if line.startswith("  ") and not line.startswith("   ") \
                        and line.rstrip().endswith(":"):
                    names.add(line.strip().rstrip(":").strip('"'))
    return names


def check_bench_graph_schemas_resolve() -> int:
    """Every `schema:` a bench graph declares must name something the
    workspace resolver can SEE.

    `graph run`'s schema gate is fail-closed (#670): a name that resolves to
    nothing REFUSES the run. That is not theoretical here — both fixed-POD
    graphs declared `cer_bench_msgs/PodPayload`, a name registered nowhere,
    so neither advertised POD leg could start, and the failure surfaced only
    when somebody tried to run one. This arm asks the question before a
    measurement instead.

    Three resolvable shapes, matching what the CLI accepts: a
    PACKAGE-QUALIFIED `pkg/Type` the workspace `.msg` store registers; the
    same shape carried by the vendored ROS 2 corpus (checked against
    `native_ros2_messages/msg/<pkg>/<Type>.msg`, not merely against the
    presence of a slash — one bug HERE was a slash: `cer_bench_msgs/PodPayload`
    read as qualified and resolved to nothing); or a BARE name registered by
    a workspace `schemas/*.yaml`. The bare case is
    what the POD class needs: `node_metadata::collect_use_imports` derives
    the bare leaf for a type brought in through a same-file `mod` (engine
    `089e47d86` — a same-file module is a Rust namespace, not a schema
    package), and the node is the source of truth for what it publishes, so
    the graph and the registration must both spell it bare.

    Stdlib-only and always runs; the `graph validate` arm below is the same
    question asked of the real CLI, and skips where no CLI is built."""
    errors = 0
    decls = _graph_schema_declarations()
    registered = _registered_workspace_schemas()
    stored = _registered_msg_store_schemas()
    builtin_root = Path(__file__).resolve().parents[2] / "crates" / "native_ros2_messages" / "msg"
    if not builtin_root.is_dir():
        print(f"FAIL  [bench graph schemas: {builtin_root} is not a directory "
              f"— the package-qualified half of this arm cannot check "
              f"anything]")
        return 1
    if not decls:
        print("FAIL  [bench graph schemas: no `schema:` declaration found in "
              f"{_GRAPHS_DIR} — this arm would pass vacuously]")
        return 1
    for path, lineno, value in decls:
        if "/" in value:
            pkg, _, leaf = value.partition("/")
            if value in stored:
                print(f"ok    [bench graph schemas: {path.name}:{lineno} "
                      f"`{value}` — registered by the workspace .msg store]")
                continue
            if not (builtin_root / pkg / "msg" / f"{leaf}.msg").is_file() \
                    and not (builtin_root / pkg / f"{leaf}.msg").is_file():
                print(f"FAIL  [bench graph schemas: {path.name}:{lineno} "
                      f"declares `schema: {value}` — it LOOKS qualified but "
                      f"the vendored corpus carries no {pkg}/{leaf} and the "
                      f"workspace .msg store registers no such entry "
                      f"(stored: {sorted(stored) or 'none'}). `graph run`'s "
                      f"fail-closed schema gate REFUSES such a graph, so "
                      f"this leg could not start]")
                errors += 1
                continue
            verdict = f"built-in ({pkg})"
        elif value in registered:
            verdict = "registered by a workspace schemas/*.yaml"
        else:
            print(f"FAIL  [bench graph schemas: {path.name}:{lineno} declares "
                  f"`schema: {value}` — a BARE name no workspace "
                  f"schemas/*.yaml registers (registered: "
                  f"{sorted(registered) or 'none'}). `graph run`'s "
                  f"fail-closed schema gate REFUSES such a graph, so this "
                  f"leg could not start]")
            errors += 1
            continue
        print(f"ok    [bench graph schemas: {path.name}:{lineno} "
              f"`{value}` — {verdict}]")
    if not (registered or stored):
        print("FAIL  [bench graph schemas: the workspace registers NOTHING "
              "(no schemas/*.yaml key, no .msg store entry) — the "
              "registration half of this arm proved nothing]")
        errors += 1
    return errors


_POD_DATA_LINE = re.compile(r"^uint8\[(\d+)\] data$", re.M)


def _committed_pod_data_len(text):
    """The `data` array length the committed PodPayload.msg DECLARES, or None.

    ANCHORED to a whole line, and shared by both gates that read it, because
    they disagreed once: one tested `'uint8[52] data' in text` and the other
    matched this regex, so a file whose real field line said 1 MiB passed the
    substring gate on the strength of a HEADER LINE quoting the committed
    default. The header legitimately quotes array lengths (it explains the
    YAML `MAX_FIXED_ARRAY_LEN` ceiling with `uint8[16777204]`), so prose
    matching is not a hypothetical here."""
    m = _POD_DATA_LINE.search(text)
    return int(m.group(1)) if m else None


def check_pod_schema_is_never_rewritten() -> int:
    """The pod class's registration is a NAME registration the runner never
    writes, and it still describes the first pinned size.

    Previously the recorder took a bag channel's `wire_fixed_size`
    from `schemas/cer_bench_msgs/msg/PodPayload.msg`, and the pod Rust type
    is generated per sweep size into a crate's OUT_DIR the CLI cannot see —
    so run_workspace.sh REGENERATED the tracked file before every rebuild
    and restored it from an EXIT/INT/TERM trap. Reviewers objected to a
    script editing checked-in files at all: a crashed run leaves the
    checkout dirty and two runs collide. The resolution has since moved onto
    the producing node's own metadata, so the file is now read by `graph
    validate` and by nothing that runs per size.

    Three claims, and the first is the one that can rot silently:

      * the runner has NO writer for the file. Asserted on the shipped
        TEXT, because the per-size path needs a full sweep to reach and
        this host cannot run one — and on the text with comments STRIPPED,
        so the paragraph above `run_size_once` explaining the removal is
        not itself read as a writer.
      * the committed file still describes the FIRST pinned size, so a
        fresh clone can `cerulion graph validate rtt_bench_pod` before any
        sweep.
      * its header survives, since that header is where a reader learns
        why a registration whose length does not track the sweep is
        correct."""
    errors = 0
    if not _RUN_WORKSPACE.exists():
        print(f"FAIL  [pod schema: {_RUN_WORKSPACE} missing]")
        return 1
    committed_path = _SCHEMAS_DIR / "cer_bench_msgs" / "msg" / "PodPayload.msg"
    try:
        committed = committed_path.read_text(encoding="utf-8")
    except OSError as e:
        # Fail attributably, never with a traceback out of the gate: the
        # registration going missing is the exact defect this arm exists
        # to catch, and a crash here takes every later arm with it.
        print(f"FAIL  [pod schema: the registered descriptor "
              f"{committed_path} is unreadable ({e}) — the pod graphs name "
              f"cer_bench_msgs/PodPayload and `graph run`'s fail-closed "
              f"schema gate refuses a name the workspace cannot resolve]")
        return errors + 1
    header_marker = "WHY THIS FILE EXISTS"
    if header_marker not in committed:
        print(f"FAIL  [pod schema: the committed PodPayload.msg has no "
              f"{header_marker!r} header — a reader has no way to learn why "
              f"its array length does not track the sweep]")
        errors += 1

    # --- no writer, on the comment-stripped shipped text -------------------
    # WHOLE comment LINES only, never a mid-line split at the first `#`.
    # A mid-line split fails OPEN, and its exemplar is already in this very
    # file: `sed -e "s/\\(#\\[cerulion_node(period_ms = \\)..."` — the ONE
    # tracked-file rewrite the runner still performs — truncates to
    # `sed -e "s/\\(`, and so does every `${#arr[@]}` and `${x#*:}`. A
    # re-added writer sharing a line with any of those shapes would be
    # invisible. Dropping only full-line comments cannot over-strip; its
    # residual is the harmless direction (a trailing comment naming a
    # forbidden token fails the gate loudly, and there are none today).
    stripped = "\n".join(
        ln for ln in _RUN_WORKSPACE.read_text(encoding="utf-8").splitlines()
        if not ln.lstrip().startswith("#"))
    # Identifier names AND the file's own literal path — but BOTH are
    # dodgeable: a writer with
    # renamed variables and a path composed from parts
    # (`POD_REG="$SCRIPT_DIR/schemas/$POD_PKG/msg/$POD_TYPE.msg"` then
    # `sed -i ... "$POD_REG"`) contains none of the five tokens and really
    # did rewrite the tracked file with this gate green.
    #
    # Two rules that a rename or a composed path cannot walk past, because
    # they name what the FILE is rather than what the code calls it:
    #
    #   `schemas` — any writer has to reach the directory somehow, and it is
    #     the ASSIGNMENT line that carries it, which a per-line "write verb
    #     near a path" scan misses (the write and the path sit on different
    #     lines). Reproduced: a writer with renamed variables and
    #     `"$SCRIPT_DIR/schemas/$POD_PKG/msg/$POD_TYPE.msg"` walked past a
    #     pure token list.
    #   `.msg` — closes the runtime-DISCOVERY shape a path scan cannot see at
    #     all: `find "$SCRIPT_DIR" -name '*.msg'` then `> "$target"` names no
    #     directory and no variable this gate knows. (`find` itself cannot be
    #     banned — the shipped script uses it three times for /dev/shm
    #     cleanup.)
    #
    # `sed -i` is banned outright on its own ground: the runner's one
    # legitimate rewrite is `sed -e ... > "$PING_SRC"`, never in-place, so an
    # in-place edit is always someone editing a file they should not.
    #
    # MEASURED on the shipped script: zero code lines mention `schemas` or
    # `.msg` (every `cer_bench_msgs` occurrence is a comment, and the only
    # bare `PodPayload` on a code line is the Rust type `PodPayloadShm` —
    # which is why a bare-`PodPayload` rule is NOT used).
    #
    # RESIDUAL: this is a source scan, so a writer that assembles the
    # extension itself (`-name "*.ms""g"`) still passes. What it takes to
    # dodge all three rules is deliberate obfuscation; what it now catches is
    # every shape an accidental regression takes, plus the three that were
    # reproduced against earlier forms of this check.
    writers = [tok for tok in ("write_pod_schema", "POD_SCHEMA_BAK",
                               "POD_SCHEMA_MSG", "cer_bench_msgs/msg",
                               "PodPayload.msg", "schemas", ".msg", "sed -i")
               if tok in stripped]
    if writers:
        print(f"FAIL  [pod schema: run_workspace.sh still carries "
              f"{writers} — the runner must not write a tracked schema "
              f"file. The RECORDER reads the layout from the "
              f"producing node (OutputMeta::wire_fixed_size), so a per-size "
              f"rewrite buys nothing and costs a dirty checkout on every "
              f"crashed run]")
        errors += 1
    else:
        print("ok    [pod schema: run_workspace.sh has no writer for the "
              "registration]")
    # ANTI-TAUTOLOGY, anchored in the LATE region the claim is about. A
    # re-added writer would sit beside the per-size rebuild inside
    # `run_size_once`, not beside the `PING_SRC_BAK` backup block near the
    # top, so anchoring on an early token would let a stripper that TRUNCATES
    # (rather than empties) pass while blinding the scan where it matters.
    # Token-anchored, never line-numbered — this file moves.
    # This token is also the per-size rebuild itself, whose presence the
    # deleted call-site arm used to assert — kept here rather than lost.
    anchor = 'CER_BENCH_POD_BYTES="$SIZE" cargo build'
    if anchor not in stripped:
        print(f"FAIL  [pod schema: the stripped view of run_workspace.sh has "
              f"no {anchor!r} — either the per-size pod rebuild is gone, or "
              f"the stripper ate the region the no-writer check above "
              f"scans, which would make it vacuous]")
        errors += 1
    else:
        print("ok    [pod schema: the stripped view still sees the per-size "
              "pod rebuild (the no-writer check is not vacuous)]")

    # --- the committed length is the first pinned size --------------------
    # ANCHORED, the same regex `check_pod_schema_hash_matches_the_generated_type`
    # uses — not a substring. The header is chatty and already quotes array
    # lengths (`uint8[16777204]` in the YAML-ceiling paragraph), so a
    # substring test is satisfied by PROSE: reproduced, a file whose real
    # field line said `uint8[1048564] data` passed this arm because one
    # header line mentioned the committed default, and the `.orig` hint
    # below — which lives on this branch — never fired.
    first = bench.PAYLOAD_SIZES[0]
    want_first = f'uint8[{first - _POD_FIXED_BYTES}] data'
    declared = _committed_pod_data_len(committed)
    if declared != first - _POD_FIXED_BYTES:
        # Name the most likely cause on any machine that ran the earlier
        # bench. That runner rewrote this file per size and restored it from
        # a trap; a SIGKILLed sweep could leave the rewrite in place with the
        # pristine bytes in a `.orig` sibling. The recovery that used to heal
        # that is gone with the rest of the mechanism, and it is deliberately
        # not re-added (it would be tracked-file writing again) — so say so
        # here instead, which is where the symptom actually shows up.
        leftover = committed_path.with_suffix(committed_path.suffix + ".orig")
        hint = (f" A `{leftover.name}` sits beside it, so this is almost "
                f"certainly a sweep by an older runner that was killed mid-run, "
                f"not a regression: restore the file from git and delete the "
                f"`.orig`." if leftover.exists() else "")
        says = ("no 'uint8[<N>] data' line at all" if declared is None
                else f"uint8[{declared}] data")
        print(f"FAIL  [pod schema committed default] PodPayload.msg must "
              f"ship describing the FIRST pinned size {first} "
              f"({want_first!r}) so a fresh clone can `graph validate` "
              f"before any sweep; it declares {says}.{hint}")
        errors += 1
    else:
        print(f"ok    [pod schema committed default describes size {first}]")
    return errors


def check_pod_sweep_leaves_the_checkout_clean() -> int:
    """A pod sweep must leave the one tracked file it still writes
    byte-identical, on a clean AND a failing exit.

    Driven through the SHIPPED backup block and the SHIPPED trap, with a
    rewrite of the ping source in between. That is the WHOLE claim: the
    registration is deliberately NOT in this fixture, because nothing in
    the extracted block can touch it and asserting that an untouched file
    is untouched proves nothing. "The runner never writes the
    registration" is `check_pod_schema_is_never_rewritten`'s property,
    asserted where it can actually fail — on the shipped source.

    A PREMISE assert guards the extraction: the block must really carry the
    ping restore, or a scoping change would silently leave this driving an
    empty script and passing."""
    errors = 0
    if not _RUN_WORKSPACE.exists():
        print(f"FAIL  [pod sweep clean: {_RUN_WORKSPACE} missing]")
        return 1
    try:
        backup = _extract_bash_block(_RUN_WORKSPACE,
                                     "if [ -f \"$PING_SRC_BAK\" ]; then",
                                     "EXIT INT TERM")
    except RuntimeError as e:
        print(f"FAIL  [pod sweep clean: {e}]")
        return errors + 1
    # PREMISE: the extracted block really is the ping backup + trap. Without
    # this, a scoping change that shrank the block would leave the fixture
    # below driving a script that restores nothing, and every arm would pass.
    for token in ('cp -f "$PING_SRC" "$PING_SRC_BAK"',
                  'mv -f "$PING_SRC_BAK" "$PING_SRC"'):
        if token not in backup:
            print(f"FAIL  [pod sweep clean: the extracted block does not "
                  f"carry {token!r} — it is not the backup+trap this arm "
                  f"means to drive, so the assertions below would be "
                  f"vacuous]")
            return errors + 1
    restore_script = """set -uo pipefail
SCRIPT_DIR="$1"
PING_SRC="$SCRIPT_DIR/ping_src.rs"
PING_SRC_BAK="$PING_SRC.orig"
stop_usage_sampler() { :; }
%(backup)s
printf 'fn main() { /* rewritten */ }\\n' > "$PING_SRC"
grep -q rewritten "$PING_SRC" || exit 8
exit "$2"
""" % {"backup": backup}
    for desc, exit_code in (("a clean exit", 0), ("a failing exit", 1)):
        with tempfile.TemporaryDirectory(prefix="clean_") as td:
            d = Path(td)
            ping = d / "ping_src.rs"
            ping.write_text("fn main() {}\n", encoding="utf-8")
            sc = d / "restore.sh"
            sc.write_text(restore_script, encoding="utf-8")
            r = subprocess.run(["bash", str(sc), str(d), str(exit_code)],
                               capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=60)
            ping_after = ping.read_text(encoding="utf-8")
            leftovers = sorted(q.name for q in d.rglob("*.orig"))
            # rc 8 is the fixture's own "the rewrite did not land" signal;
            # anything outside {0, 1} means the driven block itself failed
            # (e.g. `set -u` on a variable a regrown line references), which
            # must be a FAIL rather than an accepted exit.
            ok = (r.returncode in (0, 1)
                  and ping_after == "fn main() {}\n"
                  and not leftovers)
            if not ok:
                print(f"FAIL  [pod sweep clean: {desc}] the rewritten file "
                      f"must be back to its committed bytes with no .orig "
                      f"left; rc={r.returncode}, "
                      f"ping_restored={ping_after == 'fn main() {}' + chr(10)}"
                      f", leftovers={leftovers}\n{r.stderr.rstrip()[-300:]}")
                errors += 1
            else:
                print(f"ok    [pod sweep clean: {desc} — ping restored, "
                      f"no .orig left]")
    return errors


def _find_cerulion_cli():
    """This tree's own `cerulion`, DATED against this tree's source, or None.

    THIS tree's, deliberately: PR #733's review validated using a binary built
    from a different checkout, which reported a different check count and
    missed a real failure. The binary you validate with is part of the
    test.

    The docstring said that before this fix and the code did not:
    it walked target/debug, target/release and then PATH, took the first
    hit, and asked nothing about WHICH source built it — so a month-old
    artifact, or a `cerulion` installed from another checkout, answered
    just as well. The resolution now lives in bench.resolve_cerulion_cli
    (ONE implementation, shared with the workspace legs' CERULION=
    refusal) and declines a stale or foreign binary rather than
    validating through it. A declined binary makes the arms below SKIP,
    loudly, naming the rebuild — which is what "part of the test" has to
    mean if it means anything.

    Returns `(cli, note)` — the note is handed BACK rather than printed
    here, because the line an operator greps is the arm's own `skip`
    line, and "no binary, build one" and "a binary was found and
    declined" are different situations with different remedies. Printing
    the reason on a separate line while the actionable line still said
    "no `cerulion` binary" told an operator to build one they already
    had.

    The resolver DECLINES an undatable binary (the `CERULION=` override
    refusal takes the opposite view of the same condition, where a refusal
    would block a whole campaign rather than skip two arms). Accepting one
    here would restore exactly the earlier behaviour this function
    exists to end."""
    return bench.resolve_cerulion_cli()


# The size marker `pod_codegen.rs` writes into every generated file — the
# only statement of WHICH sweep size a generated copy was baked at.
_POD_BAKED_SIZE_LINE = re.compile(r"^// Baked payload size: (\d+) bytes total",
                                  re.M)


def _pod_msg_redeclared_at(committed: str, total_size: int) -> str:
    """The committed `PodPayload.msg` with its `uint8[N] data` line
    re-declared for a payload of `total_size` bytes.

    Everything else is the committed file BYTE FOR BYTE — header comments,
    the three fixed fields, the field ORDER, the trailing newline — so the
    only thing that moves is the one number a sweep moves. That is what
    keeps the reconstruction faithful: the property being asserted is that the
    `.msg` STORE folds the PACKAGE into the hash, and a reconstruction that
    also normalised the field list could hash equal while the shipped
    registration was wrong.

    Raises `ValueError` rather than guessing: a size at or below the fixed
    prefix has no array to declare, and a file with no (or more than one)
    `uint8[N] data` line is not the registration this rewrite understands.
    """
    if total_size <= _POD_FIXED_BYTES:
        raise ValueError(
            f"a pod payload of {total_size} B is at or below the "
            f"{_POD_FIXED_BYTES} B fixed prefix, so it declares no array")
    text, n = _POD_DATA_LINE.subn(
        f"uint8[{total_size - _POD_FIXED_BYTES}] data", committed)
    if n != 1:
        raise ValueError(
            f"expected exactly one `uint8[<N>] data` line to re-declare, "
            f"found {n}")
    return text


def _registered_pod_hash(cli, workspace_dir, env=None):
    """`(hash, detail)` from `cerulion schema info cer_bench_msgs/PodPayload`
    run in `workspace_dir`.

    `hash` is None when the command could not be RUN, failed, or printed no
    `hash:` line, and `detail` then carries the reason for the caller to print.

    TOTAL on purpose. `subprocess.run(timeout=…)` raises `TimeoutExpired`,
    which is a `SubprocessError` and NOT an `OSError`, so an escaping raise
    would take the whole parity run down with a traceback — the one thing
    every arm in this file is written not to do.

    RESIDUAL, stated rather than fixed: on a timeout `subprocess.run` kills
    the child and then `communicate()`s with no bound, so a DAEMONIZED
    grandchild holding the pipes would block there. Unreachable today — the
    only such grandchild is `cerulion-netd`, and every caller passes
    `CERULION_NETWORK=off`, which returns before a spawn."""
    try:
        r = subprocess.run([str(cli), "schema", "info", "cer_bench_msgs/PodPayload"],
                           cwd=str(workspace_dir), capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=300,
                           env=env)
    except (OSError, subprocess.SubprocessError) as e:
        return None, f"`schema info` could not be run to completion ({e})"
    m = re.search(r"^hash:\s*(0x[0-9a-fA-F]+)", r.stdout, re.M)
    if r.returncode != 0 or m is None:
        return None, f"rc={r.returncode}\n{(r.stdout + r.stderr)[-400:]}"
    return int(m.group(1), 16), ""


@contextlib.contextmanager
def _reconstructed_pod_workspace(committed_path: Path, committed: str,
                                 total_size: int):
    """A throwaway workspace whose `schemas/` tree is THIS workspace's, with
    the pod registration re-declared at `total_size`.

    The WHOLE tree is copied, not just the one file: any sibling that would
    shadow or make the name ambiguous in the real workspace does so here too,
    so the CLI resolves exactly what it would resolve at home — only at the
    swept size. `CerulionWorkspace::discover` wants a `[workspace]`
    Cargo.toml and a `graphs/` directory to find a root at all; nothing here
    is ever built.

    No tracked file is written, which is the point: the runner's per-size
    rewrite of the registration was removed, and re-introducing one,
    even briefly, even in a gate, would cost a dirty checkout on every
    crashed run."""
    with tempfile.TemporaryDirectory(prefix="podreg_") as td:
        root = Path(td)
        (root / "Cargo.toml").write_text(
            '[workspace]\nresolver = "2"\nmembers = []\n', encoding="utf-8")
        (root / "graphs").mkdir()
        shutil.copytree(_SCHEMAS_DIR, root / "schemas")
        # The registration's OWN relative tail, never a re-spelling of it: a
        # rename would otherwise write a stray file beside the real one and
        # the arm would FAIL with an affirmatively wrong cause.
        target = root / "schemas" / committed_path.relative_to(_SCHEMAS_DIR)
        if not target.exists():
            raise OSError(f"{target} is not in the copied schemas/ tree")
        target.write_text(_pod_msg_redeclared_at(committed, total_size),
                          encoding="utf-8")
        yield root


def _pod_target_roots(ctd):
    """`(active, inactive)` target roots for the BENCH workspace.

    `active` is where cargo writes the pod crate's output right now;
    `inactive` is the other root a copy might be sitting in, or `None` when
    there is only one.

    A relative `CARGO_TARGET_DIR` is resolved BY CARGO, from the directory
    cargo is invoked in — and `run_workspace.sh` invokes it after
    `cd "$SCRIPT_DIR"`, i.e. inside `benches/latency/workspace`. So
    `CARGO_TARGET_DIR=target-cache` means `<bench workspace>/target-cache`
    to the build and `<cwd>/target-cache` to a naive `Path(env)` — a
    directory nothing writes, in a process the operator can launch from
    anywhere. `bench.py::cargo_target_dir` makes exactly this correction for
    the REPO root; this is the same correction against the other cargo cwd,
    and the bases genuinely differ, which is why it cannot just call that.

    Only the ACTIVE root is searched by the caller. When `CARGO_TARGET_DIR`
    is set, cargo writes NOTHING to `<bench workspace>/target`, so anything
    still sitting there is from a different configuration: searching it too
    could only let an older copy win by mtime and have the arm validate — or
    `skip` on — output the current configuration does not produce. It is
    still REPORTED when the active root is empty, because "you built with a
    different CARGO_TARGET_DIR" and "you never built" need different
    remedies."""
    default = _WORKSPACE_DIR / "target"
    if not ctd:
        return default, None
    target = Path(ctd)
    active = target if target.is_absolute() else _WORKSPACE_DIR / target
    # Resolved before comparing: `CARGO_TARGET_DIR=target` and an absolute
    # spelling of the same directory are the same root, and reporting a
    # phantom "other configuration" for them would be noise.
    try:
        same = active.resolve() == default.resolve()
    except OSError:
        same = active == default
    return active, (None if same else default)


def _pod_hash_verdict(matches, prior_errors, superseded):
    """Which line the schema-hash arm prints: `mismatch` / `held_but_failed`
    / `superseded` / `ok`.

    Pure, because the ONE thing it must never do is unreachable from a host
    with no built CLI and so invisible to every other arm here: print a green
    `ok` for a run that has ALREADY failed. The committed-size branch above
    can record a control failure and fall straight through to this verdict,
    and the hash comparison it then reports on is a DIFFERENT question — so
    `registered == generated` can hold while the arm is failing for another
    reason entirely. `check_pod_registration_reconstruction` gates its own
    success line on `errors == 0`; this is that rule, made explicit and
    testable rather than repeated by hand.

    `held_but_failed` exists so the comparison's outcome is still stated. It
    is information the FAIL above does not carry, and dropping it would
    replace a misleading line with a missing one."""
    if not matches:
        return "mismatch"
    if prior_errors:
        return "held_but_failed"
    if superseded is not None:
        return "superseded"
    return "ok"


def _superseded_reason(superseded, readable_paths):
    """Why the newest copy was passed over: `unmarked` or `unreadable`.

    `_select_generated_copy` sets `superseded` for EITHER reason, and the two
    are distinguishable right here — a copy that could not be read never
    reached `readable` at all. Saying "carries no marker" on the unreadable
    path would assert something about a file's CONTENT that was never read:
    the same affirmatively-wrong-cause pattern the `not built` branch of this
    arm goes out of its way to avoid."""
    return "unmarked" if superseded in readable_paths else "unreadable"


def _select_generated_copy(readable, newest_candidate):
    """`(row, superseded_path)` — the newest generated copy that STATES its
    baked size, plus the newest copy in the tree when that is a DIFFERENT one
    (the caller says so out loud and downgrades its verdict).

    `readable` is `(path, text, baked_or_None)` in ascending mtime order;
    `newest_candidate` is the newest copy that EXISTS, readable or not (`None`
    only when there are none at all). REQUIRED, not defaulted: a default would
    let the call site quietly drop it and take the unreadable half of the
    supersession rule with it — which is a mutant no table over this function
    can see, because the table supplies the argument itself.
    Returns `(None, None)` when nothing states a size: such a copy cannot be
    compared against anything, and guessing is how a comparison becomes
    decorative.

    Both reasons a copy can be passed over are reported the same way, because
    the operator-visible consequence is the same: the comparison is evidence
    about a copy that is not what this tree currently generated. Missing the
    UNREADABLE half is how the verdict came to say "the newest build dir"
    about one that was not.

    Pure so the choice has an oracle: this is the arm's whole reach — pick
    the wrong copy and the comparison is about the past."""
    marked = [row for row in readable if row[2] is not None]
    if not marked:
        return None, None
    row = marked[-1]
    superseded = (newest_candidate
                  if newest_candidate is not None and newest_candidate != row[0]
                  else None)
    return row, superseded


def check_pod_registration_reconstruction() -> int:
    """The stdlib half of the swept-tree comparison, against hand oracles:
    the re-declare rewrite, the copy SELECTION, and the marker the selection
    reads.

    These three are what let the arm below compare a SWEPT tree instead of
    skipping it, and each can fail quietly: a rewrite that changed more than
    the array length would make the comparison meaningless while still
    reporting `ok`; a selector that took the wrong copy would report about
    the past; and the marker is written by ONE line of pod_codegen.rs, so
    rewording that comment would turn the whole arm into a permanent `skip`.
    No CLI, no build — runs everywhere."""
    errors = 0
    # The prose deliberately carries the hazard shape `uint8[N] data`, so
    # `_POD_DATA_LINE`'s `^`/`$` anchors are LOAD-BEARING here: unanchor them
    # and this header matches too, `subn` returns 2, and the refusal fires.
    # (The real PodPayload.msg header quotes an array length in prose too, but
    # as `uint8[16777204] —`, without the ` data` suffix — so unanchoring
    # would NOT trip on the shipped file today. This fixture adds the suffix
    # deliberately: the hazard is pinned before it ships, not after.)
    committed = ("# header: see uint8[16777204] data for the YAML ceiling\n"
                 "uint32 prep\nuint32 stamp_hi\nuint32 stamp_lo\n"
                 "uint8[52] data\n")
    want = ("# header: see uint8[16777204] data for the YAML ceiling\n"
            "uint32 prep\nuint32 stamp_hi\nuint32 stamp_lo\n"
            "uint8[1012] data\n")
    try:
        got = _pod_msg_redeclared_at(committed, 1024)
        identity = _pod_msg_redeclared_at(committed, 64)
    except ValueError as e:
        # An unanchored `_POD_DATA_LINE` lands HERE (two matches), which is
        # the point of the fixture — but a traceback would take the whole
        # parity run down, so it is reported like every other arm.
        print(f"FAIL  [pod registration reconstruction: re-declaring the "
              f"fixture refused ({e}) — most likely `_POD_DATA_LINE` lost its "
              f"`^`/`$` anchors and now matches the header's prose too]")
        return 1
    if got != want:
        print(f"FAIL  [pod registration reconstruction: re-declaring at "
              f"1024 B must move ONLY the field line — the header's prose "
              f"array length is not a declaration.\n  got:  {got!r}\n"
              f"  want: {want!r}]")
        errors += 1
    # The identity case is not special-cased anywhere, and must not be: the
    # arm below reconstructs at the committed size as its own control, so a
    # rewrite that introduced drift there would poison the control too.
    if identity != committed:
        print("FAIL  [pod registration reconstruction: re-declaring at the "
              "committed size must reproduce the committed bytes]")
        errors += 1
    # Refusals, so a nonsense size or an unrecognised file cannot be silently
    # turned into a plausible-looking registration.
    for bad_size, what in ((_POD_FIXED_BYTES, "a size equal to the fixed prefix"),
                           (0, "a zero size")):
        try:
            _pod_msg_redeclared_at(committed, bad_size)
        except ValueError:
            pass
        else:
            print(f"FAIL  [pod registration reconstruction: {what} must be "
                  f"REFUSED, not rewritten]")
            errors += 1
    for text, what in ((("uint32 prep\n"), "no `uint8[<N>] data` line"),
                       (committed + "uint8[7] data\n", "TWO `uint8[<N>] data` lines")):
        try:
            _pod_msg_redeclared_at(text, 1024)
        except ValueError:
            pass
        else:
            print(f"FAIL  [pod registration reconstruction: a file with "
                  f"{what} must be REFUSED — rewriting one of two "
                  f"declarations would register a shape nothing generates]")
            errors += 1

    # ---- the SELECTION, against a hand-written table --------------------
    a, b, c = Path("a"), Path("b"), Path("c")
    table = [
        # (readable rows ascending by mtime, newest EXISTING copy,
        #  expected picked path, expected superseded)
        ([(a, "", 64)], a, a, None),
        # the newest states a size -> it wins outright, nothing superseded
        ([(a, "", 64), (b, "", 1024)], b, b, None),
        # the newest is UNMARKED -> fall back, and SAY which copy was passed
        ([(a, "", 64), (b, "", None)], b, a, b),
        # an OLDER unmarked copy is not a supersession — the newest IS marked.
        # (Without this row a `superseded = newest if ANY row is unmarked`
        # variant survives, and every tree carrying one old copy would be
        # downgraded to `warn` with a false "NOT the newest build dir".)
        ([(a, "", None), (b, "", 1024)], b, b, None),
        # the newest copy is UNREADABLE, so it never reaches `readable` at all
        # — the verdict must still say it was passed over, or a tree whose
        # freshest artefact could not be read reports `ok … the newest build
        # dir` about one that is not.
        ([(a, "", 64)], c, a, c),
        # nothing states a size -> no comparison is possible
        ([(a, "", None), (b, "", None)], b, None, None),
        ([], None, None, None),
    ]
    for rows, newest, want_path, want_superseded in table:
        row, superseded = _select_generated_copy(rows, newest)
        picked = None if row is None else row[0]
        if picked != want_path or superseded != want_superseded:
            print(f"FAIL  [pod copy selection: {[(str(r[0]), r[2]) for r in rows]} "
                  f"newest={newest} -> picked {picked}, superseded "
                  f"{superseded}; want {want_path} / {want_superseded}]")
            errors += 1

    # ---- the ACTIVE target root, against a hand-written table -----------
    # The bug this table exists for: `Path(env)` for a RELATIVE
    # CARGO_TARGET_DIR resolves against the PARITY SCRIPT's cwd, while cargo
    # resolves it against the directory it is invoked in — the bench
    # workspace. A developer who always sets one would have had this arm
    # silently `skip` on a tree that was built, which is the exact failure
    # the arm was changed to stop doing.
    ws = _WORKSPACE_DIR
    default = ws / "target"
    root_table = [
        (None, default, None),
        ("", default, None),                                  # unset-ish
        ("target-cache", ws / "target-cache", default),       # RELATIVE -> bench ws
        ("../shared", ws / "../shared", default),             # relative, upward
        ("/tmp/cer-shared", Path("/tmp/cer-shared"), default),  # absolute
        (str(default), default, None),                        # the default, spelled out
        ("target", default, None),                            # …and spelled relatively
    ]
    for ctd_in, want_active, want_other in root_table:
        active, other = _pod_target_roots(ctd_in)
        if active != want_active or other != want_other:
            print(f"FAIL  [pod target root: CARGO_TARGET_DIR={ctd_in!r} -> "
                  f"active {active}, other {other}; want {want_active} / "
                  f"{want_other}]")
            errors += 1
    # The claim that makes the relative rows meaningful: a relative value
    # must NOT be read against this process's cwd, wherever that is.
    if _pod_target_roots("target-cache")[0] == Path("target-cache").resolve():
        print("FAIL  [pod target root: a relative CARGO_TARGET_DIR was "
              "resolved against the PARITY SCRIPT's cwd — cargo resolves it "
              "against the directory it is invoked in (the bench workspace)]")
        errors += 1

    # ---- the VERDICT never prints green over a failed run ---------------
    # The committed-size branch can record a control failure and fall
    # straight through to the verdict, where the hash comparison it reports
    # on is a DIFFERENT question — so `registered == generated` can hold
    # while the arm is failing. Reachable only with a built CLI, hence a
    # pure table: no other arm here can see it.
    verdict_table = [
        # (matches, prior errors, superseded, expected verdict)
        (True,  0, None,        "ok"),
        (True,  0, Path("b"),   "superseded"),
        (True,  1, None,        "held_but_failed"),
        (True,  1, Path("b"),   "held_but_failed"),   # a failure outranks a warn
        (False, 0, None,        "mismatch"),
        (False, 2, Path("b"),   "mismatch"),          # …and a mismatch outranks both
    ]
    for matches, prior, sup, want_v in verdict_table:
        got_v = _pod_hash_verdict(matches, prior, sup)
        if got_v != want_v:
            print(f"FAIL  [pod hash verdict: matches={matches} "
                  f"prior_errors={prior} superseded={sup} -> {got_v}; "
                  f"want {want_v}]")
            errors += 1
    if _pod_hash_verdict(True, 1, None) == "ok":
        print("FAIL  [pod hash verdict: a run that has ALREADY failed must "
              "never print a green `ok` for a later check that happened to "
              "hold]")
        errors += 1

    # ---- a passed-over copy is blamed for the RIGHT reason ---------------
    readable_paths = {Path("a"), Path("b")}
    for sup, want_reason in ((Path("b"), "unmarked"), (Path("c"), "unreadable")):
        got_reason = _superseded_reason(sup, readable_paths)
        if got_reason != want_reason:
            print(f"FAIL  [pod superseded reason: {sup} against "
                  f"{sorted(map(str, readable_paths))} -> {got_reason}; "
                  f"want {want_reason} — a copy that never reached `readable` "
                  f"could not be read, so nothing is known about its marker]")
            errors += 1

    # ---- the SELECTOR is WIRED with both of its arguments ---------------
    # The table above drives `_select_generated_copy` directly, so it cannot
    # see the CALL SITE. Dropping the second argument there takes the
    # unreadable half of the supersession rule with it, and on a host with no
    # built CLI the arm returns before the call, so even the TypeError the
    # required parameter now guarantees would not be reached. Parsed, not
    # grepped: a comment naming the function is not a call to it.
    tree = ast.parse(inspect.getsource(check_pod_schema_hash_matches_the_generated_type))
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name)
             and n.func.id == "_select_generated_copy"]
    if len(calls) != 1 or len(calls[0].args) != 2:
        got = [len(c.args) for c in calls]
        print(f"FAIL  [pod copy selection wiring: the schema-hash arm must "
              f"call `_select_generated_copy` EXACTLY once with BOTH "
              f"arguments (the readable rows AND the newest copy that "
              f"exists); found {len(calls)} call(s) with arg counts {got}]")
        errors += 1

    # ---- the MARKER is still written ------------------------------------
    # `_POD_BAKED_SIZE_LINE` is the ONLY statement of which size a generated
    # copy was baked at, and its sole producer is one `content.push_str` in
    # pod_codegen.rs. Reword or drop that comment and every copy becomes
    # unmarked, `_select_generated_copy` answers `None`, and the arm below
    # prints `skip` and returns 0 FOREVER — a gate disabled by an edit to a
    # comment. So the edit fails here instead.
    codegen = _WORKSPACE_DIR / "pod_schema" / "pod_codegen.rs"
    try:
        src = codegen.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [pod size marker: {codegen} unreadable ({e}) — the "
              f"marker the swept-tree comparison reads cannot be verified]")
        return errors + 1
    # The two HUMAN-READABLE fragments, never the Rust format string around
    # them: renaming the `{n}` binding, switching to positional `{}`, or
    # moving to `write!` all emit a byte-identical generated file, so pinning
    # the spelling would fail a change that broke nothing. What the reader
    # actually depends on is the rendered text.
    # Anchored on the OPENING QUOTE, so the marker cannot drift off column 0
    # (rustfmt's `format_strings` is off by default, so the fmt walk that
    # sweeps this bench workspace will never split the literal out from under
    # this check):
    # `_POD_BAKED_SIZE_LINE` is `^`-anchored, so a marker emitted mid-line (or
    # indented) would leave every copy unmarked while a bare substring check
    # stayed green — the very failure this gate exists to prevent.
    if ('"// Baked payload size: ' not in src) or (" bytes total" not in src):
        print(f"FAIL  [pod size marker: {codegen.name} no longer writes "
              f"`// Baked payload size: {{n}} bytes total` — that comment is "
              f"the only record of which sweep size a generated copy was "
              f"baked at, so dropping it turns the pod schema-hash arm into a "
              f"permanent `skip`. Keep the marker (and this gate) or teach "
              f"`_select_generated_copy` another way to read the size]")
        errors += 1
    else:
        # ANTI-TAUTOLOGY: the literal above must really be the thing the
        # READER matches, not a lookalike. Render the producer's line for a
        # sample size and require the reader's regex to parse it.
        rendered = "// Baked payload size: 1024 bytes total (CER_BENCH_POD_BYTES)."
        m = _POD_BAKED_SIZE_LINE.search(rendered)
        if m is None or int(m.group(1)) != 1024:
            print(f"FAIL  [pod size marker: the reader's regex does not parse "
                  f"the line the generator writes ({rendered!r})]")
            errors += 1

    if errors == 0:
        print("ok    [pod registration reconstruction moves only the array "
              "length, is a no-op at the committed size, refuses a size or a "
              "file it cannot re-declare; the copy selection prefers the "
              "newest MARKED copy and names a superseded one; and "
              "pod_codegen.rs still writes the size marker the selection "
              "reads]")
    return errors


def check_pod_schema_hash_matches_the_generated_type() -> int:
    """The REGISTERED descriptor and the type on the WIRE must hash the
    same, or a recording is not replay-grade.

    The recorder writes the producer's `SCHEMA_HASH` into the MCAP channel
    and the catalog resolves it from the workspace's registered schemas.
    Measured on a CLI built from this tree, with the earlier registration —
    the same four fields as a BARE workspace YAML — the two disagreed
    (`0xb981270238cd0358` registered vs `0xf99555aa42af17bc` generated),
    because `parse_rosmsg` folds the PACKAGE into the hash. Live delivery
    was unaffected (transport uses the cdylib's own metadata), which is
    exactly why nothing else caught it: both POD channels recorded as
    undescribable, could not render, and the bag was declared not
    replay-grade.

    Registering through the `.msg` store fixes it: the store takes its
    package from the DIRECTORY, so `cer_bench_msgs/PodPayload` hashes
    identically to the generated crate. This arm asserts that equality
    rather than trusting it — the generated constant is read from the
    crate's OUT_DIR, the registered one from `cerulion schema info`.

    WHICH generated copy, and at what size. The newest
    readable copy THAT STATES ITS BAKED SIZE — normally what this tree
    currently generates — compared at that size, read from the
    `Baked payload size:` line `pod_codegen.rs` writes into every generated
    file. A newer copy carrying no such line is fallen back past loudly, and
    the verdict is then a `warn` rather than an `ok`.

    Two earlier forms each gave up coverage somewhere, and this one is the
    fix for both. Taking the newest copy and demanding it equal the
    COMMITTED registration failed a healthy tree, because run_workspace.sh
    stopped keeping the registration in lock-step with the last
    baked size. Narrowing to a copy baked at the committed size instead
    stopped failing healthy trees but SKIPPED entirely on a swept one — and
    a sweep is the normal state of a bench machine, so in practice the arm
    stopped asking. It also PREFERRED a stale copy: `target/*/` spans debug
    and release, `CER_BENCH_POD_BYTES` defaults to the committed size, so a
    plain `cargo build` leaves a debug copy at the committed size forever
    while a sweep leaves release at the last swept one — and a January debug
    artifact was observed reporting `ok` while the tree's current generated
    type disagreed.

    So when the newest copy is baked at a size the committed file does not
    describe, the registration is RECONSTRUCTED at that size — this
    workspace's own `schemas/` tree with the pod `.msg`'s array length
    re-declared, in a temp workspace, no tracked file written (see
    `_reconstructed_pod_workspace`). The package still comes from the
    DIRECTORY, so the property the arm exists for — the store folds the
    package in, a bare workspace YAML does not — is asserted at every size
    the bench builds at.

    SCOPE of that: on a swept tree the SHIPPED `PodPayload.msg` is not
    resolved in situ by this arm at all, because the only size it could be
    compared at is one the sweep has left behind. What still covers the
    shipped file there is `check_pod_schema_is_never_rewritten`, which pins
    its committed array length against the first pinned sweep size. And on
    the one tree where both ARE available (a copy baked at the committed
    size), this arm runs BOTH — the shipped registration and the
    reconstruction — and requires them to agree, which is what keeps the
    swept-tree machinery itself checked rather than assumed.

    Needs a built CLI and a built `cer_bench_msgs`; skips loudly without
    either, since a stdlib harness can assume neither."""
    cli, why = _find_cerulion_cli()
    if cli is None:
        print(f"skip  [pod schema hash: {why}]")
        return 0
    committed_path = _SCHEMAS_DIR / "cer_bench_msgs" / "msg" / "PodPayload.msg"
    try:
        committed = committed_path.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [pod schema hash: {committed_path} unreadable ({e})]")
        return 1
    declared = _committed_pod_data_len(committed)
    if declared is None:
        print(f"FAIL  [pod schema hash: {committed_path} has no "
              f"'uint8[<N>] data' line — the committed size cannot be read, "
              f"so there is no generated copy to compare against]")
        return 1
    committed_size = declared + _POD_FIXED_BYTES
    # Note what cargo does NOT do: it keeps a build directory per
    # FINGERPRINT, not per env VALUE, and `CER_BENCH_POD_BYTES` is a
    # `rerun-if-env-changed`, so a sweep OVERWRITES `out/pod_payload.rs` in
    # place rather than accumulating one copy per size. (Copies under
    # `pod_*_node-*/out/` are older still — they predate the crate
    # extraction, when each node crate ran its own build script.)
    # The ACTIVE target root only. `benches/latency/workspace` is its own
    # cargo workspace and cargo honours `CARGO_TARGET_DIR` for it too, so an
    # ordinary developer setting moves every generated copy out of the
    # default glob — and the arm would then print "not built in this tree",
    # which is an affirmatively wrong cause for output that is right there.
    # See `_pod_target_roots` for why a relative value cannot be used as
    # written, and why the other root is reported rather than searched.
    # Through bench.py's ONE refusing accessor, not a fourth raw read of the
    # variable: this file is outside the AST walk's reach (it walks bench.py),
    # so a falsy `os.environ.get` here is the same class the walk exists to
    # close, one module over. main()'s preflight means the empty value never
    # reaches this arm on a real run.
    ctd = bench.cargo_target_dir_setting()
    active_root, other_root = _pod_target_roots(ctd)
    candidates = sorted(
        active_root.glob("*/build/cer_bench_msgs-*/out/pod_payload.rs"),
        key=lambda q: q.stat().st_mtime)
    readable = []
    for q in candidates:
        # An unreadable or non-UTF-8 candidate must not take the WHOLE parity
        # run down with a traceback — every other arm here prints and carries
        # on.
        try:
            text = q.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as e:
            print(f"warn  [pod schema hash: could not read {q} ({e}); "
                  f"skipping that copy]")
            continue
        m = _POD_BAKED_SIZE_LINE.search(text)
        readable.append((q, text, int(m.group(1)) if m else None))
    if not readable:
        if candidates:
            # Every candidate was unreadable (each warned above). Saying
            # "not built" here would be an affirmatively wrong cause.
            print(f"skip  [pod schema hash: {len(candidates)} generated "
                  f"pod_payload.rs candidate(s) exist but NONE could be read "
                  f"(see the warn line(s) above) — nothing to compare against]")
        else:
            # "you built somewhere else" and "you never built" are
            # different situations with different remedies, so the other
            # root is CHECKED before the cause is named — a bare "not
            # built" sent an operator to re-run a build whose output was
            # already on disk, under a CARGO_TARGET_DIR nothing looked at.
            elsewhere = (sorted(other_root.glob(
                "*/build/cer_bench_msgs-*/out/pod_payload.rs"))
                if other_root is not None else [])
            if elsewhere:
                print(f"skip  [pod schema hash: cer_bench_msgs is built, but "
                      f"under {other_root} — and CARGO_TARGET_DIR points "
                      f"cargo at {active_root}, which holds no generated "
                      f"pod_payload.rs. That copy is from a different "
                      f"configuration and is deliberately NOT compared. "
                      f"RESOLVABLE: rebuild with the CARGO_TARGET_DIR you "
                      f"are running this under, or unset it]")
            else:
                print(f"skip  [pod schema hash: cer_bench_msgs is not built "
                      f"in this tree (no generated pod_payload.rs under a "
                      f"cer_bench_msgs build dir in {active_root}) — run the "
                      f"workspace build first"
                      + ("" if ctd else "; CARGO_TARGET_DIR is unset, so that "
                                        "is cargo's default root")
                      + "]")
        return 0
    # The newest copy that states its size. An UNMARKED copy cannot be
    # compared against anything (nothing says which size it was baked at), so
    # falling back past one is the only way to stay covered — but it is said
    # out loud, and the verdict below is downgraded to `warn`, because the
    # comparison is then evidence about a copy that is not what this tree
    # currently generates.
    row, superseded = _select_generated_copy(
        readable, candidates[-1] if candidates else None)
    if row is None:
        print(f"skip  [pod schema hash: {len(readable)} generated "
              f"pod_payload.rs copy/copies are readable but NONE carries a "
              f"`// Baked payload size:` marker, so the size each was "
              f"generated at cannot be read and there is nothing to compare "
              f"them against. RESOLVABLE: `CER_BENCH_POD_BYTES="
              f"{committed_size} cargo build -p cer_bench_msgs` in the bench "
              f"workspace, then re-run]")
        return 0
    newest, text, baked = row
    if superseded is not None:
        why = ("carries no `// Baked payload size:` marker"
               if _superseded_reason(superseded, {r[0] for r in readable}) == "unmarked"
               else "could not be READ (see the warn line above) — so nothing "
                    "is known about its contents, marker included")
        print(f"warn  [pod schema hash: the newest generated pod_payload.rs "
              f"({superseded.parent.parent.name}) was passed over because it "
              f"{why}; comparing the newest MARKED copy "
              f"({newest.parent.parent.name}) instead, which is not what this "
              f"tree most recently generated]")
        provenance = (f"{newest.parent.parent.name}; NOT the newest build dir "
                      f"in this tree — that is "
                      f"{superseded.parent.parent.name}")
    else:
        provenance = f"{newest.parent.parent.name}, the newest build dir"
    m = re.search(r"PODPAYLOAD_SCHEMA_HASH: u64 = (0x[0-9A-Fa-f]+)", text)
    if m is None:
        print(f"FAIL  [pod schema hash: {newest} carries no "
              f"PODPAYLOAD_SCHEMA_HASH constant]")
        return 1
    want = int(m.group(1), 16)
    errors = 0
    # BOTH `schema info` runs get the kill-switch, not just the temp one: an
    # unresolvable name falls through to the remote tier, which spawns
    # `cerulion-netd` and waits on the convergence ceiling — and the branch
    # that would hit it is precisely the broken-registration case this arm
    # exists to report. A gate that can hang on discovery is not a gate, and a
    # hash answered by a ROBOT is not this workspace's registration.
    env = dict(os.environ, CERULION_NETWORK="off")

    def reconstructed(size):
        """(hash, detail) for the registration re-declared at `size`.

        TOTAL, like `_registered_pod_hash` itself and for the same reason: a
        raise here would take the whole parity run down with a traceback. The
        `ValueError` arm is reachable on an ordinary tree — the committed-size
        gate above reads the array length with `re.search` (first match) while
        the re-declare demands exactly one, so a registration carrying TWO
        `uint8[<N>] data` lines passes that gate and refuses here."""
        try:
            with _reconstructed_pod_workspace(committed_path, committed, size) as root:
                return _registered_pod_hash(cli, root, env=env)
        except ValueError as e:
            return None, (f"HARNESS: could not re-declare {committed_path.name} "
                          f"at {size} B ({e}) — either that size is nonsense or "
                          f"the registration is no longer the shape this gate "
                          f"understands")
        except OSError as e:
            return None, (f"HARNESS: could not build the temp workspace the "
                          f"comparison needs ({e})")

    if baked == committed_size:
        # The strongest form: the registration this workspace actually
        # ships, resolved where it actually lives.
        source = f"the shipped {committed_path.name}"
        got, detail = _registered_pod_hash(cli, _WORKSPACE_DIR, env=env)
        # …and, on the ONE path where both are available, the reconstruction
        # is run beside it and required to agree. Without this the machinery
        # the swept-tree branch depends on is exercised by nothing but a pure
        # string test — so a temp workspace that failed to resolve for any
        # reason would show up first as a permanent FAIL on every swept tree,
        # accusing the registration of a fault that is the harness's.
        control, control_detail = reconstructed(committed_size)
        if control is None:
            print(f"FAIL  [pod schema hash: the reconstruction control could "
                  f"not resolve {committed_path.name} at its OWN committed "
                  f"size — the swept-tree path below rests on this machinery, "
                  f"so it is broken there too\n{control_detail}]")
            errors += 1
        elif got is not None and control != got:
            print(f"FAIL  [pod schema hash: re-declaring "
                  f"{committed_path.name} at its own committed size changed "
                  f"the registered hash ({control:#x} vs {got:#x}) — the "
                  f"reconstruction is not a no-op, so every swept-tree "
                  f"comparison it drives is comparing the wrong thing]")
            errors += 1
    else:
        source = (f"{committed_path.name} re-declared at {baked} B "
                  f"(the committed file describes {committed_size} B; the "
                  f"committed array length itself stays pinned by "
                  f"check_pod_schema_is_never_rewritten)")
        got, detail = reconstructed(baked)
    if got is None:
        print(f"FAIL  [pod schema hash: `schema info cer_bench_msgs/PodPayload` "
              f"against {source} gave no hash line — the registration the "
              f"recorder needs does not resolve\n{detail}]")
        return errors + 1
    verdict = _pod_hash_verdict(got == want, errors, superseded)
    if verdict == "mismatch":
        print(f"FAIL  [pod schema hash @{baked}B: registered "
              f"{got:#x} != generated {want:#x} — the registration and the "
              f"generated type disagree at the SAME size, so this is not a "
              f"sweep artifact: most likely a package-folding regression (a "
              f"bare workspace YAML instead of the .msg store), otherwise a "
              f"HASH_RECIPE bump, an edit to the .msg field list, or a "
              f"pod_codegen.rs change to the fields around the baked array "
              f"(registration read from {source}; generated from "
              f"{provenance})]")
        errors += 1
    elif verdict == "held_but_failed":
        # The comparison held, but THIS RUN has already failed above, so no
        # green line is printed for it — `ok` two lines under a `FAIL` for
        # the same run is exactly what a skim reads past. The outcome is
        # still stated: it is information the FAIL above does not carry.
        print(f"warn  [pod schema hash @{baked}B: the comparison itself held "
              f"(registered == generated {got:#018x}), but this arm FAILED "
              f"for the reason above and its verdict does not stand]")
    elif verdict == "superseded":
        # The comparison HELD, but against a copy that is not what this tree
        # currently generates, so it is evidence about the past. Loud and
        # non-fatal — a clean `ok` here reads green on a skim.
        print(f"warn  [pod schema hash @{baked}B: registered == generated "
              f"({got:#018x}) — but from a SUPERSEDED copy ({provenance}). "
              f"The codegen side of this check is NOT covered on this tree; "
              f"rebuild cer_bench_msgs to make it current]")
    else:
        print(f"ok    [pod schema hash @{baked}B: registered == "
              f"generated ({got:#018x}, from {provenance}; registration "
              f"{source})]")
    return errors


def check_bench_graphs_validate() -> int:
    """The same question as `check_bench_graph_schemas_resolve`, asked of the
    REAL CLI: every graph in the bench workspace must pass `cerulion graph
    validate --release`.

    That is the command whose failure a code review reported, and it
    checks more than schema resolvability — the node crates, their cdylibs,
    the port shapes, the schema MATCH between a producer and its consumer,
    and the data-trigger bindings. It needs a built `cerulion` and built
    node cdylibs, neither of which this stdlib harness can assume, so it
    SKIPS loudly rather than failing on a host that has not built them."""
    cli, why = _find_cerulion_cli()
    if cli is None:
        print(f"skip  [bench graph validate: {why}; the stdlib "
              f"schema-resolution arm above still ran]")
        return 0
    errors = 0
    graphs = sorted(g.stem for g in _GRAPHS_DIR.glob("*.yaml"))
    if not graphs:
        print("FAIL  [bench graph validate: no graphs found]")
        return 1
    for g in graphs:
        r = subprocess.run([str(cli), "graph", "validate", g, "--release"],
                           cwd=str(_WORKSPACE_DIR), capture_output=True,
                           text=True, encoding="utf-8", errors="replace",
                           timeout=300)
        out = r.stdout + r.stderr
        if "cdylib for node" in out and "not found" in out:
            print(f"skip  [bench graph validate: {g} — a node cdylib is not "
                  f"built in this tree; run the workspace build first]")
            continue
        if r.returncode != 0:
            fails = [ln.strip() for ln in out.splitlines() if "[FAIL]" in ln]
            print(f"FAIL  [bench graph validate: {g}] rc={r.returncode}\n     "
                  + "\n     ".join(fails[:4]))
            errors += 1
        else:
            print(f"ok    [bench graph validate: {g}]")
    return errors


def check_class_carriers_refuse_at_the_mint() -> int:
    """The three places bench.py MINTS a class-carrying identity refuse a
    state their own docstrings forbid, instead of producing a name that
    lies about it.

      Ros2Cell     — `name` DROPS the class token on the stock/composed
                     lane branches, so a lane cell with msg="image" would
                     mint a name identical to its pod twin: two distinct
                     cells, one CSV stem, one baseline key.
      workspace_raw_name — an unknown class fell through to the incumbent
                     pinned prefix, i.e. onto the VARIABLE class's name,
                     which is what every downstream consumer classifies
                     on. (Ros2Cell.name has the OPPOSITE failure mode for
                     the same mistake — it appends the unknown value as a
                     token — so neither is left to inference.)
      SmokeCell.msg — the WORKSPACE class carrier, read only on the
                     workspace branch. A default of "variable" put a
                     second spelling of the class on a ros2 image cell,
                     disagreeing with ros2.msg, with this one ignored.

    Each refusal is paired with the control it must not break: today's
    real enumeration still builds."""
    errors = 0

    def refuses(desc, fn):
        nonlocal errors
        try:
            got = fn()
        except ValueError as e:
            print(f"ok    [class mint refuses: {desc}] {str(e)[:60]}...")
            return
        print(f"FAIL  [class mint: {desc}] expected a ValueError, got {got!r}")
        errors += 1

    def accepts(desc, fn):
        nonlocal errors
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            print(f"FAIL  [class mint control: {desc}] {type(e).__name__}: {e}")
            errors += 1
        else:
            print(f"ok    [class mint control: {desc}]")

    refuses("Ros2Cell: the stock lane with msg=image",
            lambda: bench.Ros2Cell("jazzy", "stock", "stock", "rclcpp",
                                   "stock", 0, msg="image").name)
    refuses("Ros2Cell: the composed lane with msg=image",
            lambda: bench.Ros2Cell("jazzy", "composed", "ipcon", "rclcpp",
                                   "stock", 0, msg="image").name)
    refuses("Ros2Cell: an unknown class",
            lambda: bench.Ros2Cell("jazzy", "cyclonedds", "shm", "rclcpp",
                                   "be1", 0, msg="bogus").name)
    refuses("workspace_raw_name: the ROS 2 spelling of the variable class",
            lambda: bench.workspace_raw_name("split", 0, "image"))
    refuses("workspace_raw_name: an unknown class",
            lambda: bench.workspace_raw_name("split", 0, "bogus"))
    refuses("SmokeCell: a ros2 cell carrying the workspace class field",
            lambda: bench.SmokeCell(kind="ros2", cell_name="c",
                                    payloads=(64,), msg="pod"))
    refuses("SmokeCell: a workspace cell with no class at all",
            lambda: bench.SmokeCell(kind="workspace", cell_name="c",
                                    payloads=(64,)))
    refuses("SmokeCell: a workspace cell with an unknown class",
            lambda: bench.SmokeCell(kind="workspace", cell_name="c",
                                    payloads=(64,), msg="image"))
    accepts("the real ROS 2 enumeration still builds",
            lambda: [bench.enumerate_ros2_cells(d, c)
                     for d in bench.ROS2_DISTROS for c in (0, 1)])
    accepts("the real workspace names still build",
            lambda: [bench.workspace_raw_name(leg, c, m)
                     for leg in bench.WORKSPACE_LEGS for c in (0, 1)
                     for m in bench.WORKSPACE_MSG_CLASSES])
    accepts("the real smoke set still builds",
            lambda: [bench.enumerate_smoke_cells(v)
                     for v in bench.PACING_VARIANTS])
    return errors


def check_smoke_gates_refuse_an_unreadable_log() -> int:
    """The three per-size smoke gates in ros2/run_bench.sh must not read
    "the log says nothing" as "nothing is wrong".

    `grep -q` returns 1 for no-match and 2 for cannot-read, and an `if`
    treats both as false. For `check_is_plain` that mattered most: it is
    the ONLY runtime check that the type-class LABEL matches what the
    binaries ran, and a refuse-the-wrong-marker test alone treats a log
    that says NOTHING as a log that says the right thing. So the
    assertion is POSITIVE: the class's own marker must be present. Every
    node logs it from its constructor, so a log without it LOST it (a
    truncated or rotated file, a dropped stderr redirect, a suppressed
    INFO level) or came from binaries older than `is_plain_check` — and
    the gate cannot tell which, which is the point. (A stale image is NOT
    the example: `is_plain_check` predates this axis, so such a binary
    logs `is_plain: 1`, which the NEGATIVE arm refuses on an image cell.)

    The gates are driven as the SHIPPED bash functions, extracted from
    run_bench.sh — no ROS, no docker, no container needed."""
    errors = 0
    try:
        fns = "\n\n".join(_extract_bash_function(_RUN_BENCH, n) for n in
                            ("check_is_plain", "check_dma_lock",
                             "check_structural_loan_skip"))
    except RuntimeError as e:
        print(f"FAIL  [smoke gates: {e}]")
        return 1

    def drive(fn: str, log_text, msg_class: str = "pod",
              run_rc: str = "0") -> tuple:
        """(rc, stderr). rc 0 == the gate passed the cell.

        `set -uo pipefail` and NO `set -e`, byte-for-byte the contract both
        shipped runners use — adding `set -e` would test a shell these
        gates never run under (and `check_dma_lock` legitimately leaves a
        `grep` status of 1 behind). The cost of no `set -e` is that a
        command which does not exist prints to stderr and falls through to
        `echo CLEAN`, i.e. rc 0 — so every arm expecting 0 would be
        satisfied by the gate NEVER RUNNING. `type -t` closes that: a name
        in `cases` that is not in the extracted set exits 90, which no arm
        expects."""
        with tempfile.TemporaryDirectory(prefix="gates_") as td:
            d = Path(td)
            log = d / "cell_64_node.log"
            if log_text is not None:
                log.write_text(log_text, encoding="utf-8")
            script = d / "drive.sh"
            script.write_text(
                "set -uo pipefail\n"
                f"CER_BENCH_MSG={msg_class}\n"
                f"{fns}\n"
                f"type -t {fn} >/dev/null 2>&1 || exit 90\n"
                f"{fn} \"$1\" \"$2\"\n"
                "echo CLEAN\n", encoding="utf-8")
            r = subprocess.run(["bash", str(script), str(log), run_rc],
                               capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=60)
            return (0 if "CLEAN" in r.stdout else r.returncode, r.stderr)
    # (desc, fn, log content or None for "no such file", class, want rc,
    #  a fragment the refusal MESSAGE must carry — "" for a pass).
    # The message matters: check_is_plain has TWO exit-11 branches and the
    # second one's wording ("UNVERIFIED, not verified", naming the stale
    # image) is the whole point of the positive assertion. An rc-only
    # oracle passes a variant that swaps or blanks them.
    cases = [
        ("harness self-check: an undefined gate is not a pass",
         "check_no_such_gate", "Msg::is_plain: 1\n", "pod", 90, ""),
        ("is_plain: pod cell, pod marker present", "check_is_plain",
         "Msg::is_plain: 1 (has_fixed_size=1, is_trivially_copyable=1)\n",
         "pod", 0, ""),
        ("is_plain: pod cell, non-plain marker", "check_is_plain",
         "Msg::is_plain: 0 (has_fixed_size=0)\n", "pod", 11,
         "on a pod-class cell"),
        ("is_plain: pod cell, MARKER ABSENT (a log that lost the line)",
         "check_is_plain",
         "latency_node ready (payload=64)\n", "pod", 11,
         "UNVERIFIED, not verified"),
        ("is_plain: pod cell, log unreadable", "check_is_plain", None,
         "pod", 11, "UNVERIFIED, not verified"),
        ("is_plain: image cell, non-plain marker present", "check_is_plain",
         "Msg::is_plain: 0 (has_fixed_size=0)\n", "image", 0, ""),
        ("is_plain: image cell, plain marker (label lies)", "check_is_plain",
         "Msg::is_plain: 1 (has_fixed_size=1)\n", "image", 11,
         "on a image-class cell"),
        ("is_plain: image cell, MARKER ABSENT (a log that lost the line)",
         "check_is_plain", "latency_node ready (payload=64)\n", "image", 11,
         "UNVERIFIED, not verified"),
        ("is_plain: image cell, log unreadable", "check_is_plain", None,
         "image", 11, "UNVERIFIED, not verified"),
        ("structural loan skip: sentinel present",
         "check_structural_loan_skip",
         "RMW_LOAN_RECV_UNSUPPORTED rmw=rmw_zenoh_cpp\n", "pod", 77, ""),
        ("structural loan skip: no sentinel", "check_structural_loan_skip",
         "all good\n", "pod", 0, ""),
        ("structural loan skip: log unreadable",
         "check_structural_loan_skip", None, "pod", 11,
         "loan-recv sentinel could not be checked"),
    ]
    # The run-gated half. `run_one` truncates the log BEFORE it can refuse
    # (its rc=2 is "chrt not permitted"), and the caller runs these gates
    # before consulting that rc on purpose — so an EMPTY log under a
    # non-zero run_rc must leave the run's own, more precise verdict
    # standing instead of exiting 11 with "class UNVERIFIED".
    run_gated = [
        ("is_plain: pod cell, empty log under a chrt refusal (rc=2)",
         "check_is_plain", "", "pod", "2", 0),
        ("is_plain: image cell, empty log under a chrt refusal (rc=2)",
         "check_is_plain", "", "image", "2", 0),
        ("is_plain: a NODE failure (rc=3) leaves its own verdict",
         "check_is_plain", "", "pod", "3", 0),
        ("structural loan skip: unreadable log under a chrt refusal",
         "check_structural_loan_skip", None, "pod", "2", 0),
        # The negative arm stays UNCONDITIONAL: a wrong marker is evidence
        # the binaries DID run and ran the wrong class, whatever run_one
        # went on to report.
        ("is_plain: a wrong marker still refuses under a non-zero run_rc",
         "check_is_plain", "Msg::is_plain: 0 (has_fixed_size=0)\n", "pod",
         "2", 11),
        ("structural loan skip: the sentinel still skips under rc=2",
         "check_structural_loan_skip",
         "RMW_LOAN_RECV_UNSUPPORTED rmw=rmw_zenoh_cpp\n", "pod", "2", 77),
    ]
    for desc, fn, text, cls, want, must in cases:
        got, err = drive(fn, text, cls)   # run_rc "0": the run succeeded
        if got != want or (must and must not in err):
            print(f"FAIL  [smoke gate: {desc}] want rc={want} + {must!r}, got "
                  f"rc={got}:\n{err.rstrip()[-400:]}")
            errors += 1
        else:
            print(f"ok    [smoke gate: {desc}] rc={got}")
    for desc, fn, text, cls, rrc, want in run_gated:
        got, err = drive(fn, text, cls, run_rc=rrc)
        if got != want:
            print(f"FAIL  [smoke gate: {desc}] want rc={want} at run_rc="
                  f"{rrc}, got rc={got}:\n{err.rstrip()[-300:]}")
            errors += 1
        else:
            print(f"ok    [smoke gate: {desc}] rc={got} at run_rc={rrc}")
    # check_dma_lock returns early unless /dev/cpu_dma_latency exists, so
    # on a host without it every arm would be vacuous. ONE declared
    # substitution — the device path becomes a temp file that DOES exist —
    # leaves the body under test (the grep and its rc classification,
    # which is the whole fix) shipped-verbatim.
    with tempfile.TemporaryDirectory(prefix="dma_") as dma_td:
        fake_dev = Path(dma_td) / "cpu_dma_latency"
        fake_dev.write_text("", encoding="utf-8")
        # Extracted ONCE, inside the try above, so a rename fails as one
        # attributable line rather than a traceback out of the gate.
        shipped_dma = _extract_bash_function(_RUN_BENCH, "check_dma_lock")
        dma_fn = shipped_dma.replace("/dev/cpu_dma_latency", str(fake_dev))
        if str(fake_dev) not in dma_fn:
            print("FAIL  [smoke gate: check_dma_lock — the device-path "
                  "substitution matched nothing; the arms below would be "
                  "vacuous]")
            errors += 1
        else:
            saved = fns
            fns = fns.replace(shipped_dma, dma_fn)
            # `str.replace` no-ops silently on a miss, and the guard above
            # only proves the INNER substitution matched inside dma_fn.
            if str(fake_dev) not in fns:
                print("FAIL  [smoke gate: check_dma_lock — the substituted "
                      "body did not reach the driver script; the three DMA "
                      "arms below would be vacuous]")
                errors += 1
            for desc, text, want, must in (
                    ("dma lock: failure line present",
                     "cpu_dma_lock: open failed\n", 12,
                     "present but the DMA lock failed"),
                    ("dma lock: log clean", "cpu_dma_lock: held\n", 0, ""),
                    ("dma lock: log unreadable", None, 12,
                     "posture for this size is UNKNOWN")):
                got, err = drive("check_dma_lock", text, "pod")
                if got != want or (must and must not in err):
                    print(f"FAIL  [smoke gate: {desc}] want rc={want} + "
                          f"{must!r}, got rc={got}:\n{err.rstrip()[-300:]}")
                    errors += 1
                else:
                    print(f"ok    [smoke gate: {desc}] rc={got}")
            fns = saved
    return errors


def check_raw_name_class_agreement() -> int:
    """Both runners refuse a raw prefix that carries the OTHER class's
    pinned grammar. The prefix — not CER_BENCH_MSG — is what compile_csv
    and plot.py classify on (`_pod` on the workspace stack, `_image` on
    ROS 2), so a hand-run that sets one and names the other publishes one
    class's numbers under the other's label, silently, in the figure.
    bench.py can never do it; a hand-run can, and the README documents
    hand-runs.

    Scoped to the PINNED grammars on purpose: an ad-hoc name (a probe, a
    one-off) claims no class and must still run — pinned here as a
    control, because a check that refused those would break every manual
    invocation. Cross-pinned against the REAL enumeration: every cell
    bench.py can mint must satisfy its own runner's rule, so the two
    spellings of the class cannot drift apart."""
    errors = 0
    ws_marker_var = "carries the pod class token"
    ws_marker_pod = "is not the pinned pod grammar"
    r2_marker_pod = "carries the image class token"
    r2_marker_img = "is a pinned POD cell name"
    with tempfile.TemporaryDirectory(prefix="nameclass_") as td:
        raw = Path(td)
        for desc, axes, name, marker in (
                ("workspace: pod class under a variable prefix",
                 {"CER_BENCH_MSG": "pod"},
                 "cerulion_workspace_split_chrt0", ws_marker_pod),
                ("workspace: variable class under a pod prefix",
                 {"CER_BENCH_MSG": "variable"},
                 "cerulion_workspace_mono_pod_chrt1", ws_marker_var),
                # `_pod` in the WRONG position: a bare substring accept
                # would let this through, and plot.py — which matches the
                # LEG token's `_pod` SUFFIX — classifies it as the
                # VARIABLE class, which is the leak this gate exists to
                # close, through the gate itself.
                ("workspace: _pod outside the leg token",
                 {"CER_BENCH_MSG": "pod"},
                 "cerulion_workspace_pod_split_chrt0", ws_marker_pod),
                ("workspace: _pod with no leg at all",
                 {"CER_BENCH_MSG": "pod"},
                 "cerulion_workspace_pod_chrt0", ws_marker_pod)):
            rc, err = _drive_run_workspace(raw, "split",
                                           dict(axes, CER_BENCH_RAW_NAME=name))
            if rc != 2 or marker not in err or _SMOKE_N_MARKER in err:
                print(f"FAIL  [raw-name class: {desc}] want rc=2 + {marker!r} "
                      f"before the SMOKE_N tripwire; got rc={rc}:\n"
                      f"{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [raw-name class refuses: {desc}] rc={rc}")
        for desc, axes, name in (
                ("workspace: matching pod pair", {"CER_BENCH_MSG": "pod"},
                 "cerulion_workspace_split_pod_chrt0"),
                ("workspace: matching variable pair",
                 {"CER_BENCH_MSG": "variable"},
                 "cerulion_workspace_split_chrt0"),
                ("workspace: an ad-hoc name claims no class",
                 {"CER_BENCH_MSG": "pod"}, "parity_probe")):
            rc, err = _drive_run_workspace(raw, "split",
                                           dict(axes, CER_BENCH_RAW_NAME=name))
            if (rc != 2 or _SMOKE_N_MARKER not in err
                    or ws_marker_var in err or ws_marker_pod in err):
                print(f"FAIL  [raw-name class control: {desc}] expected to "
                      f"pass the name check and stop at the SMOKE_N "
                      f"tripwire; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [raw-name class control passes: {desc}]")
        for desc, axes, name, marker in (
                ("ros2: pod class under an image cell name",
                 {"CER_BENCH_MSG": "pod"},
                 "jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0", r2_marker_pod),
                ("ros2: image class under a pinned pod cell name",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_cyclonedds_shm_rclcpp_be1_chrt0", r2_marker_img),
                ("ros2: image class under a no_shm pod cell name",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_cyclonedds_no_shm_rclcpp_be1_chrt0", r2_marker_img),
                ("ros2: image class under a zc pod cell name",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_fastdds_zc_rclcpp_be1_chrt0", r2_marker_img),
                ("ros2: image class under a stock lane name",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_stock_rclcpp_chrt0", r2_marker_img),
                ("ros2: image class under a composed lane name",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_composed_ipcon_rclcpp_chrt0", r2_marker_img)):
            rc, err = _drive_run_bench(raw, dict(axes,
                                                 CER_BENCH_RAW_NAME=name))
            if rc != 2 or marker not in err or _ROS_MARKER in err:
                print(f"FAIL  [raw-name class: {desc}] want rc=2 + {marker!r} "
                      f"before any ROS check; got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [raw-name class refuses: {desc}] rc={rc}")
        for desc, axes, name in (
                ("ros2: matching image pair",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0"),
                ("ros2: matching pod pair", {"CER_BENCH_MSG": "pod"},
                 "jazzy_cyclonedds_shm_rclcpp_be1_chrt0"),
                ("ros2: an ad-hoc name claims no class",
                 {"CER_BENCH_MSG": "image", "SHM_MODE": "shm"},
                 "parity_probe")):
            rc, err = _drive_run_bench(raw, dict(axes,
                                                 CER_BENCH_RAW_NAME=name))
            if (rc != 2 or _ROS_MARKER not in err
                    or r2_marker_pod in err or r2_marker_img in err):
                print(f"FAIL  [raw-name class control: {desc}] expected to "
                      f"pass the name check and stop at the ROS-setup check; "
                      f"got rc={rc}:\n{err.rstrip()}")
                errors += 1
            else:
                print(f"ok    [raw-name class control passes: {desc}]")
        # COVERAGE over the real enumeration: the arms above drive hand-picked
        # names, which says nothing about whether the refusal GRAMMAR reaches
        # every name bench.py can mint. A new pod lane whose stem carried
        # none of the four pinned tokens would escape the gate silently.
        # Deduped by the distinguishing part of the stem so the cost stays
        # a few seconds.
        seen: set = set()
        ros2_uncovered = []
        for distro in bench.ROS2_DISTROS:
            for chrt in (0, 1):
                for c in bench.enumerate_ros2_cells(distro, chrt)[0]:
                    if c.msg == "image":
                        continue
                    key = (c.rmw, c.shm, c.recv)
                    if key in seen:
                        continue
                    seen.add(key)
                    rc, err = _drive_run_bench(raw, {"CER_BENCH_MSG": "image",
                                                     "SHM_MODE": "shm",
                                                     "CER_BENCH_RAW_NAME": c.name})
                    if rc != 2 or r2_marker_img not in err:
                        ros2_uncovered.append((c.name, rc))
        if ros2_uncovered:
            print(f"FAIL  [raw-name class coverage: {len(ros2_uncovered)} "
                  f"enumerated ROS 2 pod name(s) are not reached by the image "
                  f"refusal grammar: {ros2_uncovered[:4]}]")
            errors += 1
        else:
            print(f"ok    [raw-name class coverage: all {len(seen)} distinct ROS 2 "
                  f"pod cell shapes are refused under CER_BENCH_MSG=image]")
        ws_uncovered = []
        for leg in bench.WORKSPACE_LEGS:
            for chrt in (0, 1):
                n = bench.workspace_raw_name(leg, chrt, "variable")
                rc, err = _drive_run_workspace(raw, "split",
                                               {"CER_BENCH_MSG": "pod",
                                                "CER_BENCH_RAW_NAME": n})
                if rc != 2 or ws_marker_pod not in err:
                    ws_uncovered.append((n, rc))
        if ws_uncovered:
            print(f"FAIL  [raw-name class coverage: workspace variable prefix(es) "
                  f"not reached by the pod refusal grammar: {ws_uncovered}]")
            errors += 1
        else:
            print("ok    [raw-name class coverage: every workspace variable "
                  "prefix is refused under CER_BENCH_MSG=pod]")
    # Cross-pin: the runner's rule and bench.py's naming are two
    # spellings of one fact. The ROS 2 half walks the REAL enumeration;
    # the workspace half walks bench.py's own leg and class tuples, which
    # is the whole space `workspace_raw_name` can mint.
    bad = []
    for distro in bench.ROS2_DISTROS:
        for chrt in (0, 1):
            for c in bench.enumerate_ros2_cells(distro, chrt)[0]:
                if ("_image_" in c.name) != (c.msg == "image"):
                    bad.append((c.name, c.msg))
    for leg in bench.WORKSPACE_LEGS:
        for chrt in (0, 1):
            for msg in bench.WORKSPACE_MSG_CLASSES:
                n = bench.workspace_raw_name(leg, chrt, msg)
                if ("_pod_" in n) != (msg == "pod"):
                    bad.append((n, msg))
    if bad:
        print(f"FAIL  [raw-name class vs enumeration: {len(bad)} name(s) "
              f"disagree with their own msg class: {bad[:6]}]")
        errors += 1
    else:
        print("ok    [raw-name class vs enumeration: every cell bench.py "
              "mints names its own class]")
    return errors


def check_type_class_legend() -> int:
    """On a figure carrying BOTH classes of the ROS 2 stack, the pod rows
    name their class in the LEGEND — not only in the matched-quantity
    footnote, which `--release` strips as an annotation. Before this, a
    release render labelled the image rows `sensor_msgs/Image (variable)`
    and left the pod rows saying nothing about their type, so the
    artifact that leaves the repo identified one class and not the other
    (the workspace stack never had this gap: both its legs carry a class
    token in every render).

    Scoped to the PAIRING on purpose, and that is the half worth pinning:
    an incumbent ROS 2 figure with no image row must stay byte-identical,
    so the token must be ABSENT there. A caller-supplied `--series`
    pretty label is the user's words and is never appended to.

    Helper half is stdlib; the release-render half needs matplotlib and
    skips loudly without it (same rule as the footnote arm)."""
    try:
        import plot
    except ImportError as e:
        print(f"skip  [type-class legend: plot.py unimportable: {e}]")
        return 0
    errors = 0
    S = _series_for_stem
    # A HAND literal, not `plot._ROS2_POD_CLASS_TOKEN`: this is the string
    # that leaves the repo in a --release artifact, and reading it out of
    # the module under test would assert only that the module agrees with
    # itself (measured: rewriting the constant to "WRONG TEXT ENTIRELY"
    # left every arm below green). The image side one line down has always
    # been a literal; the asymmetry was the tell.
    tok = "Pod<N> (fixed array)"
    if plot._ROS2_POD_CLASS_TOKEN != tok:
        print(f"FAIL  [type-class legend: the ROS 2 pod class token is "
              f"{plot._ROS2_POD_CLASS_TOKEN!r}, not {tok!r} — the release "
              f"artifact's wording changed]")
        errors += 1
    img_tok = "sensor_msgs/Image (variable)"
    img = S("jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0")
    pod = S("jazzy_cyclonedds_shm_rclcpp_be1_chrt0")
    zc_pod = S("jazzy_fastdds_zc_rclcpp_be1_chrt0")
    stock = S("jazzy_stock_rclcpp_chrt0")
    composed = S("jazzy_composed_ipcon_rclcpp_chrt0")
    ws_pod = S("cerulion_workspace_split_pod_chrt0")
    ws_var = S("cerulion_workspace_split_chrt0")
    iox2 = S("iox2_chrt0")
    pretty = plot.Series("jazzy_cyclonedds_shm_rclcpp_be1_chrt0",
                         "results_jazzy_cyclonedds_shm_rclcpp_be1_chrt0.csv",
                         "-", "k", pretty="my own words")
    # (series, paired_stacks, must-contain, must-NOT-contain)
    ros2_paired = {"ros2"}
    ws_only = {"workspace"}
    none_paired: set = set()
    cases = [
        ("ros2 pod row, ros2 stack paired", pod, ros2_paired, tok, ()),
        ("ros2 pod row on an incumbent figure", pod, none_paired, None,
         (tok,)),
        # THE scoping arm: after this axis landed the standard groups
        # always carry both WORKSPACE legs, so a figure-wide boolean would
        # have tagged every ROS 2 pod row on every incumbent figure — the
        # opposite of the "byte-identical" claim this scoping makes.
        ("ros2 pod row where only the WORKSPACE stack is paired", pod,
         ws_only, None, (tok,)),
        ("ros2 zc pod row, ros2 stack paired", zc_pod, ros2_paired, tok, ()),
        ("the stock lane (pod by enumeration)", stock, ros2_paired, tok, ()),
        ("the composed lane (pod by enumeration)", composed, ros2_paired,
         tok, ()),
        ("the image row keeps its own type name", img, ros2_paired, img_tok,
         (tok,)),
        ("the image row, unpaired", img, none_paired, img_tok, (tok,)),
        ("workspace variable leg (already self-describing)", ws_var,
         ros2_paired | ws_only, img_tok, (tok,)),
        ("workspace pod leg (already self-describing)", ws_pod,
         ros2_paired | ws_only, "fixed array (pod)", (tok,)),
        ("the native floor claims no type class", iox2, ros2_paired, None,
         (tok,)),
        ("a --series pretty label is the user's words", pretty, ros2_paired,
         "my own words", (tok,)),
    ]
    for desc, ser, pair, must, must_not in cases:
        text = plot._legend_label(ser, paired_stacks=pair)
        bad = [b for b in must_not if b in text]
        if (must is not None and must not in text) or bad:
            print(f"FAIL  [type-class legend: {desc}] want {must!r} present "
                  f"and {list(must_not)} absent; got {text!r}")
            errors += 1
        else:
            print(f"ok    [type-class legend: {desc}] -> {text!r}")
    # Every enumerated ROS 2 cell whose legend text is its RAW CSV STEM.
    # A release artifact has to name its rows, so this would be a defect —
    # except that the gap is `_ROS2_RE`'s missing `zc` vocabulary, which
    # PREDATES this axis and whose wording is a maintainer decision, deferred:
    # the fix is `(shm|no_shm|zc)` plus a
    # shm-mode label such as "SHM · DataSharing zero-copy". Pinned as a
    # DECLARED inventory rather than a FAIL, so the known gap cannot be
    # mistaken for intent and any NEW unlabelled cell fails here.
    declared_raw_stems = {
        f"{d}_fastdds_zc_{recv}_{qos}_chrt{c}"
        for d in ("jazzy", "lyrical") for c in (0, 1)
        for recv, qos in (("rclcpp", "be1"), ("rclcpp", "rel10"),
                          ("loan", "be1"))
    }
    raw_labelled = {c.name for d in bench.ROS2_DISTROS for ch in (0, 1)
                    for c in bench.enumerate_ros2_cells(d, ch)[0]
                    if plot.pretty_label(c.name) == c.name}
    if not raw_labelled:
        print("FAIL  [type-class legend: NO enumerated ROS 2 cell renders a "
              "raw stem — the zc legend-vocabulary gap this inventory "
              "declares is fixed, so delete the inventory]")
        errors += 1
    elif raw_labelled - declared_raw_stems:
        print(f"FAIL  [type-class legend: enumerated ROS 2 cell(s) render "
              f"their RAW CSV STEM as the legend label, outside the declared "
              f"zc gap — a release artifact must name its rows: "
              f"{sorted(raw_labelled - declared_raw_stems)[:4]}]")
        errors += 1
    else:
        print(f"ok    [type-class legend: the {len(raw_labelled)} raw-stem "
              f"labels are exactly the declared fastdds zc gap "
              f"(_ROS2_RE has no zc vocabulary; wording still to be settled)]")
    # An unpaired ros2 pod label must be byte-identical to pretty_label's
    # — the incumbent-figures-unchanged claim, asserted rather than implied.
    if (plot._legend_label(pod, paired_stacks=set())
            != plot.pretty_label(pod.label)):
        print("FAIL  [type-class legend: an unpaired pod label must be "
              "byte-identical to pretty_label's]")
        errors += 1
    else:
        print("ok    [type-class legend: unpaired pod label is byte-identical "
              "to pretty_label's]")
    # Through the real RELEASE render — the artifact the finding is about.
    try:
        import matplotlib  # noqa: F401
    except ImportError as e:
        print(f"skip  [type-class legend render: matplotlib absent on this "
              f"host — the release-render pin did not run: {e}]")
        return errors
    footnote_marker = "type classes: fixed array"
    with tempfile.TemporaryDirectory(prefix="legend_") as td:
        d = Path(td)
        for desc, series, want_tok in (
                ("paired release render", [img, pod], True),
                ("incumbent release render (no image row)", [pod, iox2],
                 False),
                # Both workspace legs beside ROS 2 pod rows and NO image
                # row: the workspace stack IS paired, so a figure-wide
                # boolean would add the ROS 2 token — it must not. (The
                # footnote does not appear either: `--release` strips it,
                # which is what the assertion below requires.)
                ("workspace-only pairing beside ros2 pod rows",
                 [ws_pod, ws_var, pod], False)):
            rd = d / desc.replace(" ", "_").replace("(", "").replace(")", "")
            rd.mkdir()
            _write_synthetic_results(rd, series)
            out = rd / "probe.svg"
            plot.plot_group(series, rd, out, title="parity probe",
                            variant="quiescent", release=True)
            # SVG text is XML-escaped, so `Pod<N>` renders as
            # `Pod&lt;N&gt;` — unescape before matching, or the arm would
            # read "absent" for a token that is right there.
            svg = html.unescape(out.read_text(encoding="utf-8"))
            got_tok = tok in svg
            # Presence anchor: measured, plot_group draws NO legend at all
            # when only one series survives, so "token absent" alone would
            # pass for a figure with no legend text whatsoever.
            anchor = plot.pretty_label(pod.label)
            if anchor not in svg:
                print(f"FAIL  [type-class legend render: {desc}] the pod "
                      f"row's own legend text is absent — the figure has no "
                      f"legend to carry a token, so this arm proves nothing")
                errors += 1
            elif got_tok != want_tok:
                print(f"FAIL  [type-class legend render: {desc}] pod class "
                      f"token {'present' if got_tok else 'absent'}, want "
                      f"{'present' if want_tok else 'absent'}")
                errors += 1
            elif want_tok and img_tok not in svg:
                print(f"FAIL  [type-class legend render: {desc}] the image "
                      f"row's own class token is missing — the release "
                      f"artifact must name BOTH classes")
                errors += 1
            elif footnote_marker in svg:
                # --release strips the footnote block, so the legend is
                # the only place a class can be named there. If the
                # footnote survived, a "token present" verdict would prove
                # nothing about the legend.
                print(f"FAIL  [type-class legend render: {desc}] the "
                      f"matched-quantity footnote survived --release; this "
                      f"arm proves nothing unless the legend is the only "
                      f"place the class is named")
                errors += 1
            else:
                print(f"ok    [type-class legend render: {desc}] pod token "
                      f"{'present' if got_tok else 'absent'}, footnote "
                      f"stripped by --release")
    return errors


def check_loan_lane_detector() -> int:
    """plot._is_loan_series recognizes EVERY loan-take lane bench.py can
    mint — the FastDDS DataSharing loan row (`*_zc_loan_*`) included —
    so the in-figure LOAN_CALLOUT rides
    every figure that carries such a lane. An earlier detector fix built a
    pod-stem matcher with zc, but the loan detector still read _ROS2_RE
    (shm|no_shm): a figure whose only loan lane was fastdds_zc_loan drew
    no callout. Cross-pinned against the REAL enumeration (recv == loan
    <=> detected, every distro x chrt; a vacuous run — no zc loan cell
    enumerated — fails), on a pretty-labelled overlay (the csv stem
    carries the identity), and through a real render pair. The callout
    is NOT greppable in the SVG (its withStroke path effect draws the
    text as glyph paths), so the render pin spies on Axes.annotate at
    the call site: LOAN_CALLOUT is annotated for a zc-loan-only figure
    and never for its rclcpp twin."""
    try:
        import plot
    except ImportError as e:
        print(f"skip  [loan lane detector: plot.py unimportable: {e}]")
        return 0
    errors = 0
    S = _series_for_stem
    wrong = []
    loan_shm_modes = set()
    for distro in bench.ROS2_DISTROS:
        for chrt in (0, 1):
            cells, _skips = bench.enumerate_ros2_cells(distro, chrt)
            for c in cells:
                if plot._is_loan_series(S(c.name)) != (c.recv == "loan"):
                    wrong.append((c.name, c.recv))
                if c.recv == "loan":
                    loan_shm_modes.add(c.shm)
    if wrong:
        print(f"FAIL  [loan lane detector vs enumeration: {len(wrong)} "
              f"cell(s) misread: {wrong[:6]}]")
        errors += 1
    elif "zc" not in loan_shm_modes:
        print("FAIL  [loan lane detector vs enumeration: no zc loan cell "
              "enumerated, so the zc arm proved nothing]")
        errors += 1
    else:
        print(f"ok    [loan lane detector agrees with the enumeration on "
              f"every ROS 2 cell (loan lanes over shm modes "
              f"{sorted(loan_shm_modes)})]")
    overlay = S("jazzy_fastdds_zc_loan_be1_chrt0", label="Loan lane (pretty)",
                csv="/elsewhere/results_jazzy_fastdds_zc_loan_be1_chrt0.csv")
    if not plot._is_loan_series(overlay):
        print("FAIL  [loan lane detector: pretty-labelled zc loan overlay "
              "not detected via its csv stem]")
        errors += 1
    else:
        print("ok    [loan lane detector: pretty-labelled zc loan overlay "
              "resolves via the csv stem]")
    try:
        import matplotlib  # noqa: F401 — plot_group imports it lazily
    except ImportError as e:
        print(f"skip  [loan lane render: matplotlib absent on this host — "
              f"the LOAN_CALLOUT render pin did not run: {e}]")
        return errors
    import matplotlib.axes
    annotated = []
    real_annotate = matplotlib.axes.Axes.annotate

    def spy_annotate(ax, text, *args, **kwargs):
        annotated.append(text)
        return real_annotate(ax, text, *args, **kwargs)

    with tempfile.TemporaryDirectory(prefix="loan_") as td:
        d = Path(td)
        for desc, stem, want in (
                ("zc loan lane alone", "jazzy_fastdds_zc_loan_be1_chrt0",
                 True),
                ("zc rclcpp twin (no loan lane)",
                 "jazzy_fastdds_zc_rclcpp_be1_chrt0", False)):
            rd = d / desc.replace(" ", "_"); rd.mkdir()
            series = [S(stem)]
            _write_synthetic_results(rd, series)
            annotated.clear()
            matplotlib.axes.Axes.annotate = spy_annotate
            try:
                plot.plot_group(series, rd, rd / "probe.svg",
                                title="parity probe", variant="quiescent")
            finally:
                matplotlib.axes.Axes.annotate = real_annotate
            got = plot.LOAN_CALLOUT in annotated
            if got != want:
                print(f"FAIL  [loan lane render: {desc}] LOAN_CALLOUT "
                      f"{'present' if got else 'absent'}, want "
                      f"{'present' if want else 'absent'}")
                errors += 1
            else:
                print(f"ok    [loan lane render: {desc}] LOAN_CALLOUT "
                      f"{'present' if got else 'absent'}")
    return errors


def check_custom_overlay_braces() -> int:
    """plot.py's --series (custom overlay) mode draws the beneath-axis
    rate braces ONLY under an explicit --variant: a manual overlay
    carries no suite pacing metadata (its paths may be absolute, from
    any run dir), so inferring the results dir's variant would make the
    figure claim a pacing it never established. Pinned by spying
    plot._draw_axis_braces through the REAL main(): no --variant => no
    brace call; --variant quiescent => braces drawn for 'quiescent'; and
    the control — an enumerated GROUP render without --variant still
    infers quiescent and draws braces (the fix is scoped to overlays)."""
    try:
        import plot
    except ImportError as e:
        print(f"skip  [custom overlay braces: plot.py unimportable: {e}]")
        return 0
    try:
        import matplotlib  # noqa: F401 — main() refuses without it
    except ImportError as e:
        print(f"skip  [custom overlay braces: matplotlib absent on this "
              f"host — the --series render pin did not run: {e}]")
        return 0
    errors = 0
    calls = []
    real_draw = plot._draw_axis_braces

    def spy_draw(ax, sizes, variant):
        calls.append(variant)
        return real_draw(ax, sizes, variant)

    S = _series_for_stem
    with tempfile.TemporaryDirectory(prefix="overlay_") as td:
        d = Path(td)
        overlay = S("iox2_chrt0")
        _write_synthetic_results(d, [overlay])
        csv = d / overlay.csv_name
        cases = [
            ("--series without --variant", ["--series", f"Overlay={csv}"],
             []),
            ("--series with --variant quiescent",
             ["--series", f"Overlay={csv}", "--variant", "quiescent"],
             ["quiescent"]),
            ("group render without --variant (control: still infers "
             "quiescent)", ["--group", "native", "--skip-missing"],
             ["quiescent"]),
        ]
        for desc, extra, want in cases:
            calls.clear()
            plot._draw_axis_braces = spy_draw
            try:
                rc = plot.main(["--results-dir", str(d), "--out-dir",
                                str(d / "plots")] + extra)
            finally:
                plot._draw_axis_braces = real_draw
            if rc != 0 or calls != want:
                print(f"FAIL  [custom overlay braces: {desc}] rc={rc}, brace "
                      f"calls={calls}, want {want}")
                errors += 1
            else:
                print(f"ok    [custom overlay braces: {desc}] brace calls="
                      f"{calls}")
    return errors


_PLOT_ARMS_WITHOUT_MATPLOTLIB = """
import sys
sys.modules["matplotlib"] = None   # makes `import matplotlib` raise ImportError
sys.path.insert(0, sys.argv[1])
import check_percentile_parity as c
rc = 0
for arm in (c.check_plot_variant_gate, c.check_type_class_visuals,
            c.check_type_class_footnote, c.check_loan_lane_detector,
            c.check_custom_overlay_braces, c.check_type_class_legend):
    rc += arm()
sys.exit(1 if rc else 0)
"""


def check_plot_arms_skip_without_matplotlib() -> int:
    """The advertised stdlib-only path: with matplotlib ABSENT every plot
    arm (SIX of them — the list above and `skip_markers` below must grow
    with each new render half, or the newest arm is exactly the one
    outside the net that exists for it) must still run its stdlib half
    and SKIP its render loudly — never
    abort the gate with a traceback. plot.py imports matplotlib lazily
    (inside plot_group), so `import plot` succeeds without it and an
    import-guard around `import plot` alone is inert: an earlier render arm
    crashed with ModuleNotFoundError at plot_group. Driven in a
    subprocess with matplotlib blocked via sys.modules, so the pin holds
    on hosts that DO have matplotlib (this one)."""
    errors = 0
    r = subprocess.run([sys.executable, "-c", _PLOT_ARMS_WITHOUT_MATPLOTLIB,
                        str(Path(__file__).resolve().parent)],
                       capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=120)
    skip_markers = ("skip  [type-class footnote render: matplotlib absent",
                    "skip  [loan lane render: matplotlib absent",
                    "skip  [custom overlay braces: matplotlib absent",
                    "skip  [type-class legend render: matplotlib absent")
    missing = [m for m in skip_markers if m not in r.stdout]
    if r.returncode != 0 or "Traceback" in r.stderr or missing:
        print(f"FAIL  [plot arms without matplotlib] rc={r.returncode}, "
              f"render-skip lines missing: {missing or 'none'}"
              f"\n--- stdout tail ---\n{r.stdout[-600:]}"
              f"\n--- stderr tail ---\n{r.stderr[-600:]}")
        errors += 1
    else:
        n_ok = sum(1 for ln in r.stdout.splitlines() if ln.startswith("ok"))
        print(f"ok    [plot arms without matplotlib: {n_ok} stdlib checks ran, "
              f"the render half skipped loudly, no traceback]")
    return errors


def check_smoke_type_class_coverage() -> int:
    """The smoke gate exercises BOTH type classes on each stack it
    covers (bench.enumerate_smoke_cells): a workspace pod representative
    beside the variable rows and a ROS 2 image representative beside
    the pod cells; every workspace cell's name derives from ITS class
    (workspace_raw_name(leg, chrt, msg) — a pod cell named without the
    token would gate the wrong row); and the class REACHES the runner:
    bench._run_smoke_cells is driven with run_workspace_leg replaced by
    a recorder, which must see each workspace cell's own msg (the
    mutant that drops the msg= pass-through records 'variable' for the
    pod cell and fails here)."""
    errors = 0
    cells = bench.enumerate_smoke_cells("quiescent")
    ws = [c for c in cells if c.kind == "workspace"]
    ws_classes = {c.msg for c in ws}
    if not {"variable", "pod"} <= ws_classes:
        print(f"FAIL  [smoke type-class coverage: workspace cells cover "
              f"{sorted(ws_classes)}, want both variable and pod]")
        errors += 1
    if not any(c.msg == "pod" and c.leg == "split"
               and set(c.payloads) == {bench.SMOKE_PAYLOAD_SMALL,
                                       bench.SMOKE_PAYLOAD_LARGE}
               for c in ws):
        print("FAIL  [smoke type-class coverage: no split pod cell gated at "
              "both payloads]")
        errors += 1
    for c in ws:
        want = bench.workspace_raw_name(c.leg, c.chrt, c.msg)
        if c.cell_name != want:
            print(f"FAIL  [smoke type-class coverage: {c.cell_name!r} "
                  f"should be {want!r} for msg={c.msg}]")
            errors += 1
    ros2_classes = {c.ros2.msg for c in cells if c.kind == "ros2"}
    if not {"pod", "image"} <= ros2_classes:
        print(f"FAIL  [smoke type-class coverage: ROS 2 cells cover "
              f"{sorted(ros2_classes)}, want both pod and image]")
        errors += 1
    if not errors:
        print(f"ok    [smoke type-class coverage: workspace "
              f"{sorted(ws_classes)}, ROS 2 {sorted(ros2_classes)}; "
              f"names derive from each cell's class]")

    seen = []

    def recording_leg(leg, chrt, variant, raw_dir, log_dir, sizes,
                      msg="variable"):
        seen.append((leg, chrt, msg, tuple(sizes)))
        return "ok"

    saved_leg = bench.run_workspace_leg
    saved_env = {k: os.environ.get(k) for k in
                 ("CER_BENCH_SMOKE_N", "CER_BENCH_TARGET_SAMPLES",
                  "CER_BENCH_WARMUP")}
    try:
        for k in saved_env:
            os.environ.pop(k, None)
        bench.run_workspace_leg = recording_leg
        with tempfile.TemporaryDirectory(prefix="smokemsg_") as td:
            bench._run_smoke_cells("quiescent", ws, Path(td))
    finally:
        bench.run_workspace_leg = saved_leg
        for k, v in saved_env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    want = [(c.leg, c.chrt, c.msg, tuple(c.payloads)) for c in ws]
    if seen != want:
        print(f"FAIL  [smoke type-class propagation: run_workspace_leg saw "
              f"{seen}, want {want}]")
        errors += 1
    else:
        print(f"ok    [smoke type-class propagation: run_workspace_leg "
              f"receives each workspace cell's own class: "
              f"{[m for _l, _c, m, _s in seen]}]")
    return errors


# ---------------------------------------------------------------------------
# The image class's builtin_interfaces/Time stamp codec
# ---------------------------------------------------------------------------

_ROS2_BENCH_SRC = Path(__file__).resolve().parent / "ros2" / "ros2_rtt_bench"
_STAMP_ORACLE = _ROS2_BENCH_SRC / "test" / "stamp_codec_oracle.cpp"
_STAMP_CODEC = _ROS2_BENCH_SRC / "src" / "stamp_codec.hpp"
# A floor, not the exact count: the oracle should be free to grow. It
# exists because an EMPTY translation unit compiles clean, so without it a
# truncated oracle reports green having asserted nothing.
_STAMP_ORACLE_MIN_ASSERTS = 25
# Set by check_image_stamp_codec when the compiled oracle actually RAN, so
# the PASS summary can claim what the run observed and nothing more. The
# summary's own comment says a name there "is the only signal that a check
# ran"; asserting the codec agrees with its oracles on a host that never
# compiled them would make that signal fire having established nothing.
_STAMP_ORACLE_RAN = False
_MSG_CLASS_DISPATCH = _ROS2_BENCH_SRC / "src" / "msg_class_dispatch.hpp"


def _cxx_code_only(text: str) -> str:
    """`text` with C++ comments blanked out, string literals kept.

    A text arm that searches raw source can be satisfied — or fooled — by
    a comment that merely NAMES the token it is looking for. That is not
    hypothetical here: the very comment explaining why `write_stamp` no
    longer narrows spells `static_cast<int32_t>(t / 1e9)`, and the
    comment explaining the SHM sweep spells `cleanup_iceoryx()`. Blanking
    comments (rather than deleting them) keeps every offset intact, so a
    caller can still slice the ORIGINAL text by an index found here.

    C++ block comments do not nest, so the state machine is a small one.
    Double-quoted strings ARE modelled — a `//` inside one would
    otherwise blank the rest of a real line.

    CHAR literals are deliberately NOT modelled, and that is a fix rather
    than a shortcut. C++14 digit separators are this tree's house style
    (`1'000'000'000ULL` appears in six sibling files), and an apostrophe
    branch reads the first separator as an opening quote and then skips
    forward to the next one — leaving every comment in between
    un-blanked, which is exactly the hole this helper exists to close.
    Pinned by `check_cxx_code_only`, whose digit-separator row fails on
    the apostrophe-aware version."""
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
            continue
        if c == "/" and i + 1 < n and text[i + 1] == "*":
            # CONSUME the opener first. Scanning for `*/` from the opening
            # `/` lets the opener's own `*` and a body starting with `/`
            # form a closer, so `/*/ ... */` read as a complete, empty
            # comment and the rest leaked through as code — fail-open, in
            # the one helper whose entire job is to stop a comment from
            # satisfying a text arm.
            out[i] = out[i + 1] = " "
            i += 2
            while i < n and not (text[i] == "*" and i + 1 < n and
                                 text[i + 1] == "/"):
                if text[i] != "\n":
                    out[i] = " "
                i += 1
            for k in range(i, min(i + 2, n)):
                out[k] = " "
            i += 2
            continue
        i += 1
    return "".join(out)


def check_cxx_code_only() -> int:
    """`_cxx_code_only` is the eyes of every C++ text arm below, so it gets
    its own hand oracles — the same treatment cerulion_core gives its
    `code_only` twin.

    The digit-separator row is the one that matters: an apostrophe-aware
    stripper reads `1'000` as an opening char literal and skips to the
    next apostrophe, leaving the comments in between visible — and a
    comment naming `encode_ros_stamp(` then satisfies the call-site arm
    while the shipping code narrows. Six sibling files in this tree carry
    `1'000'000'000ULL`, so restoring the house style must not re-open it."""
    errors = 0
    cases = [
        ("a line comment is blanked", "int a; // encode_ros_stamp(\nint b;",
         "encode_ros_stamp(", False),
        ("a block comment is blanked",
         "int a; /* cleanup_iceoryx() */ int b;", "cleanup_iceoryx()", False),
        ("a multi-line block comment is blanked",
         "int a;\n/* one\n   encode_ros_stamp(\n   three */\nint b;",
         "encode_ros_stamp(", False),
        ("a // inside a string does not blank the rest of the line",
         'f("http://x"); int keepme;', "keepme", True),
        ("code survives", "int keepme = encode_ros_stamp(t);",
         "encode_ros_stamp(", True),
        # A body that STARTS with `/`. Scanning for the closer from the
        # opening `/` let the opener's `*` pair with it, so this read as a
        # complete empty comment and the rest leaked through as code.
        ("a block comment whose body starts with /",
         "/*/ encode_ros_stamp( */ int keepme;", "encode_ros_stamp(", False),
        ("...and the code after it survives",
         "/*/ x */ int keepme;", "keepme", True),
        ("a block comment ending in **/",
         "/* encode_ros_stamp( **/ int keepme;", "encode_ros_stamp(", False),
        # THE regression: the house style must not blind the stripper.
        ("a digit separator does not open a char literal",
         "const uint64_t k = 1'000'000'000ULL;\n// encode_ros_stamp( here\n",
         "encode_ros_stamp(", False),
        ("...and the code around it survives",
         "const uint64_t k = 1'000'000'000ULL;\n// x\nint keepme;",
         "keepme", True),
    ]
    for what, src, needle, want_visible in cases:
        seen = needle in _cxx_code_only(src)
        if seen != want_visible:
            print(f"FAIL  [cxx code_only: {what} — {needle!r} "
                  f"{'survived' if seen else 'was blanked'}, want the "
                  f"{'opposite' if want_visible != seen else 'same'}]")
            errors += 1
    # Offsets must be preserved: callers slice the ORIGINAL text by an
    # index found in this view.
    src = "int a; /* xx */ int b; // yy\n"
    if len(_cxx_code_only(src)) != len(src):
        print("FAIL  [cxx code_only: the view must be the same length as "
              "the source — callers index the original by it]")
        errors += 1
    # The brace matcher that reads this view must not be unbalanced by a
    # brace inside a string literal.
    body = _brace_block_at('void f() { g("a { b"); int keepme; }', 0,
                           "cxx code_only self-check")
    if "keepme" not in body or not body.endswith("}"):
        print(f"FAIL  [cxx code_only: the brace matcher must skip braces "
              f"inside string literals, got {body!r}]")
        errors += 1
    # An unterminated block comment must fail CLOSED (blank to the end),
    # never leave the tail visible.
    if "encode_ros_stamp(" in _cxx_code_only("int a; /* encode_ros_stamp("):
        print("FAIL  [cxx code_only: an unterminated block comment must "
              "blank to the end of file, not leave its tail visible]")
        errors += 1
    # The static_assert wording predicate, both spellings and the
    # non-assertion control. Clang <= 16 says "static_assert failed" while
    # Clang 17+ and GCC say "static assertion failed", and the linkage
    # control recognised only the latter — a FALSE self-test failure on an
    # ordinary clang. A bare `error:` must still NOT satisfy it, or a
    # perturbation that merely orphans a local (a -Werror,-Wunused break)
    # would let a vacuous oracle pass the control written to catch it.
    for line, want in (
        ("x.cpp:9:1: error: static assertion failed due to requirement", True),
        ('x.cpp:9:1: error: static_assert failed "probe"', True),
        ("x.hpp:77:18: error: unused variable [-Werror,-Wunused-variable]",
         False),
        ("x.cpp:1:1: note: in instantiation of", False),
    ):
        if _is_assertion_failure(line) != want:
            print(f"FAIL  [cxx code_only: the static_assert wording "
                  f"predicate answered {not want} for {line!r}]")
            errors += 1
    # `_extract_cxx_block`'s AMBIGUITY guard. Every C++
    # call-site arm in this file names a function by a text opener and then
    # asserts over the body that follows it. If two declarations share that
    # opener the extractor has no way to know which one the arm meant, so
    # it refuses rather than silently taking the FIRST — which would let an
    # overload, or a copy inside an `#ifdef`, satisfy an assertion about
    # the shipping one. Driven here because nothing else drove it: the
    # guard shipped undriven, and an undriven refusal is one `git revert`
    # away from being a silent first-match.
    dup = ("void write_stamp(Image& m) { a(); }\n"
           "void write_stamp(Pod& m) { b(); }\n")
    try:
        got = _extract_cxx_block(dup, "void write_stamp(", "dup probe")
    except RuntimeError as e:
        if "more than once" not in str(e):
            print(f"FAIL  [cxx block extractor: an ambiguous opener must "
                  f"say so; the refusal reads {str(e)!r}]")
            errors += 1
    else:
        print(f"FAIL  [cxx block extractor: two declarations share the "
              f"opener and it returned the first one ({got!r}) — an arm "
              f"asserting over the shipping overload would be satisfied by "
              f"its neighbour]")
        errors += 1
    # The control: ONE occurrence still extracts, so the guard refuses
    # ambiguity rather than refusing everything.
    one = "void write_stamp(Image& m) { keepme(); }\n"
    try:
        body = _extract_cxx_block(one, "void write_stamp(", "dup control")
    except RuntimeError as e:
        # Bare, this would kill the whole run where every neighbouring arm
        # prints a FAIL line.
        print(f"FAIL  [cxx block extractor: a UNIQUE opener was refused "
              f"({e}) — the ambiguity guard must not refuse every call "
              f"site]")
        errors += 1
    else:
        if "keepme" not in body:
            print(f"FAIL  [cxx block extractor: a UNIQUE opener must still "
                  f"extract its body, got {body!r}]")
            errors += 1
    if errors == 0:
        print(f"ok    [cxx code_only: {len(cases)} hand oracles agree, "
              f"offsets preserved, an unterminated block fails closed, a "
              f"C++14 digit separator does not blind it, and an ambiguous "
              f"block opener is refused while a unique one still extracts]")
    return errors


def _calls_in(func, name: str) -> list:
    """Line numbers (1-based, within the function's own source) of every
    call to `name` in `func`'s body.

    Parsed, not grepped: a comment or a docstring naming a function is not
    a call to it, and every ordering claim below would otherwise be
    satisfied by prose. This arm's first draft WAS satisfied by prose —
    it read `run_workspace_leg`'s comment about `cleanup_iceoryx()` as the
    call itself and reported an ordering failure that did not exist."""
    tree = ast.parse(inspect.getsource(func))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            target = node.func
            fname = (target.id if isinstance(target, ast.Name)
                     else target.attr if isinstance(target, ast.Attribute)
                     else None)
            if fname == name:
                out.append(node.lineno)
    return sorted(out)


def _brace_block_at(text: str, at: int, where: str) -> str:
    """The balanced `{...}` block starting at the first `{` on or after
    `at`. Brace-counted, because C++ bodies hold braces of their own and a
    lazy regex would stop at the first one."""
    j = text.find("{", at)
    if j < 0:
        raise RuntimeError(f"{where}: no braced block at/after offset {at}")
    depth = 0
    k = j
    n = len(text)
    while k < n:
        c = text[k]
        # A brace inside a STRING is not a brace. Callers pass a
        # comment-blanked view, which keeps literals — and a printf format
        # holding `{` would otherwise unbalance the count and hand back a
        # truncated body that every search then reads as "absent".
        if c == '"':
            k += 1
            while k < n:
                if text[k] == "\\":
                    k += 2
                    continue
                if text[k] == '"':
                    break
                k += 1
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[j:k + 1]
        k += 1
    raise RuntimeError(f"{where}: unterminated block at offset {j}")


def _paren_block_at(text: str, at: int, where: str) -> str:
    """The balanced `(...)` starting at the first `(` on or after `at`.

    The brace counter's sibling, and string-aware for the same reason: a
    printf format string holds parentheses of its own, and counting them
    would hand back a truncated call that every later search reads as
    "absent"."""
    j = text.find("(", at)
    if j < 0:
        raise RuntimeError(f"{where}: no parenthesised call at/after {at}")
    depth = 0
    k = j
    n = len(text)
    while k < n:
        c = text[k]
        if c == '"':
            k += 1
            while k < n:
                if text[k] == "\\":
                    k += 2
                    continue
                if text[k] == '"':
                    break
                k += 1
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[j:k + 1]
        k += 1
    raise RuntimeError(f"{where}: unbalanced parentheses from offset {j}")


def _extract_cxx_block(text: str, opener: str, where: str) -> str:
    """The braced body that follows `opener`, brace-counted.

    Deliberately not a regex: `write_stamp`'s body contains braces of its
    own, and a lazy match would stop at the first one and read as if the
    call it is looking for were absent."""
    i = text.find(opener)
    if i < 0:
        raise RuntimeError(f"{where}: {opener!r} is not in the file")
    if text.find(opener, i + 1) >= 0:
        raise RuntimeError(f"{where}: {opener!r} appears more than once")
    return _brace_block_at(text, i, where)


# Both spellings a compiler may use for a failed `static_assert`. Clang 17+
# and GCC say "static assertion failed"; Clang <= 16 (clang-14 on Ubuntu
# 22.04 / Debian 12, Apple clang through Xcode 15) says "static_assert
# failed". The compiler search below prefers clang, so the older spelling
# is an ordinary configuration, and a control that recognised only the
# newer one reported a FALSE self-test failure on it — the perturbed build
# really did fail, and the arm said the oracle had not noticed.
_ASSERT_FAILED_SPELLINGS = ("static assertion failed", "static_assert failed")


def _is_assertion_failure(line: str) -> bool:
    """Did THIS diagnostic line report a failed static_assert?

    Deliberately NOT satisfied by a bare `error:`. The linkage control
    below needs to know the oracle's ASSERTIONS fired, not merely that a
    build failed: a perturbation that orphans a local fails on
    `-Werror,-Wunused-variable` just as loudly, and accepting that would
    let a vacuous oracle pass the very control written to catch it."""
    return any(sp in line for sp in _ASSERT_FAILED_SPELLINGS)


def check_image_stamp_codec() -> int:
    """The image class stamps `header.stamp`, whose `sec` is an int32 — and
    the encoder narrowed into it with nothing checking the range.

    A host past INT32_MAX seconds of monotonic uptime (2147483647 s, ~68
    years) made that an out-of-range conversion; the reader sign-extended
    whatever came back and the cell reported a number that is not a
    latency, under a label saying it is one. Theoretical on today's hosts
    and pinned here for exactly that reason: the failure is
    SILENT, and a bench whose product is a number must refuse rather than
    report.

    Two claims, because fixing one without the other fixes nothing:

    (a) the ARITHMETIC — driven by a compiled hand oracle
        (ros2/ros2_rtt_bench/test/stamp_codec_oracle.cpp) over the shipping
        stamp_codec.hpp. Every function there is `constexpr`, so the
        oracle is a wall of `static_assert`s and the COMPILER is the
        runner: no ROS distro needed, no binary to execute, and constant
        evaluation refuses undefined behaviour outright. A failure names
        the assertion's own message.

    (b) the CALL SITE — that MsgAdapter<sensor_msgs::msg::Image> actually
        routes both halves through that codec, and that the class dispatch
        asks the start-up question before it runs anything. A codec
        nothing calls is a codec that passes its own oracle while the
        shipping path still narrows; this is the same "assert on the
        shipped text of the one function that owns it" the pod-schema call
        site arm above uses, and for the same reason — the real path needs
        a container to reach."""
    errors = 0

    # (b) first: it needs no compiler, so a host without one still gets it.
    try:
        text = _cxx_code_only(_MSG_CLASS_DISPATCH.read_text(encoding="utf-8"))
    except OSError as e:
        print(f"FAIL  [image stamp call site: cannot read "
              f"{_MSG_CLASS_DISPATCH.name}: {e}]")
        return 1
    try:
        write_body = _extract_cxx_block(
            text, "static void write_stamp(sensor_msgs::msg::Image & m, "
                  "uint64_t t)", "image stamp call site")
        read_body = _extract_cxx_block(
            text, "static uint64_t read_stamp(const sensor_msgs::msg::Image "
                  "& m)", "image stamp call site")
    except RuntimeError as e:
        print(f"FAIL  [image stamp call site: {e}]")
        return 1

    # A token CENSUS is not a behavioural claim: three separate variants
    # pass it untouched — `(void)
    # encode_ros_stamp(t);` beside an implicit `m.header.stamp.sec = t /
    # kNs` (calls the codec AND narrows), `(void)
    # image_stamp_range_deliverable();` (asks and discards the verdict),
    # and the guard hoisted onto a branch that never runs. So each arm
    # below pins the SHAPE — where the value comes from, and what the
    # refusal does with it.
    for field in ("sec", "nanosec"):
        if not re.search(r"m\.header\.stamp\." + field +
                         r"\s*=\s*s\." + field + r"\s*;", write_body):
            print(f"FAIL  [image stamp call site: write_stamp does not "
                  f"assign header.stamp.{field} FROM the codec's result — "
                  f"calling the codec and then narrowing separately passes "
                  f"a mere presence check while the bug is still there]")
            errors += 1
    if not re.search(r"=\s*encode_ros_stamp\s*\(\s*t\s*\)", write_body):
        print("FAIL  [image stamp call site: write_stamp never binds "
              "encode_ros_stamp(t)'s RESULT — a discarded call is not a "
              "use]")
        errors += 1
    else:
        print("ok    [image stamp call site: write_stamp assigns both "
              "header.stamp fields from encode_ros_stamp(t)'s result]")
    if not re.search(r"return\s+decode_ros_stamp\s*\(", read_body):
        print("FAIL  [image stamp call site: read_stamp does not RETURN "
              "decode_ros_stamp(...) — the reconstruction is somewhere "
              "else]")
        errors += 1
    else:
        missing = [f for f in ("sec", "nanosec")
                   if f"m.header.stamp.{f}" not in read_body]
        if missing:
            print(f"FAIL  [image stamp call site: read_stamp does not read "
                  f"header.stamp.{'/'.join(missing)}]")
            errors += 1
        else:
            print("ok    [image stamp call site: read_stamp returns "
                  "decode_ros_stamp over both header.stamp fields]")
    # ONE assignment per field. Banning `static_cast<int32_t>` catches the
    # C++ spelling of the bypass; appending a C-style
    # `m.header.stamp.sec = (int32_t)(t / kNs);` AFTER the codec's
    # assignment does not use it, and the shape regexes above match the
    # FIRST assignment and are satisfied.
    for field in ("sec", "nanosec"):
        n_assign = len(re.findall(
            r"m\.header\.stamp\." + field + r"\s*=", write_body))
        if n_assign != 1:
            print(f"FAIL  [image stamp call site: write_stamp assigns "
                  f"header.stamp.{field} {n_assign} times — a second "
                  f"assignment after the codec's is a bypass whichever way "
                  f"it is spelled]")
            errors += 1
    for what, body in (("write_stamp", write_body), ("read_stamp", read_body)):
        if "static_cast<int32_t>" in body:
            print(f"FAIL  [image stamp call site: {what} still narrows into "
                  f"int32 itself — the codec's range handling is bypassed]")
            errors += 1

    # The REFUSAL, scoped to the branch that actually runs an image cell.
    # A guard sitting above that branch — or on a branch nothing takes —
    # satisfies a plain "appears before" ordering test.
    macro_at = text.find("#define ROS2_RTT_BENCH_DISPATCH_BY_CLASS")
    if macro_at < 0:
        print("FAIL  [image stamp refusal: ROS2_RTT_BENCH_DISPATCH_BY_CLASS "
              "is not in the file — has the dispatch been renamed?]")
        return errors + 1
    # Line continuations first: the macro is one logical statement spelled
    # across many physical lines, and `\s` does not match the backslash.
    macro = text[macro_at:].replace("\\\n", "\n")
    branch_at = macro.find('bench_msg_class() == "image"')
    if branch_at < 0:
        print("FAIL  [image stamp refusal: the dispatch has no image "
              "branch]")
        return errors + 1
    try:
        branch = _brace_block_at(macro, branch_at, "image stamp refusal")
    except RuntimeError as e:
        print(f"FAIL  [image stamp refusal: {e}]")
        return errors + 1
    guard = re.search(
        r"if\s*\(\s*!\s*ros2_rtt_bench::image_stamp_range_deliverable\s*\("
        r"\s*\)\s*\)\s*\{[^{}]*return\s+2\s*;", branch)
    runner = branch.find("RUNNER<sensor_msgs::msg::Image>()")
    if guard is None:
        print("FAIL  [image stamp refusal: the image branch does not REFUSE "
              "on image_stamp_range_deliverable() — the verdict must gate a "
              "`return 2`, not merely be called (a discarded call leaves an "
              "unmeasurable host running the sweep)]")
        errors += 1
    elif runner < 0:
        print("FAIL  [image stamp refusal: the image branch does not run the "
              "image instantiation]")
        errors += 1
    elif guard.start() > runner:
        print("FAIL  [image stamp refusal: the image branch refuses AFTER "
              "dispatching the image runner — the whole cell is measured "
              "before the refusal]")
        errors += 1
    else:
        print("ok    [image stamp refusal: the image branch refuses with "
              "`return 2` on image_stamp_range_deliverable() before it runs "
              "the image instantiation]")

    # The wrapper must DELEGATE to the constexpr decision, and its message
    # must carry the remedy.
    #
    # The decision moved into stamp_codec.hpp for a measured reason: while
    # it was an `if` here, a variant inverting it and a variant turning the
    # trailing `return false` into `return true` BOTH survived the whole
    # suite — nothing compiles this header, so the only arm reaching it
    # read its text, and text sees tokens rather than decisions. Polarity
    # is now pinned by compiled static_asserts; what is left to check here
    # is that the wrapper still asks that function, and that an operator
    # who trips it is told what to do.
    try:
        guard_body = _extract_cxx_block(
            text, "inline bool image_stamp_range_deliverable()",
            "image stamp refusal")
    except RuntimeError as e:
        print(f"FAIL  [image stamp headroom: {e}]")
        errors += 1
    else:
        if not re.search(r"if\s*\(\s*stamp_range_deliverable\s*\(\s*now"
                         r"\s*\)\s*\)", guard_body):
            print("FAIL  [image stamp headroom: the refusal does not gate "
                  "on stamp_range_deliverable(now) — the decision must stay "
                  "in the ROS-free header, where the compiled oracle can "
                  "pin which way it points]")
            errors += 1
        elif not re.search(r"return\s+false\s*;\s*\}\s*$",
                           guard_body.rstrip()):
            print("FAIL  [image stamp headroom: the refusal's message arm "
                  "does not end in `return false;` — printing the refusal "
                  "and then returning true runs the sweep anyway, which is "
                  "worse than never checking]")
            errors += 1
        elif "CER_BENCH_MSG=pod" not in guard_body:
            print("FAIL  [image stamp headroom: the refusal must name the "
                  "remedy (measure the pod class) — a refusal an operator "
                  "cannot act on is a wall, not a gate]")
            errors += 1
        else:
            print("ok    [image stamp headroom: the refusal gates on the "
                  "codec's stamp_range_deliverable(now) and names the pod "
                  "class as the remedy]")
        # ...and the DECISION must still supply the headroom. A
        # `ros_stamp_encodable(now_ns, 0)` there would satisfy every
        # compiled arm that passes its own headroom explicitly, while
        # re-admitting a run that starts one second below the ceiling and
        # crosses it mid-sweep.
        try:
            codec = _cxx_code_only(_STAMP_CODEC.read_text(encoding="utf-8"))
            decision = _extract_cxx_block(
                codec, "inline constexpr bool stamp_range_deliverable(",
                "image stamp headroom")
        except (OSError, RuntimeError) as e:
            print(f"FAIL  [image stamp headroom: {e}]")
            errors += 1
        else:
            if not re.search(r"ros_stamp_encodable\s*\(\s*now_ns\s*,\s*"
                             r"kStampHeadroomSeconds\s*\)", decision):
                print("FAIL  [image stamp headroom: stamp_range_deliverable "
                      "does not ask with kStampHeadroomSeconds — asking "
                      "about the current instant alone re-admits a run that "
                      "crosses the ceiling while it is running]")
                errors += 1
            else:
                print("ok    [image stamp headroom: the decision asks with "
                      "kStampHeadroomSeconds, not about `now` alone]")

    # (a) the compiled oracle.
    # $CXX is honoured, but only when it names ONE resolvable executable:
    # a perfectly ordinary `CXX="ccache clang++"` would otherwise reach
    # subprocess as a single filename, fail to spawn, and report this arm
    # FAILED on a host whose compiler is fine.
    env_cxx = os.environ.get("CXX")
    cxx = ((shutil.which(env_cxx) if env_cxx else None) or
           shutil.which("clang++") or shutil.which("c++"))
    if cxx is None:
        print("skip  [image stamp codec oracle: no C++ compiler (clang++ / "
              "c++ / $CXX) — the call-site arms above still ran]")
        return errors
    global _STAMP_ORACLE_RAN
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
           "-fsyntax-only", str(_STAMP_ORACLE)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=300)
    except (OSError, subprocess.SubprocessError) as e:
        print(f"FAIL  [image stamp codec oracle: {' '.join(cmd)} could not "
              f"run: {e}]")
        return errors + 1
    if r.returncode != 0:
        failed = [ln for ln in (r.stdout + r.stderr).splitlines()
                  if _is_assertion_failure(ln) or "error:" in ln]
        print(f"FAIL  [image stamp codec oracle: rc={r.returncode}]\n     "
              + "\n     ".join(failed[:6] or
                               [(r.stdout + r.stderr).strip()[-400:]]))
        return errors + 1
    # A GUTTED oracle compiles clean: an empty translation unit is valid
    # C++, so "the compile succeeded" is only evidence if something was
    # asserted. Verified: `: > empty.cpp && clang++ ... -fsyntax-only` is
    # rc=0.
    # Through the comment-blanked view, not the raw text: this file builds
    # _cxx_code_only precisely because a comment naming a token satisfies a
    # raw search, and block-commenting half the oracle leaves it compiling
    # clean under -Werror (the helpers stay used, so -Wunused catches
    # nothing) while a raw count still reports the full figure.
    n_asserts = _cxx_code_only(
        _STAMP_ORACLE.read_text(encoding="utf-8")).count("static_assert(")
    if n_asserts < _STAMP_ORACLE_MIN_ASSERTS:
        print(f"FAIL  [image stamp codec oracle: the oracle carries "
              f"{n_asserts} static_asserts, expected at least "
              f"{_STAMP_ORACLE_MIN_ASSERTS} — an empty file compiles clean, "
              f"so a gutted oracle would report this arm green having "
              f"checked nothing]")
        return errors + 1

    # TWO CONTROLS, because "the build succeeded" is only evidence if the
    # build could have failed AND the oracle is still attached to the code
    # it claims to test.
    #
    # The assert-count floor above is not enough on its own: it defends
    # against DELETION, and an oracle can be switched off while keeping
    # every `static_assert(` token — `#if 0` around the namespace, or
    # rewriting each as `static_assert(true || …)`. Both compile clean,
    # both keep the count at its full figure, and both were measured to
    # leave this arm printing `ok`.
    with tempfile.TemporaryDirectory(prefix="ctl_") as td:
        ctl = Path(td)
        # (a) NEGATIVE: a deliberately false assertion must fail the build
        # AND be attributable. This also proves the line filter matches
        # THIS compiler's wording — GCC and clang do not agree on it.
        neg_probe = ctl / "must_fail.cpp"
        neg_probe.write_text(
            '#include "%s"\n'
            "static_assert(ros2_rtt_bench::kRosTimeSecMax == 1,\n"
            '              "negative control");\n'
            % _STAMP_CODEC.resolve(), encoding="utf-8")
        neg = subprocess.run(cmd[:-1] + [str(neg_probe)], capture_output=True,
                             text=True, encoding="utf-8", errors="replace",
                             timeout=300)
        neg_lines = [ln for ln in (neg.stdout + neg.stderr).splitlines()
                     if _is_assertion_failure(ln) or "error:" in ln]
        # ...and a TRUE assertion over the same header must BUILD. Without
        # this the false probe's nonzero rc is not a discriminating claim:
        # a probe that failed for any reason at all (a bad include path, a
        # flag this compiler rejects) would satisfy it just as well.
        true_probe = ctl / "must_build.cpp"
        true_probe.write_text(
            '#include "%s"\n'
            "static_assert(ros2_rtt_bench::kRosTimeSecMax == 2147483647,\n"
            '              "positive control");\n'
            % _STAMP_CODEC.resolve(), encoding="utf-8")
        aff = subprocess.run(cmd[:-1] + [str(true_probe)],
                             capture_output=True, text=True,
                             encoding="utf-8", errors="replace", timeout=300)
        # (b) POSITIVE / LINKAGE: perturb a COPY of the shipping codec the
        # way the bug did — encode narrows instead of saturating —
        # and compile a COPY of the oracle against it. A live oracle FAILS.
        # A disabled one, a vacuous one, or one that has drifted onto its
        # own copy of the arithmetic all pass, and that is what this
        # catches.
        (ctl / "src").mkdir()
        (ctl / "test").mkdir()
        codec_src = _STAMP_CODEC.read_text(encoding="utf-8")
        # The perturbation keeps `limit` USED: replacing the whole ternary
        # with `sec` would orphan the local and make
        # the probe fail on `-Werror,-Wunused-variable` — nonzero for a
        # reason that has nothing to do with the oracle, so a completely
        # vacuous oracle would still "fail" it and the control would pass for the
        # wrong reason. That is why the verdict below ALSO requires a
        # static-assertion diagnostic rather than merely a nonzero rc.
        saturating = ("static_cast<int32_t>(sec > limit ? limit : sec),")
        if saturating not in codec_src:
            print("FAIL  [image stamp codec oracle: the linkage control "
                  "cannot find the saturating cast to perturb — "
                  "encode_ros_stamp has been rewritten and this control is "
                  "no longer perturbing the thing it names]")
            return errors + 1
        (ctl / "src" / _STAMP_CODEC.name).write_text(
            codec_src.replace(saturating,
                              "static_cast<int32_t>(sec > limit ? sec : sec),"),
            encoding="utf-8")
        (ctl / "test" / _STAMP_ORACLE.name).write_text(
            _STAMP_ORACLE.read_text(encoding="utf-8"), encoding="utf-8")
        pos = subprocess.run(
            cmd[:-1] + [str(ctl / "test" / _STAMP_ORACLE.name)],
            capture_output=True, text=True, encoding="utf-8",
            errors="replace", timeout=300)

        if aff.returncode != 0:
            print(f"FAIL  [image stamp codec oracle: a TRUE static_assert "
                  f"over the shipping header did not build "
                  f"(rc={aff.returncode}) — the probe harness itself is "
                  f"broken, so the false probe's failure below proves "
                  f"nothing\n     "
                  + (aff.stdout + aff.stderr).strip()[-300:] + "]")
            errors += 1
        elif neg.returncode == 0:
            print("FAIL  [image stamp codec oracle: a deliberately false "
                  "static_assert COMPILED — this arm cannot fail, so its "
                  "green says nothing]")
            errors += 1
        elif not neg_lines:
            print(f"FAIL  [image stamp codec oracle: a false static_assert "
                  f"failed the build but this arm's line filter extracted "
                  f"nothing from {Path(cxx).name}'s diagnostics, so a real "
                  f"failure would report no attributable line]")
            errors += 1
        elif not any(_is_assertion_failure(ln)
                     for ln in (pos.stdout + pos.stderr).splitlines()):
            print(f"FAIL  [image stamp codec oracle: re-introducing the "
                  f"narrowing encoder did not trip a single ASSERTION "
                  f"(rc={pos.returncode}) — the oracle is disabled, "
                  f"vacuous, or no longer reading the shipping header, so "
                  f"its green says nothing about the code that ships"
                  + (". It did fail to build, which is not the same claim"
                     if pos.returncode != 0 else "") + "]")
            errors += 1
        else:
            _STAMP_ORACLE_RAN = True
            print(f"ok    [image stamp codec oracle: {n_asserts} "
                  f"static_asserts over the shipping stamp_codec.hpp agree "
                  f"({Path(cxx).name} -Wall -Wextra -Wpedantic -Werror); a "
                  f"deliberately false one is caught and attributed, and "
                  f"re-introducing the narrowing encoder FAILS the oracle]")
    return errors


# ---------------------------------------------------------------------------
# The `cerulion` you validate with is part of the test
# ---------------------------------------------------------------------------

# The remedy every refusal must carry, read from the module under test so
# an arm cannot drift from the shipped text by a capital letter.
REBUILD_CMD = bench.CLI_REBUILD_CMD


def _touch_ns(path: Path, when_ns: int) -> None:
    """A stand-in `cerulion` at a chosen mtime.

    The mode is set EXPLICITLY, every time: the resolver requires a
    candidate to be an executable file, `write_text` preserves whatever
    mode an existing file already had, and one arm below deliberately
    chmods a fixture non-executable — so an implicit mode would leak that
    state into every arm that follows it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    os.chmod(path, 0o755)
    os.utime(path, ns=(when_ns, when_ns))


_BENCH_README = Path(__file__).resolve().parent / "README.md"


# ONE member per REACHING MECHANISM, and each reachable ONLY that way.
# A fixture that lets `cerulion_core` be reached three ways at once means
# that disabling the `workspace = true` resolution — the mechanism EVERY
# internal dependency in the real tree uses — changes nothing: a broken
# variant runs green. A fixture where the branches overlap tests none of
# them.
_CRATES_DIR = "crates"


def _fixture_member_dir(rel: str) -> str:
    """The repo-relative member DIRECTORY a `_CLOSURE_FIXTURE` key belongs
    to, or "" for a root-level file.

    The members live one level down, under `crates/`, which is also what
    `_cli_member_closure` returns and what `_CLI_CRATE_DIR` names — so a
    member directory is the first TWO path segments. Splitting on the first
    `/` alone returns the literal `crates` for every member, which is in
    neither classification table, so the totality check would compare
    `{"crates"}` against the real member set and the walk oracle would
    expect only the root files: both arms fail loudly rather than quietly,
    but neither would be asserting what it names.
    """
    parts = rel.split("/")
    if len(parts) >= 3 and parts[0] == _CRATES_DIR:
        return "/".join(parts[:2])
    return ""


_CLOSURE_FIXTURE = {
    "Cargo.toml": """[workspace]
members = ["crates/cerulion_cli", "crates/cerulion_core", "crates/cerulion_wire",
           "crates/cerulion_macros", "crates/cerulion_bagd", "crates/cerulion_bag",
           "crates/cerulion_link", "crates/cerulion_hygiene", "crates/cerulion_mdns",
           "crates/cerulion_discovery", "crates/cerulion_pairing", "crates/cerulion_wireclient",
           "crates/cerulion_heaphook",
           "crates/cerulion_decoy", "crates/cerulion_patched", "crates/cerulion_viz"]

[workspace.dependencies]
cerulion_core = { path = "crates/cerulion_core", version = "=0.1.0" }
# DOTTED keys: ONE entry arriving as several declarations. A reader that
# keeps the LAST reads the path-less one and demotes a real member to
# "registry dependency", losing it and everything reachable only through it.
cerulion_wireclient.path = "crates/cerulion_wireclient"
cerulion_wireclient.version = "=0.1.0"
# Inherited, and NOT in this tree: a workspace entry with no path is an
# ordinary registry dependency, and following it would be a wrong answer.
serde = { version = "1", features = ["derive"] }
cerulion_viz = { path = "crates/cerulion_viz", version = "=0.1.0" }

# TWO path patches, because a scanner that folds this whole table into one
# entry still finds the FIRST `path =` in the blob and looks correct. The
# real root manifest carries a single patch entry, so only a fixture with
# two of them can see that.
[patch.crates-io]
cerulion_decoy = { path = "crates/cerulion_decoy" }
cerulion_patched = { path = "crates/cerulion_patched" }

# ...and the SUB-TABLE spelling of the same thing, which a prefix-matching
# `patch` predicate parses as ordinary keys and loses. Its own member, so
# the loss is attributable to THAT branch and not to the dotted key.
[patch.crates-io.cerulion_mdns]
path = "crates/cerulion_mdns"
""",
    "Cargo.lock": "version = 3\n",
    "crates/cerulion_cli/Cargo.toml": """[package]
name = "cerulion_cli"

[dependencies]
# inherited from [workspace.dependencies] -- how nearly every internal
# dependency in the real tree is spelled
cerulion_core = { workspace = true }
# a direct path
cerulion_macros = { path = "../cerulion_macros" }
# inherited, and spelled in the root with dotted keys
cerulion_wireclient = { workspace = true }
# a QUOTED bare key -- valid TOML, the same key as `path`
cerulion_heaphook = { "path" = "../cerulion_heaphook" }
# a TOML LITERAL string: valid TOML, valid cargo, and a double-quote-only
# reader drops it silently
cerulion_link = { path = '../cerulion_link' }
# registry: carries no path, so it is not in this tree
clap = { version = "4", features = ["derive"] }
# inherited AND registry: present in the workspace table, no path there
serde = { workspace = true }

[target.'cfg(unix)'.dependencies]
# A `#` inside a string is data: truncating here would drop the path.
cerulion_bagd = { path = "../cerulion_bagd", features = ["a#b"] }
# A MULTI-LINE inline table -- the shape FIVE manifests in this workspace's
# own closure use (accountd, cli_engine, core, netd, remoted), and a joiner
# that mis-tracks it turns the whole narrowing off. Its OWN member, so the
# loss is attributable to the joiner and not to the target-gated table.
cerulion_discovery = { path = "../cerulion_discovery", features = [
    "a",
    "b",
] }

[dev-dependencies.cerulion_bag]
path = "../cerulion_bag"
# A MULTI-LINE ARRAY inside a sub-table. Ordinary cargo, and handled on
# only ONE of the two spellings until the join counted brackets too: this
# shape made the whole manifest unreadable and switched the narrowing off,
# while the equivalent inline `{ .. }` was accepted.
features = [
    "x",
    "y",
]
""",
    "crates/cerulion_cli/src/main.rs": "fn main() {}\n",
    # TRANSITIVE: reachable only THROUGH cerulion_core. Its dev-dependency
    # back on cerulion_cli is a real CYCLE -- this workspace has one
    # (cerulion_core <-> native_ros2_messages) and a walk without a dedup
    # does not terminate on it.
    "crates/cerulion_core/Cargo.toml": """[package]
name = "cerulion_core"

[dependencies]
cerulion_wire = { path = "../cerulion_wire" }

[dev-dependencies]
cerulion_cli = { path = "../cerulion_cli" }
""",
    "crates/cerulion_core/src/lib.rs": "// core\n",
    # A DOTTED key, the modern cargo spelling.
    "crates/cerulion_wire/Cargo.toml": """[package]
name = "cerulion_wire"

[dependencies]
cerulion_hygiene.path = "../cerulion_hygiene"
""",
    "crates/cerulion_wire/src/lib.rs": "// wire\n",
    "crates/cerulion_macros/Cargo.toml": '[package]\nname = "cerulion_macros"\n',
    "crates/cerulion_macros/src/lib.rs": "// macros\n",
    "crates/cerulion_bagd/Cargo.toml": '[package]\nname = "cerulion_bagd"\n',
    "crates/cerulion_bagd/src/lib.rs": "// bagd\n",
    "crates/cerulion_bag/Cargo.toml": '[package]\nname = "cerulion_bag"\n',
    "crates/cerulion_bag/src/lib.rs": "// bag\n",
    "crates/cerulion_link/Cargo.toml": '[package]\nname = "cerulion_link"\n',
    "crates/cerulion_link/src/lib.rs": "// link\n",
    "crates/cerulion_hygiene/Cargo.toml": '[package]\nname = "cerulion_hygiene"\n',
    "crates/cerulion_hygiene/src/lib.rs": "// hygiene\n",
    "crates/cerulion_mdns/Cargo.toml": '[package]\nname = "cerulion_mdns"\n',
    "crates/cerulion_mdns/src/lib.rs": "// mdns\n",
    "crates/cerulion_discovery/Cargo.toml": '[package]\nname = "cerulion_discovery"\n',
    "crates/cerulion_discovery/src/lib.rs": "// discovery\n",
    "crates/cerulion_decoy/Cargo.toml": '[package]\nname = "cerulion_decoy"\n',
    "crates/cerulion_decoy/src/lib.rs": "// decoy\n",
    # A patch target is COMPILED from this tree, so its own path
    # dependencies change the CLI too -- reachable only if the patch entry
    # is WALKED rather than merely added to the result.
    "crates/cerulion_patched/Cargo.toml": """[package]
name = "cerulion_patched"

[dependencies]
cerulion_pairing = { path = "../cerulion_pairing" }
""",
    "crates/cerulion_patched/src/lib.rs": "// patched\n",
    "crates/cerulion_pairing/Cargo.toml": '[package]\nname = "cerulion_pairing"\n',
    "crates/cerulion_pairing/src/lib.rs": "// pairing\n",
    "crates/cerulion_wireclient/Cargo.toml": '[package]\nname = "cerulion_wireclient"\n',
    "crates/cerulion_wireclient/src/lib.rs": "// wireclient\n",
    "crates/cerulion_heaphook/Cargo.toml": '[package]\nname = "cerulion_heaphook"\n',
    "crates/cerulion_heaphook/src/lib.rs": "// heaphook\n",
    # Depends ON core, but nothing depends on IT -- the shape the finding
    # names (the viz tree, rmw_cerulion, the fixtures).
    "crates/cerulion_viz/Cargo.toml": """[package]
name = "cerulion_viz"

[dependencies]
cerulion_core = { workspace = true }
""",
    "crates/cerulion_viz/src/lib.rs": "// viz\n",
}

# The closure the fixture must produce, and WHY each member is in it. The
# reason column is not decoration: it is what a failure prints, so a lost
# member names the scanner branch that lost it.
_CLOSURE_REACHED_BY = {
    "crates/cerulion_cli": "the root of the walk",
    "crates/cerulion_core": "`workspace = true`, resolved through the root manifest",
    "crates/cerulion_wire": "TRANSITIVELY, through cerulion_core",
    "crates/cerulion_hygiene": "a DOTTED key (`name.path = ...`) in cerulion_wire",
    "crates/cerulion_macros": "a direct `path` in [dependencies]",
    "crates/cerulion_link": "a path in TOML LITERAL quotes",
    "crates/cerulion_bagd": "a [target.'cfg(..)'.dependencies] table",
    "crates/cerulion_discovery": "a MULTI-LINE inline table",
    "crates/cerulion_bag": "a [dev-dependencies.X] sub-table",
    "crates/cerulion_decoy": "the FIRST [patch.crates-io] path entry",
    "crates/cerulion_patched": "the SECOND [patch.crates-io] path entry — lost by "
                        "a scanner that folds that table into one blob",
    "crates/cerulion_pairing": "a path dependency OF a patch target, reachable "
                        "only if patch entries are WALKED",
    "crates/cerulion_wireclient": "a workspace entry spelled with DOTTED keys, "
                           "inherited by the CLI",
    "crates/cerulion_heaphook": "a QUOTED bare key (`\"path\" = ...`)",
}
# Reached only through `[patch.crates-io.NAME]`, whose sub-table spelling a
# prefix-matching predicate parses as ordinary keys.
_CLOSURE_REACHED_BY["crates/cerulion_mdns"] = (
    "a [patch.crates-io.NAME] SUB-TABLE path entry")

# Members the fixture declares that must NOT be in the closure. Named, so
# deleting one from the fixture cannot quietly weaken the arm.
_CLOSURE_EXCLUDED = {
    "crates/cerulion_viz": "a workspace member that DEPENDS on core and that "
                    "nothing links",
}


def check_fixture_trees_do_not_spend_the_fallback_note() -> int:
    """A self-test that points `bench.REPO_ROOT` at a fixture must put the
    fallback note's once-per-process flag back.

    Rebinding REPO_ROOT to a fixture — or stubbing `_cli_member_closure`
    outright — is precisely what makes a walk un-narrowable, so any such
    function WILL trip the "cannot narrow" path and set the flag. MEASURED: one walk over a
    fixture tree flips it from False to True. Leaving it set means a
    genuine fallback later in the same run prints NOTHING — the one thing
    that note exists to prevent, silenced by a test.

    Structural rather than behavioural because the alternative is
    re-running a ten-second scenario to observe one boolean, and because
    the rule has to hold for the NEXT fixture someone writes, not only for
    the two here."""
    tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    offenders = []
    checked = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        rebinds = False
        reads = writes = False
        for sub in ast.walk(node):
            if isinstance(sub, ast.Attribute) and \
                    isinstance(sub.value, ast.Name) and sub.value.id == "bench":
                # BOTH ways a self-test forces the fallback: pointing
                # REPO_ROOT at a fixture (an un-narrowable tree) and
                # stubbing the derivation itself. Matching only the former
                # would miss a future function that stubs the derivation
                # instead.
                if sub.attr in ("REPO_ROOT", "_cli_member_closure") and \
                        isinstance(sub.ctx, ast.Store):
                    rebinds = True
                if sub.attr == "_CLI_CLOSURE_NOTE_SHOWN":
                    if isinstance(sub.ctx, ast.Store):
                        writes = True
                    else:
                        reads = True
        if rebinds:
            checked.append(node.name)
            if not (reads and writes):
                offenders.append(node.name)
    if not checked:
        print("FAIL  [fallback note: no self-test in this file rebinds "
              "`bench.REPO_ROOT` or `bench._cli_member_closure`, so this "
              "rule matches nothing — read it before trusting the arms "
              "that do]")
        return 1
    if offenders:
        print(f"FAIL  [fallback note: {', '.join(offenders)} force the "
              f"whole-workspace fallback (by pointing `bench.REPO_ROOT` at "
              f"a fixture, or by stubbing `bench._cli_member_closure`) "
              f"without saving and restoring "
              f"`bench._CLI_CLOSURE_NOTE_SHOWN`. Such a walk is "
              f"un-narrowable, so it SPENDS the once-per-process note and a "
              f"genuine fallback later in the run is silent]")
        return 1
    print(f"ok    [fallback note: all {len(checked)} fixture-rebinding "
          f"checks ({', '.join(sorted(checked))}) put the once-per-process "
          f"flag back]")
    return 0


def _closure_fixture_walk_oracle() -> set:
    """The files the walk must name over `_CLOSURE_FIXTURE`, DERIVED from
    the fixture rather than restated: a second copy of the layout breaks
    the arm whenever a fixture member is added, for a reason that has
    nothing to do with the closure."""
    return {rel for rel in _CLOSURE_FIXTURE
            if not _fixture_member_dir(rel)
            or _fixture_member_dir(rel) in _CLOSURE_REACHED_BY}


def check_cli_source_closure() -> int:
    """Editing a member `cerulion_cli` does not link must not make a good
    `cerulion` read STALE.

    The freshness floor is the newest mtime over the files that can change
    the CLI. It used to be the newest mtime over the WHOLE workspace, so a
    one-character edit to the viz tree or to rmw_cerulion raised it and the
    next run declined a binary those crates cannot affect. Safe (a loud
    skip naming the rebuild) but wrong, and on this tree it is 232 of 1436
    files.

    `_cli_member_closure` narrows it from the MANIFESTS -- never by
    invoking cargo, which this stdlib-only harness must run without. Four
    kinds of evidence, because no one of them is the property:

    (a) PURE oracles over the scanners, against hand-written tables --
        including the three-answer `_toml_path_value`, whose two
        non-string answers have OPPOSITE consequences.
    (b) A real git fixture whose shape mirrors this workspace, with one
        member per REACHING MECHANISM (inherited, direct path, literal
        quotes, dotted key, multi-line inline table, target-gated table,
        dev sub-table, transitive, both `[patch]` spellings) plus a
        dependency CYCLE and one member nothing links: the closure, the
        walk, and then THE claim -- touching the unlinked member leaves a
        binary FRESH, touching a linked one makes the same binary STALE.
    (c) The FAILURE directions: every shape the scanner cannot model must
        widen back to the whole workspace WITH a reason, never narrow on
        a guess.
    (d) The REAL tree, because (a)-(c) all run against a synthetic
        fixture: a derivation that collapses to "cannot narrow" on THIS
        checkout ships inert and green otherwise."""
    errors = 0

    # ---- (a) the scanners, against hand tables ---------------------------
    for label, line, want in (
        ("a bare comment", 'path = "a"  # gone', 'path = "a"  '),
        ("a # inside a basic string", 'f = ["a#b"] # gone', 'f = ["a#b"] '),
        ("a # inside a literal string", "f = 'a#b' # gone", "f = 'a#b' "),
        ("an escaped quote", 'n = "a\\"#b" # gone', 'n = "a\\"#b" '),
        ("no comment at all", 'path = "a"', 'path = "a"'),
    ):
        got = bench._toml_strip_comment(line)
        if got != want:
            print(f"FAIL  [cli closure: strip_comment({label}) => {got!r}, "
                  f"want {want!r}]")
            errors += 1
    for label, header, want in (
        ("a plain table", "dependencies", ["dependencies"]),
        ("a workspace table", "workspace.dependencies",
         ["workspace", "dependencies"]),
        ("a target-gated table", "target.'cfg(unix)'.dependencies",
         ["target", "cfg(unix)", "dependencies"]),
        # The dot lives INSIDE the quotes, so splitting on every dot would
        # produce a four-segment header and the table would be missed.
        ("a quoted segment holding a dot",
         'target."cfg(a.b)".dependencies',
         ["target", "cfg(a.b)", "dependencies"]),
    ):
        got = bench._toml_header_segments(header)
        if got != want:
            print(f"FAIL  [cli closure: header_segments({label}) => {got}, "
                  f"want {want}]")
            errors += 1
    for label, base, path, want in (
        ("a sibling", "crates/cerulion_cli", "../cerulion_core",
         "crates/cerulion_core"),
        ("a nested member", "", "crates/test_fixtures/x",
         "crates/test_fixtures/x"),
        # A member sits TWO segments deep (`crates/<name>`), so reaching the
        # checkout root takes `../..` and escaping it one more level again.
        # Kept at the real depth deliberately: with a one-segment base these
        # two arms pass while asserting the wrong thing about this tree.
        ("the checkout root", "crates/cerulion_cli", "../..", ""),
        ("an escape", "crates/cerulion_cli", "../../../elsewhere", None),
        ("an absolute path", "crates/cerulion_cli", "/opt/elsewhere", None),
    ):
        got = bench._repo_relative_dir(base, path)
        if got != want:
            print(f"FAIL  [cli closure: repo_relative_dir({label}) => "
                  f"{got!r}, want {want!r}]")
            errors += 1
    # THREE answers, and the difference decides the safe direction: a
    # missing `path` key is an ordinary registry dependency (skip it), a
    # `path` key this reader cannot parse is a member it would DROP
    # (widen instead). Collapsing both to None would lose that distinction.
    U = bench._UNREADABLE
    for label, value, want in (
        ("double quotes", '{ path = "../x" }', "../x"),
        ("literal quotes", "{ path = '../x' }", "../x"),
        ("a path beside other keys", '{ version = "1", path = "../y" }',
         "../y"),
        ("a QUOTED bare key", '{ "path" = "../x" }', "../x"),
        ("no path key at all", '"1.0"', None),
        ("a workspace inherit", "{ workspace = true }", None),
        ("a multi-line-literal path", '{ path = """../x""" }', U),
        ("an empty path", '{ path = "" }', U),
    ):
        got = bench._toml_path_value(value)
        if got is not want and got != want:
            shown = "UNREADABLE" if got is U else repr(got)
            print(f"FAIL  [cli closure: path_value({label}) => {shown}, want "
                  f"{'UNREADABLE' if want is U else repr(want)}]")
            errors += 1
    dep = lambda s: s[-1] in bench._DEP_TABLE_NAMES          # noqa: E731
    # A dependency-table line the scanner cannot read must ABANDON the
    # derivation, not skip the line: skipping is how a closure goes
    # silently under-inclusive and mints a confident CLI_FRESH.
    for label, text in (
        ("an unreadable inline line", "[dependencies]\n{ not a key = 1\n"),
        ("an unclosed inline table", '[dependencies]\na = { path = "b",\n'),
        # The sub-table body used to be appended verbatim, so the rule held
        # on one path and not the other.
        ("an unreadable sub-table line",
         "[dependencies.foo]\n{ not a key = 1\n"),
    ):
        if bench._manifest_declarations(text, dep) is not None:
            print(f"FAIL  [cli closure: {label} must make the whole "
                  f"manifest unreadable, not be skipped]")
            errors += 1
    # `[[bin]]`-style ARRAY-OF-TABLES headers share the `[...]` shape. The
    # scanner strips both bracket pairs, so `[[dependencies]]` — which
    # cargo does not accept and no manifest has — would otherwise read as
    # the dependency table and its entries would enter the closure. Pinned
    # because it is reachable by a one-character edit, not because a real
    # manifest does it.
    aot = bench._manifest_declarations(
        '[[bin]]\nname = "x"\npath = "src/main.rs"\n', dep)
    if aot:
        print(f"FAIL  [cli closure: an [[array-of-tables]] header is not a "
              f"dependency table, but the scanner read {aot} out of one — "
              f"a `path` there names a SOURCE FILE, not a member]")
        errors += 1
    # DECLARATIONS, not a name-keyed dict: one dependency may be declared
    # in two tables, and keeping only the last drops whichever member the
    # other declaration reached.
    both = bench._manifest_declarations(
        '[dependencies]\na = { path = "x" }\n'
        '[target.\'cfg(windows)\'.dependencies]\na = { path = "y" }\n', dep)
    if both != [("a", '{ path = "x" }'), ("a", '{ path = "y" }')]:
        print(f"FAIL  [cli closure: one name declared in two tables must "
              f"yield BOTH declarations, got {both}]")
        errors += 1

    if shutil.which("git") is None:
        print("skip  [cli closure: no `git` on PATH for the fixture arms]")
        return errors

    saved_root = bench.REPO_ROOT
    saved_cache = bench._CLI_SOURCE_FLOOR_CACHE
    # The "could not narrow" note is once-per-PROCESS, and the arms below
    # deliberately defeat the scanner — so without this save/restore a test
    # would SPEND the note and a genuine failure later in the same run
    # would be silent, which is the very thing the note exists to prevent.
    saved_note = bench._CLI_CLOSURE_NOTE_SHOWN
    try:
        with tempfile.TemporaryDirectory(prefix="closure_") as td:
            repo = Path(td) / "repo"
            for rel, text in _CLOSURE_FIXTURE.items():
                f = repo / rel
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_text(text, encoding="utf-8")
            git = ["git", "-c", "user.email=b@e", "-c", "user.name=b",
                   "-c", "commit.gpgsign=false"]
            for argv in (["init", "-q"], ["add", "-A"],
                         ["commit", "-q", "-m", "fixture"]):
                r = subprocess.run(git + argv, cwd=str(repo),
                                   capture_output=True, text=True, timeout=60)
                if r.returncode != 0:
                    print(f"FAIL  [cli closure: fixture `git {argv[0]}` "
                          f"rc={r.returncode}: "
                          f"{(r.stdout + r.stderr).strip()[-200:]}]")
                    return errors + 1
            bench.REPO_ROOT = repo

            # The fixture must be TOTAL: every member directory it declares
            # is either expected IN the closure (with the mechanism that
            # puts it there) or expected OUT of it. A member in neither set
            # is a fixture nobody is asserting anything about, which is how
            # a mechanism quietly stops being covered.
            declared = {m for m in (_fixture_member_dir(rel)
                                    for rel in _CLOSURE_FIXTURE) if m}
            classified = set(_CLOSURE_REACHED_BY) | set(_CLOSURE_EXCLUDED)
            if declared != classified:
                print(f"FAIL  [cli closure: the fixture declares "
                      f"{sorted(declared)} but classifies "
                      f"{sorted(classified)} — a member in neither set is "
                      f"asserted about by nothing]")
                errors += 1

            # ---- (b) the closure, the walk, and THE claim ----------------
            # A dedup-free walk does not terminate on the fixture's cycle,
            # so this call is also the termination pin.
            got, why = bench._cli_member_closure()
            want = set(_CLOSURE_REACHED_BY)
            if got != want:
                missing = sorted(want - (got or set()))
                extra = sorted((got or set()) - want)
                lost = "; ".join(f"{m} ({_CLOSURE_REACHED_BY[m]})"
                                 for m in missing)
                unwanted = "; ".join(
                    f"{m} ({_CLOSURE_EXCLUDED.get(m, 'not a linked member')})"
                    for m in extra)
                print(f"FAIL  [cli closure: the derivation names "
                      f"{sorted(got) if got is not None else None} "
                      f"(reason {why!r}), want {sorted(want)}."
                      + (f" LOST: {lost} — that names the scanner branch "
                         f"that stopped working." if missing else "")
                      + (f" UNWANTED: {unwanted}." if extra else "")
                      + "]")
                errors += 1
            bench._CLI_SOURCE_FLOOR_CACHE = None
            named = set(bench._tracked_cli_sources() or [])
            want_walk = _closure_fixture_walk_oracle()
            if named != want_walk:
                print(f"FAIL  [cli closure: the walk names {sorted(named)}, "
                      f"want {sorted(want_walk)} — the root manifests are "
                      f"always kept (a Cargo.lock bump is how a registry "
                      f"dependency reaches this gate) and the unlinked "
                      f"members are dropped]")
                errors += 1

            bench._CLI_SOURCE_FLOOR_CACHE = None
            base = bench.cerulion_source_floor_ns()
            if base is None:
                print("FAIL  [cli closure: the fixture tree is undatable, so "
                      "the freshness arms below would assert nothing]")
                return errors + 1
            # A binary built at the floor: FRESH now, by the classifier's
            # own equal-is-fresh rule.
            binary_ns = base
            later = base + 30_000_000_000
            os.utime(repo / "crates" / "cerulion_viz" / "src" / "lib.rs",
                     ns=(later, later))
            bench._CLI_SOURCE_FLOOR_CACHE = None
            verdict, reason = bench.classify_cerulion_binary(
                bench.PROV_IN_TREE, binary_ns,
                bench.cerulion_source_floor_ns())
            if verdict != bench.CLI_FRESH:
                print(f"FAIL  [cli closure: editing `cerulion_viz` — a "
                      f"member the CLI does not link — declined a good "
                      f"binary as {verdict} ({reason!r}). That is the "
                      f"finding: a viz edit must not send an operator to "
                      f"rebuild the CLI]")
                errors += 1
            # ...and the control, without which the arm above passes for a
            # closure that names nothing at all.
            #
            # cerulion_wire, not cerulion_core: it is reachable only
            # THROUGH core, so this control fails as well for a walk that
            # stops after the CLI's own direct dependencies.
            os.utime(repo / "crates" / "cerulion_wire" / "src" / "lib.rs",
                     ns=(later, later))
            bench._CLI_SOURCE_FLOOR_CACHE = None
            verdict, reason = bench.classify_cerulion_binary(
                bench.PROV_IN_TREE, binary_ns,
                bench.cerulion_source_floor_ns())
            if verdict != bench.CLI_STALE:
                print(f"FAIL  [cli closure: editing `cerulion_wire` — "
                      f"which the CLI links THROUGH cerulion_core — must "
                      f"make the same binary {bench.CLI_STALE}, got "
                      f"{verdict} ({reason!r}). The narrowing has gone "
                      f"under-inclusive, which is the direction that mints "
                      f"a confident CLI_FRESH]")
                errors += 1

            # ---- (c) every way it may refuse to narrow ------------------
            cli_manifest = repo / "crates" / "cerulion_cli" / "Cargo.toml"
            good = cli_manifest.read_text(encoding="utf-8")
            # The note is captured, not suppressed: it is asserted below.
            bench._CLI_CLOSURE_NOTE_SHOWN = False
            note_err = io.StringIO()
            for label, text, needle in (
                ("an unreadable manifest",
                 "[dependencies]\n{ not a key = 1\n",
                 "holds a shape this scanner cannot read"),
                # The dangerous ones: each of these used to be a `continue`
                # in some draft, and a `continue` here DROPS a member.
                ("a path this scanner cannot read",
                 '[dependencies]\nx = { path = """../cerulion_core""" }\n',
                 "with a `path` this scanner cannot read"),
                # THREE levels: the CLI manifest is `crates/cerulion_cli`,
                # so `../../elsewhere` lands on `elsewhere` INSIDE the
                # checkout and this arm would assert the unreadable-manifest
                # reason instead of the escape it names.
                ("a path outside the checkout",
                 '[dependencies]\nx = { path = "../../../elsewhere" }\n',
                 "outside this checkout"),
                ("a path naming a directory with no manifest",
                 '[dependencies]\nx = { path = "../nowhere" }\n',
                 "could not be read"),
                ("a workspace inherit the root does not declare",
                 "[dependencies]\nnot_declared = { workspace = true }\n",
                 "does not declare"),
                # A closure containing the checkout ROOT excludes nothing,
                # so it is a narrowing in name only — and a `""` entry
                # matching every path silently is worse than saying so.
                ("a path naming the checkout root",
                 '[dependencies]\nx = { path = "../.." }\n',
                 "checkout root"),
            ):
                cli_manifest.write_text(text, encoding="utf-8")
                closure, why = bench._cli_member_closure()
                if closure is not None:
                    print(f"FAIL  [cli closure: {label} must make the "
                          f"closure UNAVAILABLE, got {sorted(closure)}]")
                    errors += 1
                elif needle not in why:
                    print(f"FAIL  [cli closure: {label} must say why "
                          f"({needle!r}), got {why!r}]")
                    errors += 1
                bench._CLI_SOURCE_FLOOR_CACHE = None
                with contextlib.redirect_stderr(note_err):
                    widened = set(bench._tracked_cli_sources() or [])
                if "crates/cerulion_viz/src/lib.rs" not in widened:
                    print(f"FAIL  [cli closure: with the closure "
                          f"unavailable ({label}) the walk must widen back "
                          f"to the WHOLE workspace, got {sorted(widened)}]")
                    errors += 1
            cli_manifest.write_text(good, encoding="utf-8")
            # A patch target is seeded into the same frontier as a member,
            # so a failure while walking one reports a bare directory
            # unless the reason carries where it came from — and a name
            # only `[patch]` declares sends the reader through
            # `[dependencies]` tables that never had it.
            patched = repo / "crates" / "cerulion_patched" / "Cargo.toml"
            keep = patched.read_text(encoding="utf-8")
            patched.unlink()
            closure, why = bench._cli_member_closure()
            if closure is not None:
                print(f"FAIL  [cli closure: a patch target with no manifest "
                      f"must make the closure UNAVAILABLE, got "
                      f"{sorted(closure)}]")
                errors += 1
            elif "patched in as `cerulion_patched`" not in why:
                print(f"FAIL  [cli closure: a failure while walking a PATCH "
                      f"target must say it came from `[patch]`, got "
                      f"{why!r} — the name is in no `[dependencies]` table]")
                errors += 1
            patched.write_text(keep, encoding="utf-8")
            # A narrowing that switches itself off in silence is one nobody
            # can tell from a narrowing that is working.
            note = note_err.getvalue()
            if "could not be narrowed" not in note or \
                    "WHOLE workspace" not in note:
                print(f"FAIL  [cli closure: falling back to the whole "
                      f"workspace must SAY so, naming the reason — stderr "
                      f"carried {note.strip()[-200:]!r}]")
                errors += 1
            elif note.count("could not be narrowed") != 1:
                print(f"FAIL  [cli closure: the fallback note must be "
                      f"printed ONCE per process, not once per call — it "
                      f"appeared {note.count('could not be narrowed')} "
                      f"times across the arms above]")
                errors += 1
    finally:
        bench.REPO_ROOT = saved_root
        bench._CLI_SOURCE_FLOOR_CACHE = saved_cache
        bench._CLI_CLOSURE_NOTE_SHOWN = saved_note

    # ---- (d) the REAL tree: the narrowing must not ship inert ------------
    # Everything above runs against a synthetic fixture, so every one of
    # its arms stays green for a derivation that answers "cannot narrow"
    # on this checkout -- which is a feature that is off.
    real, why = bench._cli_member_closure()
    if not (REPO_ROOT / "crates" / "cerulion_cli" / "Cargo.toml").is_file():
        print("skip  [cli closure: this checkout has no cerulion_cli to "
              "narrow against]")
    elif real is None:
        print(f"FAIL  [cli closure: the derivation cannot narrow THIS "
              f"checkout ({why}) — every fixture arm above still passes, "
              f"so the narrowing would ship inert and green]")
        errors += 1
    else:
        want_in = {"crates/cerulion_cli", "crates/cerulion_core", "crates/cerulion_cli_engine"}
        want_out = {"crates/cerulion_viz", "crates/rmw_cerulion"}
        if not want_in <= real:
            print(f"FAIL  [cli closure: this checkout's closure is missing "
                  f"{sorted(want_in - real)} — the CLI links them, and a "
                  f"binary built before an edit to one would read a "
                  f"confident CLI_FRESH]")
            errors += 1
        elif real & want_out:
            print(f"FAIL  [cli closure: this checkout's closure still "
                  f"carries {sorted(real & want_out)}, which the CLI does "
                  f"not link — the narrowing is not narrowing]")
            errors += 1
        else:
            saved_cache2 = bench._CLI_SOURCE_FLOOR_CACHE
            bench._CLI_SOURCE_FLOOR_CACHE = None
            narrow = len(bench._tracked_cli_sources() or [])
            bench._CLI_SOURCE_FLOOR_CACHE = saved_cache2
            wide = sum(1 for p in _real_tree_source_paths())
            if narrow >= wide:
                print(f"FAIL  [cli closure: the narrowed walk names "
                      f"{narrow} files and the whole workspace names "
                      f"{wide} — nothing was actually dropped]")
                errors += 1
            else:
                print(f"ok    [cli source closure: on THIS checkout the "
                      f"closure is {len(real)} members and the walk drops "
                      f"{wide - narrow} of {wide} files]")

    if errors == 0:
        print(f"ok    [cli source closure: the scanners agree with their "
              f"hand tables; the fixture closure is "
              f"{sorted(_CLOSURE_REACHED_BY)} (one member per reaching "
              f"mechanism, over a dependency cycle); editing an unlinked "
              f"member leaves a binary FRESH while editing a "
              f"transitively-linked one makes the same binary STALE; and "
              f"every shape the scanner cannot model widens back to the "
              f"whole workspace with a reason]")
    return errors


def _real_tree_source_paths():
    """This checkout's source set with the closure narrowing DISABLED --
    the earlier walk, recomputed here so the (d) arm can say how much
    the narrowing actually drops."""
    saved = bench._cli_member_closure
    saved_cache = bench._CLI_SOURCE_FLOOR_CACHE
    saved_note = bench._CLI_CLOSURE_NOTE_SHOWN
    try:
        bench._cli_member_closure = lambda: (None, "narrowing disabled for "
                                                   "the comparison arm")
        bench._CLI_SOURCE_FLOOR_CACHE = None
        # This arm disables the narrowing ON PURPOSE, so its fallback note
        # is noise rather than news -- swallowed here, and the flag put
        # back, so a genuine fallback later in this process still says so.
        with contextlib.redirect_stderr(io.StringIO()):
            return bench._tracked_cli_sources() or []
    finally:
        bench._cli_member_closure = saved
        bench._CLI_SOURCE_FLOOR_CACHE = saved_cache
        bench._CLI_CLOSURE_NOTE_SHOWN = saved_note


def check_cerulion_override_is_documented() -> int:
    """The `CERULION` refusal is user-visible, so the README must describe
    it — and describe it COMPLETELY enough to answer the only question an
    operator has: will my campaign run or stop?

    Not a style check. The README already ENUMERATES the causes of the
    smoke gate's exit 1 ("a contradictory CER_BENCH_PACING or
    CER_BENCH_DMA_LOCK export, or a malformed CER_BENCH_PAYLOAD_SIZES"),
    and this PR added a cause to that list — leaving the enumeration
    actively wrong rather than merely thin.

    A HAND oracle over the outcome vocabulary, with a stated limit on its
    reach: it pins that each outcome the guard implements is NAMED, not
    that the prose describing it is correct. A new refusal added without a
    README row fails here; a row whose explanation drifts does not."""
    errors = 0
    try:
        doc = _BENCH_README.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [CERULION docs: cannot read {_BENCH_README.name}: {e}]")
        return 1
    if "CER_BENCH_DMA_LOCK` export, a `CERULION` override" not in doc:
        print("FAIL  [CERULION docs: the smoke exit-1 enumeration does not "
              "name the CERULION override — it lists the env-contract "
              "violations that refuse before any cell runs, and this is now "
              "one of them, so the list is wrong rather than short]")
        errors += 1
    # One phrase per outcome the guard can produce. Each is the WORD an
    # operator would search the README for after seeing the refusal.
    for outcome, needle in (
        ("empty means unset", "empty"),
        ("stale", "stale"),
        ("foreign", "foreign"),
        ("undatable / proceeding unverified", "UNVERIFIED"),
        ("whitespace-only", "whitespace-only"),
        ("relative path", "relative"),
        ("trailing separator", "separator"),
        ("not executable", "executable"),
        ("the runner's default path is exempt", "default path"),
        ("the deleted debug directory", "freshest-wins"),
        ("the remedy", "unset CERULION"),
        ("the rebuild remedy", "cargo build -p cerulion_cli --release"),
        ("the uncovered direct invocation", "does not go through"),
    ):
        if needle not in doc:
            print(f"FAIL  [CERULION docs: the README does not describe "
                  f"{outcome} (no {needle!r}) — an operator cannot predict "
                  f"whether a campaign runs or stops]")
            errors += 1
    # The exit-1 enumeration must name EVERY cause, not just the one this
    # arm was written for. An empty CARGO_TARGET_DIR became a fourth cause
    # and the enumeration kept listing three — a docs oracle that only
    # checks the old case IS the staleness class it exists to catch, so
    # the needle is pinned here and in cmd_smoke's docstring beside it.
    for where, text, needle in (
            ("README's exit-1 enumeration", doc, "empty `CARGO_TARGET_DIR`"),
            ("cmd_smoke's docstring",
             inspect.getdoc(bench.cmd_smoke) or "", "EMPTY\n    CARGO_TARGET_DIR")):
        probe = needle.replace("\n    ", " ")
        flat = " ".join(text.split())
        if probe.replace("`", "") not in flat.replace("`", ""):
            print(f"FAIL  [CERULION docs: {where} lists the causes of exit 1 "
                  f"but not an EMPTY CARGO_TARGET_DIR, which is refused "
                  f"before any cell runs and exits 1 like the rest. An "
                  f"enumeration that is missing a cause sends the reader "
                  f"looking for a bug that is not there]")
            errors += 1

    if errors == 0:
        print("ok    [CERULION docs: the README names the override in the "
              "exit-1 enumeration and describes every accepted, refused and "
              "warned outcome, with both remedies and the one path it does "
              "not cover; and that enumeration — in the README and in "
              "cmd_smoke's docstring — names the empty CARGO_TARGET_DIR "
              "cause too]")
    return errors


# Functions in check_percentile_parity.py that may NAME `CARGO_TARGET_DIR`,
# each with the reason it must. A DECLARED inventory rather than a shape
# rule, because in this module a legitimate mention and an illegitimate
# one are shaped alike: an arm that SETS the variable to drive the readers
# writes `os.environ[...] = ...`, and one that saves-and-restores it reads
# `os.environ.get(...)` — the same call a genuine fourth reader makes.
# Nothing in the AST separates a fixture from a decision, so the
# separation is declared and anything absent from this list is reported.
#
# bench.py carries no such list on purpose: it is production code, and
# there the ONE accessor is the only place with any business naming it.
_DECLARED_CTD_NAMERS = {
    # The walk itself, which names the variable in order to look for it —
    # and its nested predicate, which is where the literal actually sits.
    "check_percentile_parity.py:_cargo_target_dir_env_readers",
    "check_percentile_parity.py:names_the_var",
    # Drives the three readers under each spelling; sets and restores.
    "check_percentile_parity.py:check_empty_cargo_target_dir_is_refused",
    # Saves and restores an ambient value around its own fixtures.
    "check_percentile_parity.py:check_cerulion_binary_freshness",
}


def _cargo_target_dir_env_readers(
        sources: "Optional[Sequence[Tuple[Path, str]]]" = None) -> "List[str]":
    """Every place that NAMES `CARGO_TARGET_DIR`, by enclosing function —
    or `<module>` for one outside any function — across bench.py AND this
    file, minus the declared inventory above.

    An AST walk, not a regex: a comment naming the variable is not a read,
    and this file already pays for that distinction on the C++ side
    (`_cxx_code_only`). `ast` sees the difference for free.

    IT REPORTS A MENTION, NOT A RECOGNISED READ, and that is the whole
    design. The first draft matched a fixed shape — `os.environ`
    subscripted, or `.get`/`.pop`/`.setdefault` on it — and so answered
    "no other reader" for every spelling it had not predicted:
    `os.getenv("CARGO_TARGET_DIR")`, a module-level read outside any
    function (it collected function bodies only), `from os import environ`
    then a bare `environ[...]`, or a local alias. Each was RUN as a mutant
    and each came back green. A predicate that enumerates the ways to read
    an environment variable will always be behind somebody's next idea; a
    predicate that flags the NAME anywhere undeclared cannot be.

    BOTH modules, because this file had a reader of its own — the caller
    of `_pod_target_roots` — and a walk that covered only bench.py
    declared "no second place reads the variable" while a second place
    read it one module over.

    The cost is a false positive on any code that names the variable for a
    non-reading reason — paid by the COMMENT-blind AST, which is why this
    is an AST walk and not a grep. Prose is spared by construction rather
    than by blindness: `ast` DOES see docstrings (they are ordinary
    `Constant` nodes), but the match is exact equality against the whole
    name, which a sentence merely mentioning it never satisfies —
    measured, on this docstring, which names the variable repeatedly and
    does not trip it.

    THREE spellings count: the string literal, a bare
    `CARGO_TARGET_DIR_ENV` (how bench.py itself writes it), and the
    ATTRIBUTE form `<anything>.CARGO_TARGET_DIR_ENV` — which is the only
    legal spelling in THIS file, since it reaches the constant through
    `import bench`."""
    # `bench.__file__` is the module the behavioural arms actually drive,
    # not a second derivation of the path, so the walk cannot end up
    # reading a different copy of bench.py from the one under test.
    # Parameterised ONLY so the fixture arm can drive it over a scratch
    # file holding a hostile spelling; production callers pass nothing.
    if sources is None:
        sources = (
            (Path(bench.__file__).resolve(), ""),
            (Path(__file__).resolve(), "check_percentile_parity.py:"),
        )
    found: "List[str]" = []

    def names_the_var(node) -> bool:
        if isinstance(node, ast.Constant) and node.value == "CARGO_TARGET_DIR":
            return True
        if isinstance(node, ast.Name) and node.id == "CARGO_TARGET_DIR_ENV":
            return True
        # THE ATTRIBUTE FORM, and it is the one that matters in THIS file:
        # the checker does `import bench`, so a bare `CARGO_TARGET_DIR_ENV`
        # is not even legal here — the only spelling a reader would use is
        # `bench.CARGO_TARGET_DIR_ENV`, an ast.Attribute the Name test
        # never matched. A future `os.environ.get(bench.CARGO_TARGET_DIR_ENV)`
        # would therefore bypass the empty-value refusal while this arm
        # reported clean: a blind spot in the safety net itself. Measured
        # before fixing — a planted reader of exactly that shape scored 0.
        #
        # Matched on the ATTRIBUTE NAME alone, deliberately, rather than by
        # enumerating aliases of the bench module: `import bench as b` then
        # `b.CARGO_TARGET_DIR_ENV` is the same read, and any alias list is
        # one rename behind. Anything spelling that attribute is reported,
        # which is the fail-closed direction.
        return (isinstance(node, ast.Attribute)
                and node.attr == "CARGO_TARGET_DIR_ENV")

    def defines_the_constant(node, scope: str) -> bool:
        """Is this the `CARGO_TARGET_DIR_ENV = "CARGO_TARGET_DIR"` line?

        The ONE place the name must literally appear — spelling it once is
        the point of the constant. Exempted by SHAPE (an assignment whose
        single target is that exact name) and only at MODULE scope, so a
        function-local assignment to the same name buys no exemption. The
        VALUE is still walked: a bare skip here exempted the right-hand
        side too, and `CARGO_TARGET_DIR_ENV = os.environ.get(...)` — a
        read hidden on the constant's own definition line — came back
        clean."""
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            return False
        target = node.targets[0]
        return (isinstance(target, ast.Name)
                and target.id == "CARGO_TARGET_DIR_ENV"
                and scope == "<module>")

    def visit(node, scope: str, prefix: str) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                visit(child, child.name, prefix)
                continue
            if isinstance(child, ast.ClassDef):
                visit(child, f"{scope}.{child.name}", prefix)
                continue
            if defines_the_constant(child, scope):
                visit(child.value, scope, prefix)
                continue
            if names_the_var(child) and (prefix + scope) not in _DECLARED_CTD_NAMERS:
                found.append(prefix + scope)
            visit(child, scope, prefix)

    for path, prefix in sources:
        visit(ast.parse(path.read_text(encoding="utf-8")), "<module>", prefix)
    return sorted(set(found))


def check_empty_cargo_target_dir_is_refused() -> int:
    """`CARGO_TARGET_DIR=` (empty) must be REFUSED, the way cargo refuses
    it — by every reader, and only when it is exactly empty.

    Three readers here asked the question and gave three different answers,
    all of them inventing a configuration cargo does not have:
    `cargo_target_dir()` mapped the empty value to `<repo>/target` (a falsy
    test), `_runner_rebuilds()` inherited that and answered True, and
    `cli_provenance()` treated it as a set custom dir and answered
    `unknown`. Cargo's own answer is to refuse: measured on cargo 1.98.1,
    `CARGO_TARGET_DIR= cargo build` (and `check`, and `metadata`) exits 101
    with "the target directory is set to an empty string in the
    `CARGO_TARGET_DIR` environment variable". Nothing can be built or
    enumerated under that spelling, so vouching for a binary found beside
    it is vouching for one the runner could never have produced.

    The reviewer's one-token suggestion — treat empty as unset — was
    REFUTED on #801 and is what the WHITESPACE control below keeps refuted
    from the other side: `CARGO_TARGET_DIR=' '` BUILDS (measured, same
    cargo, into a directory literally named " "), so a `.strip()`-based
    refusal would invent a rule cargo does not have, in the direction that
    switches a working setup off. Exactly-empty, nothing else.

    One arm per reader, plus the two controls without which "it raised"
    says nothing: an ORDINARY value must still be answered (the readers
    are not simply broken) and the whitespace value must be answered too."""
    errors = 0
    saved = os.environ.get("CARGO_TARGET_DIR")

    # Every reader, and its non-refusing answer, so a reader that stops
    # refusing is caught by the same table that proves it answers at all.
    default_cli = bench.REPO_ROOT / "target" / "release" / "cerulion"
    readers = (
        ("cargo_target_dir", lambda: bench.cargo_target_dir()),
        ("_runner_rebuilds", lambda: bench._runner_rebuilds(default_cli)),
        ("cli_provenance", lambda: bench.cli_provenance(default_cli)),
    )
    try:
        # (a) EMPTY -> every reader refuses, and says enough to act on.
        os.environ["CARGO_TARGET_DIR"] = ""
        for name, call in readers:
            try:
                got = call()
            except SystemExit as e:
                # The CONTENT check lives HERE, in the handler that binds
                # the message. An earlier edit inserted the BaseException
                # arm between the two and left this block stranded after
                # that arm's `continue`, where Python never reached it —
                # so the arm asserted only that SOMETHING was raised while
                # its `ok` line went on claiming "naming the remedy".
                # Measured on the stranded version: a refusal rewritten to
                # the bare word "nope", and one spelled `SystemExit(0)` (a
                # refusal carrying a SUCCESS status — `bench.py smoke`
                # would then exit 0 having run nothing), BOTH scored
                # errors = 0.
                msg = str(e)
                missing = [w for w in ("CARGO_TARGET_DIR",
                                       "empty string",
                                       "unset CARGO_TARGET_DIR")
                           if w not in msg]
                if missing:
                    print(f"FAIL  [empty CARGO_TARGET_DIR: {name}'s refusal "
                          f"does not name {missing} — an operator who has "
                          f"an empty value exported in a shell rc cannot "
                          f"act on it: {msg[:160]!r}]")
                    errors += 1
                # A SystemExit carrying a zero or absent STATUS is not a
                # refusal at all: the entry points propagate it, and the
                # process exits 0 having measured nothing.
                if e.code in (0, None):
                    print(f"FAIL  [empty CARGO_TARGET_DIR: {name}() raised "
                          f"SystemExit({e.code!r}) — a refusal must carry a "
                          f"failing status, or `bench.py smoke` exits 0 "
                          f"under a configuration cargo cannot build]")
                    errors += 1
            except Exception as e:
                # A refusal spelled as some OTHER exception is not this
                # contract: it escapes every caller that handles SystemExit
                # (the entry points, resolve_cerulion_cli) and surfaces as
                # a traceback. Caught and REPORTED here rather than allowed
                # to propagate, or this arm would read as a checker crash.
                # `Exception`, NOT `BaseException`: the latter also swallows
                # the operator's Ctrl-C, three times over.
                print(f"FAIL  [empty CARGO_TARGET_DIR: {name}() raised "
                      f"{type(e).__name__} instead of SystemExit — the "
                      f"callers catch SystemExit, so this refusal reaches "
                      f"an operator as a traceback: {str(e)[:120]!r}]")
                errors += 1
                continue
            else:
                print(f"FAIL  [empty CARGO_TARGET_DIR: {name}() answered "
                      f"{got!r} instead of refusing. Cargo exits 101 on "
                      f"this configuration, so that answer describes a "
                      f"build that cannot happen]")
                errors += 1

        # (b) WHITESPACE -> NOT refused. A legal directory name, measured.
        #
        # Each reader is called ONCE and its outcome captured, refusal
        # included. A second, unguarded call for the value assertion below
        # would let a refusing variant's SystemExit escape this function
        # and abort the whole checker — turning an attributable FAIL into
        # a crash, and skipping arms (c) and (d) entirely. Measured on the
        # `.strip()` variant, which is exactly the shape this arm exists to
        # catch.
        os.environ["CARGO_TARGET_DIR"] = " "
        answers = {}
        for name, call in readers:
            try:
                answers[name] = call()
            except SystemExit as e:
                print(f"FAIL  [whitespace CARGO_TARGET_DIR: {name}() "
                      f"REFUSED ' ', which cargo builds into a directory of "
                      f"that name — a `.strip()` refusal invents a rule "
                      f"cargo does not have: {str(e)[:120]!r}]")
                errors += 1
        if "cargo_target_dir" in answers and \
                answers["cargo_target_dir"] != bench.REPO_ROOT / " ":
            print(f"FAIL  [whitespace CARGO_TARGET_DIR: it must resolve "
                  f"like any other relative value (against the repo root, "
                  f"cargo's cwd), got {answers['cargo_target_dir']!r}]")
            errors += 1
        # ...and the OTHER two readers must ANSWER, not merely not-refuse.
        # Without these a reader degenerating to a constant satisfies every
        # arm in this function: `_runner_rebuilds` returning True always
        # passes the unset control below and never refuses here, and
        # `cli_provenance` is asked nowhere else at all.
        if answers.get("_runner_rebuilds") is not False:
            print(f"FAIL  [whitespace CARGO_TARGET_DIR: the runner does NOT "
                  f"rebuild the file it runs under a custom target dir "
                  f"(cargo writes into ' ', the runner runs "
                  f"target/release/cerulion), so _runner_rebuilds must "
                  f"answer False — got "
                  f"{answers.get('_runner_rebuilds')!r}]")
            errors += 1
        # cli_provenance is asserted only to ANSWER, from its closed set.
        # The exact verdict here is ENVIRONMENT-DEPENDENT and must not be
        # pinned: with a custom target dir the path argument is withdrawn
        # and the DEPFILE decides, so a checkout WITH a built CLI reads
        # in_tree and one WITHOUT reads unknown. Pinning either spelling
        # would pass on this desk and fail on a developer's built tree —
        # a guard keyed on the environment, which is the shape this repo
        # bans outright. What IS invariant is that the reader answers at
        # all, with a member of the closed set.
        if answers.get("cli_provenance") not in bench.PROV_ANSWERS:
            print(f"FAIL  [whitespace CARGO_TARGET_DIR: cli_provenance must "
                  f"ANSWER from {list(bench.PROV_ANSWERS)} — got "
                  f"{answers.get('cli_provenance')!r}]")
            errors += 1

        # (c) UNSET -> the ordinary answers. Without this the refusals
        # above are satisfied by readers that refuse everything. Guarded
        # for the same reason as (b): a reader that refuses UNCONDITIONALLY
        # must be reported here, not allowed to abort the run.
        os.environ.pop("CARGO_TARGET_DIR", None)
        try:
            unset_dir = bench.cargo_target_dir()
            unset_rebuilds = bench._runner_rebuilds(default_cli)
            unset_prov = bench.cli_provenance(default_cli)
        except SystemExit as e:
            print(f"FAIL  [empty CARGO_TARGET_DIR control: a reader refused "
                  f"with the variable UNSET, which is the ordinary case "
                  f"every bench run takes: {str(e)[:120]!r}]")
            errors += 1
        else:
            if unset_dir != bench.REPO_ROOT / "target":
                print(f"FAIL  [empty CARGO_TARGET_DIR control: with the "
                      f"variable UNSET the target dir is the checkout's "
                      f"own, got {unset_dir!r}]")
                errors += 1
            if unset_rebuilds is not True:
                print("FAIL  [empty CARGO_TARGET_DIR control: with the "
                      "variable UNSET the runner does rebuild the file it "
                      "runs — a reader that refuses or denies everything "
                      "would satisfy the refusal arms above]")
                errors += 1
            # The default target dir is proof BY CONSTRUCTION only while it
            # IS the default, so unset must read in_tree where a custom dir
            # reads unknown. Pinned on BOTH sides, or `cli_provenance`
            # collapsing to a constant passes this whole function.
            if unset_prov != bench.PROV_IN_TREE:
                print(f"FAIL  [empty CARGO_TARGET_DIR control: with the "
                      f"variable UNSET a candidate under the checkout's own "
                      f"target/ is {bench.PROV_IN_TREE!r} by construction — "
                      f"got {unset_prov!r}]")
                errors += 1
    finally:
        if saved is None:
            os.environ.pop("CARGO_TARGET_DIR", None)
        else:
            os.environ["CARGO_TARGET_DIR"] = saved

    # (d2) THE WALK SEES EVERY SPELLING A READER COULD USE.
    # Arm (d) below asks the walk what it found; it cannot tell whether
    # the walk is CAPABLE of finding a given shape. So the shapes are
    # planted in a scratch file and the walk is pointed at that.
    #
    # The attribute form is the one this file would actually use: the
    # checker reaches the constant through `import bench`, so a bare
    # `CARGO_TARGET_DIR_ENV` is not even legal here and
    # `bench.CARGO_TARGET_DIR_ENV` is the idiomatic spelling. The Name-only
    # predicate never matched it — measured, a planted
    # `os.environ.get(bench.CARGO_TARGET_DIR_ENV)` scored 0 — which is a
    # blind spot in the safety net item 9 exists to be.
    with tempfile.TemporaryDirectory(prefix="walk_") as wtd:
        planted = Path(wtd) / "planted.py"
        planted.write_text(
            "import os\n"
            "import bench\n"
            "import bench as aliased\n"
            "\n\n"
            "def reads_via_attribute():\n"
            "    return os.environ.get(bench.CARGO_TARGET_DIR_ENV)\n"
            "\n\n"
            "def reads_via_aliased_attribute():\n"
            "    return os.environ.get(aliased.CARGO_TARGET_DIR_ENV)\n"
            "\n\n"
            "def reads_via_literal():\n"
            "    return os.getenv(\"CARGO_TARGET_DIR\")\n"
            "\n\n"
            "def reads_nothing():\n"
            "    return os.environ.get(\"SOME_OTHER_VAR\")\n",
            encoding="utf-8")
        try:
            seen = _cargo_target_dir_env_readers(
                sources=((planted, "planted:"),))
        except Exception as e:                        # noqa: BLE001
            print(f"FAIL  [env walk shapes: the walk raised "
                  f"{type(e).__name__} ({e}) on a scratch source]")
            errors += 1
            seen = []
        for fn, why in (
                ("reads_via_attribute",
                 "`bench.CARGO_TARGET_DIR_ENV` — the ONLY legal spelling in "
                 "this file, since it imports bench as a module"),
                ("reads_via_aliased_attribute",
                 "`aliased.CARGO_TARGET_DIR_ENV` — an alias list would "
                 "always be one rename behind, so the match is on the "
                 "attribute name"),
                ("reads_via_literal", "the bare string literal")):
            if f"planted:{fn}" not in seen:
                print(f"FAIL  [env walk shapes: a reader spelled as {why} "
                      f"is NOT seen by the walk (it reported {seen}). Such "
                      f"a reader would bypass the empty-value refusal "
                      f"while arm (d) below reported clean — a blind spot "
                      f"in the guard itself]")
                errors += 1
        # ...and the anti-tautology half: the walk must not flag a
        # function that names some OTHER variable, or every verdict above
        # is satisfied by a walk that reports everything.
        if "planted:reads_nothing" in seen:
            print(f"FAIL  [env walk shapes: the walk flagged a reader of an "
                  f"UNRELATED variable ({seen}) — it reports everything, so "
                  f"the shape verdicts above mean nothing]")
            errors += 1

    # (e) THE REFUSAL STILL RUNS main()'s CLEANUP.
    # The preflight raises from inside `main`, and `main` resets three
    # process globals in a `finally` so a SECOND in-process drive cannot
    # read the previous invocation's `--run-dir` and name the wrong
    # directory in its own refusal. Raised ABOVE that `try` — where the
    # preflight first sat — the refusal skipped the reset entirely, so a
    # later imported call inherited the FAILED run's args and an earlier
    # drive's posture basis. Driven for real here rather than read: the
    # arm invokes `main` in-process, takes the refusal, and asserts all
    # three globals came back.
    saved_e = os.environ.get("CARGO_TARGET_DIR")
    try:
        # Seed all three with values a later drive must not inherit.
        bench._REFUSAL_ARGS = argparse.Namespace(run_dir="/tmp/stale")
        bench._DMA_BASIS = "host"
        bench._DMA_PROBE = (True, "stale probe")
        os.environ["CARGO_TARGET_DIR"] = ""
        refused = False
        try:
            bench.main(["smoke"])
        except SystemExit:
            refused = True
        except Exception as e:                      # noqa: BLE001
            print(f"FAIL  [empty CARGO_TARGET_DIR cleanup: driving main() "
                  f"raised {type(e).__name__}, not the refusal: "
                  f"{str(e)[:120]!r}]")
            errors += 1
        if not refused:
            print("FAIL  [empty CARGO_TARGET_DIR cleanup: bench.main(['smoke']) "
                  "did not refuse under an empty value — the dispatch "
                  "preflight is not reached, so the cleanup claim below is "
                  "untestable and `smoke` would run under a configuration "
                  "cargo cannot build]")
            errors += 1
        else:
            stale = []
            if bench._REFUSAL_ARGS != argparse.Namespace():
                stale.append(f"_REFUSAL_ARGS={bench._REFUSAL_ARGS!r}")
            if bench._DMA_BASIS is not None:
                stale.append(f"_DMA_BASIS={bench._DMA_BASIS!r}")
            if bench._DMA_PROBE is not None:
                stale.append(f"_DMA_PROBE={bench._DMA_PROBE!r}")
            if stale:
                print(f"FAIL  [empty CARGO_TARGET_DIR cleanup: the refusal "
                      f"skipped main()'s reset — {stale} survived it. A "
                      f"later in-process drive then names the FAILED run's "
                      f"directory in its own refusal, and the posture "
                      f"basis is a one-way ratchet, so a leftover value "
                      f"locks out the next sweep's own declaration. Raise "
                      f"the preflight INSIDE main's try/finally]")
                errors += 1
    finally:
        bench._REFUSAL_ARGS = argparse.Namespace()
        bench._DMA_BASIS = None
        bench._DMA_PROBE = None
        if saved_e is None:
            os.environ.pop("CARGO_TARGET_DIR", None)
        else:
            os.environ["CARGO_TARGET_DIR"] = saved_e

    # (f) THE DISPATCH GUARD — this PR's whole marginal value.
    # Four subcommands reach the accessor through
    # `refuse_foreign_cerulion_override`'s preflight; `native` and `ros2`
    # BUILD without resolving a CLI and reach it only through main's
    # dispatch guard. Arm (e) above drives `smoke`, which is refused
    # through the OTHER path — so until this arm existed, deleting the
    # dispatch guard left the whole suite green while `bench.py native`
    # regressed to dying on a cargo error that names neither the variable
    # nor the remedy. Measured: deleting the guard scores 0.
    #
    # The command function is REPLACED by a recorder for the drive, so the
    # arm asserts the refusal fired BEFORE the verb did any work — and so
    # a regressed guard cannot start a real build (or a docker pull) from
    # inside the checker. `build_parser` looks the name up when it runs,
    # so patching the module attribute really does change dispatch.
    saved_f = os.environ.get("CARGO_TARGET_DIR")
    for verb in ("native", "ros2"):
        attr = "cmd_" + verb
        original = getattr(bench, attr)
        reached = []
        setattr(bench, attr, lambda a, _r=reached, _v=verb: (_r.append(_v), 0)[1])
        try:
            os.environ["CARGO_TARGET_DIR"] = ""
            bench._REFUSAL_ARGS = argparse.Namespace(run_dir="/tmp/stale-f")
            bench._DMA_BASIS = "host"
            bench._DMA_PROBE = (True, "stale probe")
            outcome = None
            try:
                bench.main([verb])
            except SystemExit as e:
                outcome = e
            except Exception as e:                    # noqa: BLE001
                print(f"FAIL  [dispatch guard: `bench.py {verb}` under an "
                      f"empty CARGO_TARGET_DIR raised "
                      f"{type(e).__name__} ({str(e)[:100]!r}) instead of "
                      f"the refusal]")
                errors += 1
                continue
            if outcome is None:
                print(f"FAIL  [dispatch guard: `bench.py {verb}` did NOT "
                      f"refuse under an empty CARGO_TARGET_DIR. It builds "
                      f"with cargo and resolves no CLI, so it reaches the "
                      f"accessor ONLY through main's dispatch guard — "
                      f"without it the verb dies on `cargo build failed in "
                      f"<dir>`, which names neither the variable nor the "
                      f"remedy]")
                errors += 1
            else:
                msg = str(outcome)
                missing = [w for w in ("CARGO_TARGET_DIR", "empty string",
                                       "unset CARGO_TARGET_DIR")
                           if w not in msg]
                if missing:
                    print(f"FAIL  [dispatch guard: `bench.py {verb}`'s "
                          f"refusal does not name {missing}: {msg[:140]!r}]")
                    errors += 1
                if outcome.code in (0, None):
                    print(f"FAIL  [dispatch guard: `bench.py {verb}` raised "
                          f"SystemExit({outcome.code!r}) — a refusal must "
                          f"carry a failing status]")
                    errors += 1
            if reached:
                print(f"FAIL  [dispatch guard: `cmd_{verb}` RAN before the "
                      f"refusal ({reached}) — the guard must refuse at "
                      f"dispatch, ahead of any build or image pull, not "
                      f"after the verb has started work]")
                errors += 1
            # main's cleanup must have run here too.
            leaked = [n for n, v, want in
                      (("_REFUSAL_ARGS", bench._REFUSAL_ARGS,
                        argparse.Namespace()),
                       ("_DMA_BASIS", bench._DMA_BASIS, None),
                       ("_DMA_PROBE", bench._DMA_PROBE, None))
                      if v != want]
            if leaked:
                print(f"FAIL  [dispatch guard: `bench.py {verb}`'s refusal "
                      f"skipped main()'s reset — {leaked} survived it]")
                errors += 1
        finally:
            setattr(bench, attr, original)
            bench._REFUSAL_ARGS = argparse.Namespace()
            bench._DMA_BASIS = None
            bench._DMA_PROBE = None
            if saved_f is None:
                os.environ.pop("CARGO_TARGET_DIR", None)
            else:
                os.environ["CARGO_TARGET_DIR"] = saved_f

    # (d) NO FOURTH READER. The three arms above name three functions; a
    # reader added later would answer the empty value however it liked and
    # no arm here would notice. Routing every read through the one
    # refusing accessor is what makes the behaviour above a property of
    # the file rather than of three functions somebody remembered.
    try:
        found = _cargo_target_dir_env_readers()
    except (OSError, SyntaxError, ValueError) as e:
        print(f"FAIL  [empty CARGO_TARGET_DIR: bench.py could not be parsed "
              f"for its environment readers ({e}) — this arm fails closed "
              f"rather than reporting no readers]")
        errors += 1
    else:
        # The accessor itself must be in the list, or the walk is finding
        # nothing and its emptiness means nothing.
        if "cargo_target_dir_setting" not in found:
            print(f"FAIL  [empty CARGO_TARGET_DIR: the AST walk does not "
                  f"see the read inside cargo_target_dir_setting itself "
                  f"(it found {found}) — the walk is broken, so any "
                  f"'no other reader' claim from it is vacuous]")
            errors += 1
        extra = [f for f in found if f != "cargo_target_dir_setting"]
        if extra:
            print(f"FAIL  [empty CARGO_TARGET_DIR: {extra} NAME "
                  f"CARGO_TARGET_DIR outside cargo_target_dir_setting(). "
                  f"Every reader must go through that accessor, which is "
                  f"where the empty value is refused — three readers "
                  f"giving three different answers to one question is the "
                  f"finding itself. This arm reports a MENTION rather than "
                  f"a recognised read, so `os.getenv`, a module-level "
                  f"read, an aliased `environ` and any spelling nobody has "
                  f"thought of yet all land here; if one of these names "
                  f"the variable for a non-reading reason, spell it "
                  f"through the accessor rather than narrowing the walk]")
            errors += 1

    if errors == 0:
        print("ok    [empty CARGO_TARGET_DIR: all three readers refuse it "
              "the way cargo does, with a failing status and naming the "
              "remedy; ' ' is still an ordinary directory name each reader "
              "ANSWERS, and unset still means the checkout's own target/; "
              "`native` and `ros2` are refused at DISPATCH, before either "
              "builds anything; every refusal still runs main()'s cleanup, "
              "so no later in-process drive inherits a stale run dir or "
              "posture; and the walk — which sees the literal, the bare "
              "name and the attribute form through any alias — finds the "
              "variable named nowhere but the one refusing accessor]")
    return errors


_SAMPLE_GATE = _ROS2_BENCH_SRC / "src" / "sample_gate.hpp"
_SAMPLE_GATE_ORACLE = _ROS2_BENCH_SRC / "test" / "sample_gate_oracle.cpp"
# A floor, not the exact count: an EMPTY translation unit compiles clean
# under -Werror, so "the oracle built" is evidence only if something was
# asserted. Raise it when the oracle grows; never lower it to match a
# deletion.
_SAMPLE_GATE_MIN_ASSERTS = 22
# Both spellings a compiler uses for an unhandled enumerator. Clang says
# `[-Werror,-Wswitch]`; GCC says `[-Werror=switch]` and words the message
# differently again. A filter that knew only clang's would report the
# exhaustiveness control GREEN on a GCC host whose diagnostic it could not
# see — a control that cannot fail.
_WSWITCH_SPELLINGS = ("-Wswitch", "-Werror=switch", "not handled in switch",
                      "enumeration value")
_SAMPLE_GATE_ORACLE_RAN = False

# The three latency SINKS, and the one function in each where an echo's
# stamp pair is judged. A table rather than three copies of the arm,
# because the whole finding was that one guard lived inline in three
# places and every one of them dropped silently.
# Per sink: the file, the function where the stamp pair is judged, the
# bail that function needs, and the expression the SECOND report_delivery()
# call must come AFTER. That last column is the whole point of the second
# call site — a call placed BEFORE the spin latches the at-most-once flag
# at start-up and makes every cell report all-zeroes, and two calls inside
# finalize() satisfy a bare count while leaving the receipt exactly as
# unreachable as it was before the split. Both were measured green against
# a count-only arm.
_SAMPLE_GATE_SINKS = (
    ("latency_node.cpp",
     "void on_echo(const typename Msg::ConstSharedPtr & msg)", "return",
     "rclcpp::spin(node);", "int run_latency()", "int run_latency()"),
    ("composed_rtt_node.cpp",
     "void on_echo(const typename Msg::ConstSharedPtr & msg)", "return",
     "exec.spin();", "int run_composed()", "int run_composed()"),
    # The rcl sink judges inside its drain LOOP, so its bail is `continue`
    # — a `return` there would abandon the run, not the sample. Its drain
    # loop IS its spin, so the second call sits after the loop's closing
    # brace; the anchor is the loop head.
    ("latency_node_rcl.cpp", "  int run()", "continue",
     "while (rclcpp::ok() && !done_) {", "int run_latency_rcl()",
     # The rcl sink's spin IS its drain loop, inside the member `run()` —
     # NOT inside the free run_latency_rcl(). Anchoring the receipt search
     # on the whole file matched the OUTER loop head instead, far above
     # every call site, and the arm was vacuous for this sink.
     "  int run()"),
)

# Every counter on a DELIVERY line is printed from the member of the same
# name plus a trailing underscore (`unstamped=%zu` <- `unstamped_`). That
# convention is what lets the report arm below check the format string
# against the ARGUMENT LIST positionally instead of merely asking whether
# each name appears somewhere.
# The two DROP arms, and for each: its counter, its sibling's counter (which
# must NOT appear in it), and a phrase its log line must carry so the two
# arms' messages cannot be swapped without notice.
_DROP_ARMS = (
    ("kUnstamped", "unstamped_", "nonpositive_rtt_", "NO stamp"),
    ("kNonPositiveRtt", "nonpositive_rtt_", "unstamped_", "not after"),
)


def _drop_arm_failures(fname: str, verdict: str, counter: str, sibling: str,
                       phrase: str, body: "Optional[str]",
                       bail: str) -> "List[str]":
    """Everything wrong with ONE drop arm, as messages — PURE, so the
    hostile bodies below can be driven without a sink on disk.

    Extracted from the loop because the inline version CRASHED on the very
    shape it exists to report: an arm whose body is empty — a fall-through
    arm, or one holding only a comment, which `_cxx_code_only` blanks —
    made `body.strip().splitlines()[0]` raise IndexError, so the checker
    died with a traceback and every later arm in `main()` was skipped
    instead of one clean FAIL being printed. Reproduced before fixing."""
    out: "List[str]" = []
    if body is None:
        out.append(f"FAIL  [sample gate call site: {fname} has no "
                   f"`case StampVerdict::{verdict}:` arm — the verdict "
                   f"falls through to whatever follows it]")
        return out
    first = body.strip()
    if not first.startswith(f"++{counter};"):
        # An EMPTY body is a real shape, not an impossible one, and it is
        # described rather than indexed into.
        opens_with = (repr(first.splitlines()[0].strip()) if first
                      else "an EMPTY body (a fall-through arm, or one "
                           "holding only a comment)")
        out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} "
                   f"arm does not OPEN with `++{counter};` (it opens with "
                   f"{opens_with}) — a bump nested inside the log-once "
                   f"guard never leaves 0, one placed after the bail never "
                   f"runs at all, and an empty arm drops the echo in "
                   f"exactly the silence this change removes]")
    if f"++{sibling}" in body:
        out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} arm "
                   f"also bumps `{sibling}` — the two conditions have "
                   f"different remedies and must not share a count]")
    guard = re.search(r"if\s*\(\s*" + counter + r"\s*==\s*1\s*\)", body)
    if guard is None:
        out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} arm "
                   f"has no `if ({counter} == 1)` log-once guard — either "
                   f"the first-of-regime line is gone (an all-unusable "
                   f"cell then says nothing while it runs) or it fires on "
                   f"EVERY drop, which is the flood this repo suppresses "
                   f"everywhere else]")
    else:
        try:
            guarded = _brace_block_at(body, guard.start(),
                                      f"sample gate/{fname}")
        except RuntimeError as e:
            out.append(f"FAIL  [sample gate call site: {e}]")
            guarded = ""
        if "RCLCPP_WARN" not in guarded:
            out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} "
                       f"arm does not log INSIDE its `{counter} == 1` "
                       f"guard — a WARN outside it fires on every drop, "
                       f"and an empty guard logs nothing at all; both pass "
                       f"a token search]")
        elif phrase not in guarded:
            out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} "
                       f"arm logs a message that does not say {phrase!r} — "
                       f"the two arms' texts are interchangeable and "
                       f"nothing else reads them, so a swap misdirects "
                       f"every operator who sees one]")
    if not re.search(r"\b" + bail + r"\s*;$", body.rstrip()):
        out.append(f"FAIL  [sample gate call site: {fname}'s {verdict} arm "
                   f"does not END with `{bail};` — counting the drop and "
                   f"then taking the sample anyway records an unsigned "
                   f"wrap under a label that says latency, and a bail "
                   f"placed EARLIER leaves the log-once line unreachable]")
    return out


_DELIVERY_KEY_RE = re.compile(r"(\w+)=%zu")


def _delivery_printf_pairs(body: str, where: str) -> "Tuple[List[str], List[str]]":
    """(keys, arguments) of the one `std::fprintf` on a DELIVERY line.

    Returned POSITIONALLY and compared positionally by the caller, because
    the bug this pins is an ORDER swap: a format string naming every key
    and an argument list passing every counter can still report each count
    under its neighbour's name, and every presence check in the world says
    yes to that. Measured: swapping two arguments passed the first draft of
    this arm."""
    # Selected by the LINE'S OWN TEXT, not by being the first fprintf in
    # the body: `run_bench.sh` greps `DELIVERY role=` (and `receipt_count`
    # matches `DELIVERY role=<role> ` whole-token), so that literal IS the
    # contract. Taking the first fprintf instead let ANY earlier printf
    # shadow the real one, after which no key matched and every content
    # check below was skipped in silence.
    marker = '"DELIVERY role=latency '
    at = body.find(marker)
    if at < 0:
        raise RuntimeError(
            f"{where}: no `DELIVERY role=latency ` literal in the report "
            f"body — run_bench.sh greps that exact prefix (and "
            f"receipt_count matches it with a TRAILING SPACE), so renaming "
            f"it silently empties every _delivery.txt while the node still "
            f"looks like it is reporting")
    # Walk back to the enclosing call so the argument list comes with it.
    call_at = body.rfind("std::fprintf(", 0, at)
    if call_at < 0:
        raise RuntimeError(f"{where}: the DELIVERY literal is not inside a "
                           f"`std::fprintf(` call")
    call = _paren_block_at(body, call_at, where)
    inner = call[1:-1]
    # The receipt goes to STDERR. `stderr` sits before the format string so
    # it is invisible to the positional check below, and run_bench.sh
    # captures the node's stderr into the bin log it greps.
    if not inner.lstrip().startswith("stderr"):
        raise RuntimeError(
            f"{where}: the DELIVERY receipt is not written to `stderr` "
            f"(first argument is {inner.lstrip()[:24]!r}) — run_bench.sh "
            f"greps the node log stderr captures")
    # Arguments begin after the LAST string-literal chunk (the format is
    # spelled as adjacent literals across several lines).
    end_of_fmt = inner.rfind('"')
    if end_of_fmt < 0:
        raise RuntimeError(f"{where}: the fprintf has no format string")
    fmt = inner[:end_of_fmt + 1]
    args = [a.strip() for a in inner[end_of_fmt + 1:].split(",") if a.strip()]
    return (_DELIVERY_KEY_RE.findall(fmt), args)


# Post-construction exits that are KNOWN to leave without a receipt, each
# with its reason. A DECLARED inventory rather than a shape exemption, so
# the residual is visible to a reader and any NEW un-receipted exit still
# fails the sweep.
#
# There is exactly one, and it is NOT fixed here on purpose: adding the
# call is a one-line change to a SINK, and the sinks in this PR carry
# container evidence (a colcon build plus live cells) taken at a specific
# tree. Changing one would invalidate that evidence and need another box
# run, so the residual is recorded and handed on rather than quietly
# closed. Its cost is bounded: the re-raise fires only when an RCLError
# is genuine AND rclcpp is still ok — i.e. a real transport fault, not
# the timeout or signal path the counters exist for — and the process
# then dies by exception with an RCL diagnostic already logged.
_DECLARED_UNRECEIPTED_EXITS = {
    ("latency_node_rcl.cpp", "throw"),
}

# Every C++ spelling a sink could be written as. ONE definition, used by
# both the sink-set derivation and the class sweep — they disagreed once,
# and the disagreement WAS the hole (`*.*pp` missed .cc/.cxx/.h).
_SINK_EXTENSIONS = ("*.cpp", "*.hpp", "*.cc", "*.cxx", "*.h", "*.c")
# Derived, not repeated: the two tables disagreed on `.c`, so a
# `src/sneaky.c` named by a CMake target landed in the BUILT set, was
# under src/ (so the outside-src claim was silent) and never appeared in
# the src/ walk (so the accounting claim never fired) — it escaped the
# accounting whose whole point is to be inescapable.
_CMAKE_SOURCE_SUFFIXES = tuple(pat[1:] for pat in _SINK_EXTENSIONS)


def _sink_candidate_files() -> "List[Path]":
    """Every file under the bench's src/ that could be a sink."""
    return sorted(q for pat in _SINK_EXTENSIONS
                  for q in (_ROS2_BENCH_SRC / "src").glob(pat))


def _bash_while_blocks(text: str) -> "List[Tuple[str, str]]":
    """Every TOP-LEVEL `while <cond> ; do <body> done` in `text`.

    Structural, because the thing being checked is a LOOP, and a token
    search cannot see one: `while [`, `sleep` and `kill -0 "$pid"` all
    appearing somewhere in a slice says nothing about whether the poll is
    inside the loop or whether the loop is bounded at all. `done` is
    counted against `do` so a nested loop does not close the outer one.

    TOP-LEVEL is the second half of that, and it is load-bearing rather
    than tidy: the caller accepts the slice if ANY returned loop is a
    grace, so an INNER loop that is perfectly bounded would vouch for an
    outer `while true` wrapped around it — a wait that still never ends.
    A loop nested inside another cannot bound the wait, so it is not
    offered; its text is still part of the outer body, which is where the
    `sleep` and the poll are looked for, so a poll inside a nested `if`
    (the real ping/pong spelling) still counts.
    """
    out: "List[Tuple[str, str]]" = []
    word = r"(?<![\w-]){}(?![\w-])"
    consumed = 0
    for m in re.finditer(word.format("while"), text):
        if m.start() < consumed:
            continue
        opener = re.search(word.format("do"), text[m.end():])
        if opener is None:
            continue
        cond = text[m.end():m.end() + opener.start()]
        body_start = m.end() + opener.end()
        depth = 1
        for t in re.finditer(word.format("(?:do|done)"), text[body_start:]):
            depth += 1 if t.group(0) == "do" else -1
            if depth == 0:
                out.append((cond, text[body_start:body_start + t.start()]))
                consumed = body_start + t.end()
                break
    return out


# A shell word holding a variable: `$n`, `${n}`, `"$n"`, `"${n}"`. The `$`
# is mandatory, so a bare literal (`0`) can never be read as the counter.
_SH_VAR = r'"?\$\{?(\w+)\}?"?'
# The ceiling a counter is compared against: a literal, or a named one.
_SH_LIMIT = r'(?:\d+|"?\$\{?\w+\}?"?)'

# Counter on the LEFT, rising toward a ceiling:  [ "$n" -lt 20 ]
_BOUND_RISING = re.compile(_SH_VAR + r"\s+-l[te]\s+" + _SH_LIMIT)
# The MIRRORED spelling, ceiling on the left:    [ 20 -gt "$n" ]
_BOUND_MIRRORED = re.compile(_SH_LIMIT + r"\s+-g[te]\s+" + _SH_VAR)
# The shapes that LOOK like a bound and are not: `-gt 0` / `-ne 0` put a
# FLOOR under a counter, not a ceiling over it.
_BOUND_FLOOR_ONLY = re.compile(_SH_VAR + r"\s+-(?:gt|ge|ne)\s+\d+")

# ---- the CONDITION grammar -------------------------------------------
#
# Finding the ceiling and the increment says nothing on its own, because
# neither is read in the context of the condition's LOGICAL STRUCTURE:
#
#     while [ "$n" -lt "$MAX" ] || true; do sleep 0.1; n=$((n + 1)); done
#
# has a ceiling over `$n`, has an increment of `$n`, and never ends. The
# fix is not another rejected spelling — `|| true` has siblings in
# `|| [ -f /tmp/go ]`, `! [ … ]`, `-o`, `; true`, `$(cmd)` and `(( … ))`,
# and a reject-list is only ever as long as the last review. What is
# checked is the WHOLE condition, against a grammar:
#
#     condition := test ( '&&' test )*
#     test      := '[' … ']' | '[[' … ']]' | 'kill -0' …
#                | 'node_still_ours' …
#
# Conjunction is the one safe combinator, and that is a property rather
# than a convention: `A && B` is FALSE whenever A is, so no conjunct —
# however vacuous — can outlive the ceiling, while a single always-true
# DISJUNCT makes the whole condition always true no matter how tight the
# ceiling is. Everything outside the grammar is REFUSED BY NAME rather
# than interpreted, because the thing this check stands in front of is a
# reap that hangs forever: a checker that cannot prove the loop
# terminates has to say so out loud, not guess.
#
# Known deliberate refusals: a `[[ a && b ]]` spelling (write it as two
# `&&`-joined tests) and an arithmetic `(( n < MAX ))` condition (write
# the bound as a `[ … ]` test). Both are legal bash and neither appears
# in the harness; each refusal NAMES its token, so it is a loud "spell it
# differently", never a silent pass.
_COND_REFUSED = (
    (re.compile(r"\|\|"), "||",
     "a disjunction — ONE always-true alternative (`|| true`, "
     "`|| [ -f /tmp/go ]`) makes the whole condition true however tight "
     "the ceiling is"),
    (re.compile(r"(?<![\w!<>=-])!(?!=)"), "!",
     "a negation, which inverts the test the bound is read off — spell "
     "the bound positively"),
    (re.compile(r"(?<![\w-])-o(?![\w-])"), "-o",
     "`test`'s own disjunction, inside the brackets"),
    (re.compile(r"\(\("), "((",
     "an arithmetic condition this check does not parse — spell the "
     "bound as a `[ … ]` test"),
    (re.compile(r"\$\("), "$(",
     "a command substitution: the condition's status is then whatever "
     "some other command returns"),
    (re.compile(r"`"), "`",
     "a command substitution: the condition's status is then whatever "
     "some other command returns"),
    (re.compile(r"(?<![\w./-])(?:true|false)(?![\w./-])"), "true/false",
     "a constant-status command, not a test"),
    (re.compile(r"(?<![\w./:-]):(?![\w./:-])"), ":",
     "the null command, which always succeeds"),
    # A `;` list and a pipeline both take the status of their LAST
    # command, so either can carry a real ceiling test in front and still
    # be true regardless — the same bypass as `|| true`, spelled without
    # an operator this check would otherwise notice. `||` is matched
    # first, so a bare `|` reaching here is a pipe.
    (re.compile(r";"), ";",
     "a command list: bash takes the LAST command's status, so a real "
     "ceiling test in front of it decides nothing"),
    (re.compile(r"\|"), "|",
     "a pipeline: its status is the LAST command's, so a real ceiling "
     "test at the head of it decides nothing"),
)

# One conjunct, anchored at BOTH ends so a trailing `; true` (bash takes
# the LAST command's status) cannot ride along behind a real test.
_COND_TEST = re.compile(
    r"^(?:\[\[.*\]\]|\[.*\]|kill\s+-0\s+\S.*|node_still_ours\s+\S.*)$",
    re.DOTALL)


def _loop_condition_tests(cond: str) -> "Tuple[Optional[str], List[str]]":
    """Parse a `while` condition as `test ( && test )*`.

    Returns `(refusal, tests)` — `refusal` is None exactly when the
    condition is inside the grammar, and `tests` are its conjuncts.
    """
    text = cond.strip()
    while text.endswith(";"):
        text = text[:-1].strip()
    if not text:
        return ("its condition is empty", [])
    for pat, name, why in _COND_REFUSED:
        if pat.search(text):
            return (f"its condition contains `{name}` — {why}. The only "
                    f"accepted condition is one `[ … ]`/`[[ … ]]` test, "
                    f"optionally `&&`-joined to further `[`/`[[`/`kill -0` "
                    f"tests, because conjunction is the only combinator a "
                    f"ceiling survives", [])
    tests = [t.strip() for t in text.split("&&")]
    for t in tests:
        # `&&` is gone from the conjuncts, so a surviving `&` is a
        # background operator (which makes the status the ASYNC job's,
        # i.e. always 0). `2>&1` / `&>file` are redirections, not that.
        if re.search(r"(?<![>])&(?![>])", t):
            return (f"its condition contains `&` outside a redirection, in "
                    f"`{t[:48]}` — backgrounding a test makes the "
                    f"condition's status the async job's, which is always "
                    f"true", [])
        if not _COND_TEST.match(t):
            return (f"its condition has a conjunct that is not a test: "
                    f"`{t[:48]}`. Every conjunct must be a `[ … ]`, "
                    f"`[[ … ]]` or `kill -0` test, so that nothing in the "
                    f"condition can outlive the ceiling", [])
    return (None, tests)


def _loop_counter_bounded(cond: str, body: str) -> "Optional[str]":
    """Is this loop FINITE — a counter with a ceiling, raised toward it?

    Returns None when it is, else why not.

    The first version of this check accepted `-gt 0` as a "numeric
    ceiling". It is not one: `while [ "$n" -gt 0 ]; do …; n=$((n + 1));
    done` satisfies it and runs FOREVER — so the checker would have
    approved a reap that hangs, which is the outcome the bound exists to
    prevent. A floor is not a ceiling, and the two are one character
    apart.

    THREE halves, because any two of them are satisfiable by a loop that
    runs forever: a CONDITION inside the grammar above (`|| true` beside a
    perfect ceiling and a perfect increment is still an infinite loop),
    a comparison in one of its bracket tests that puts a CEILING over the
    counter, AND an INCREMENT of THAT counter in the body. A ceiling
    nothing moves toward never trips; an increment with no ceiling never
    stops; and either, under a condition that is true regardless, is
    decoration.
    """
    refusal, tests = _loop_condition_tests(cond)
    if refusal is not None:
        return refusal
    brackets = [t for t in tests if t.startswith("[")]
    m = None
    for t in brackets:
        m = _BOUND_RISING.search(t) or _BOUND_MIRRORED.search(t)
        if m is not None:
            break
    if m is None:
        if any(_BOUND_FLOOR_ONLY.search(t) for t in brackets):
            return ("its condition puts a FLOOR under the counter "
                    "(`-gt`/`-ge`/`-ne` against a literal) rather than a "
                    "ceiling over it — that is not a bound, and with an "
                    "increment in the body the loop never ends")
        return ("no finite upper bound in its condition (an unbounded wait "
                "can hang the reap forever)")
    counter = m.group(1)
    raised = re.search(
        r"\b" + counter + r"\s*=\s*\$\(\(\s*" + counter + r"\s*\+"
        r"|\(\(\s*" + counter + r"\s*(?:\+\+|\+=)"
        r"|\blet\s+" + counter + r"\s*(?:\+\+|\+=)",
        body)
    if raised is None:
        return (f"its condition bounds `{counter}`, but the body never "
                f"raises `{counter}` toward that bound — the ceiling can "
                f"never be reached, so the loop is unbounded in practice")
    return None


def _grace_loop_failure(between: str, pid_var: str, role: str,
                        sig_a: str, sig_b: str) -> "Optional[str]":
    """Is there a BOUNDED loop between two rungs that polls this pid?

    Three properties, and the token check this replaces had none of them —
    it accepted a slice where `while [`, `sleep` and `kill -0 "$pid"`
    merely co-occurred, so a poll sitting OUTSIDE the loop, or an
    UNBOUNDED `while :`, both passed:

      BOUNDED   a CONDITION inside the `test ( && test )*` grammar, a
                FINITE ceiling over the counter in one of its bracket
                tests, and an increment of that counter in the body
                raising it toward the ceiling — see
                `_loop_counter_bounded`. Without all three, the reap can
                hang forever on a node that ignores the signal, which is
                worse than the ungraceful kill it replaced.
      POLLS     `kill -0 "<this role's pid>"` in the condition or the
                body. The two shapes differ legitimately — the latency
                loop polls in its condition, the ping/pong loop polls a
                PAIR inside an `if` in its body — so both are accepted,
                but it must be THIS role's pid: a loop waiting on some
                other process is not a grace for this one.
      SLEEPS    `sleep` in the BODY, or the loop spins hot instead of
                giving the node time to act.
    """
    # The IDENTITY argument is part of the poll, not decoration. Matching
    # `node_still_ours "<pid>"` as a prefix accepts a ONE-argument call,
    # and that call is not a weaker check — it is the FALLBACK, a bare
    # `kill -0`. So a grace polling without the identity reads a recycled
    # pid as "still running", keeps waiting on a stranger, and ends at the
    # unscoped `pkill` backstop with the receipt lost. Built and RUN
    # against this checker before the fix: 237 ok / 0 FAIL with both
    # latency grace polls stripped to one argument.
    poll = f'node_still_ours "{pid_var}" "{pid_var[:-4]}_id"'
    best: "Optional[str]" = None
    for cond, body in _bash_while_blocks(between):
        missing = []
        unbounded = _loop_counter_bounded(cond, body)
        if unbounded is not None:
            missing.append(unbounded)
        if poll not in cond and poll not in body:
            missing.append(f"it never polls {pid_var} inside the loop")
        if not re.search(r"(?<![\w-])sleep(?![\w-])", body):
            missing.append("no `sleep` in its body (a hot spin, not a "
                           "grace)")
        if not missing:
            return None
        if best is None:
            best = "; ".join(missing)
    if best is None:
        return (f"there is no `while … do … done` between the {sig_a} and "
                f"the {sig_b} sent to the {role} role at all")
    return (f"the loop between the {sig_a} and the {sig_b} sent to the "
            f"{role} role is not a grace: {best}")


def _owned_rung(pid_var: str, sig: str) -> str:
    """The one accepted spelling of a rung: signalled BY IDENTITY.

    A pid is reusable and these nodes are `&` children of this shell, so
    bash's SIGCHLD handler reaps them asynchronously and the kernel is
    free to hand the pid to somebody else the moment it does. A bare
    `kill -SIG "$pid"` cannot tell the difference; `kill_owned_pid`
    re-checks the recorded start time and refuses a stranger.

    A ladder makes that window wide rather than incidental: each rung is
    sent up to a whole grace after the last liveness poll, and the rung
    following a grace that ended BECAUSE the child exited is aimed at a
    pid just watched to disappear. run_bench.sh runs as root inside the
    container, so a stray KILL is unrestricted."""
    return f'kill_owned_pid "{pid_var}" "{pid_var[:-4]}_id" "" {sig}'


# Every signal name a reap could use. Spelled out rather than matched as
# `-\w+`, so the guard cannot be satisfied by re-spelling the signal.
_BARE_KILL_SIGS = ("INT", "TERM", "KILL", "HUP", "QUIT", "USR1", "USR2",
                   "STOP", "CONT", "2", "9", "15",
                   # `-0` sends nothing, but a bare one is the same
                   # defect wearing a poll's clothes: the OUTER
                   # wall-ceiling loop GATES the whole ladder and sits
                   # inside no rung slice, so `_grace_loop_failure` never
                   # sees it. Reverting that loop to `kill -0
                   # "$latency_pid"` was caught by no arm at all. The
                   # helper's own fallback spells it `"$1"`, so it is not
                   # matched here.
                   "0")


# Every pid this script records and later signals or polls, with the
# identity variable that makes the claim checkable and the `comm` its
# rungs pass. The three NODE roles were the whole of this table until a
# review found that the two DAEMON pids climbed an identity-checked TERM
# and then escalated with the identity DROPPED — a bare `kill -0` grace
# poll and a bare `kill -KILL` backstop, twice each (the EXIT trap and
# stop_iox_roudi) — so the one signal this script's own comment says must
# never be sent blind was sent blind at the end of every teardown. Worse
# than the node case, not better: a daemon pid is recorded at START and
# escalated at TEARDOWN, so the window is the whole cell rather than a 2 s
# grace, and the script runs as root in the container.
#
# The comm column is the remedy's third argument, not a second check: it
# is `iox-roudi` for the unwrapped daemon, and EMPTY for everything behind
# `ros2 run` or a `chrt`/`taskset` prefix, whose comm is not stable enough
# to gate on (the start-time identity still pins those).
_IDENTITY_GATED_PIDS = (
    ("latency", "$latency_pid", "$latency_id", ""),
    ("ping", "$ping_pid", "$ping_id", ""),
    ("pong", "$pong_pid", "$pong_id", ""),
    ("iox-roudi", "$IOX_ROUDI_PID", "$IOX_ROUDI_ID", "iox-roudi"),
    ("zenohd launcher", "$ZENOHD_PID", "$ZENOHD_ID", ""),
    # `kill_router_pid` passes `rmw_zenohd`; the column is the remedy
    # text's third argument, so a wrong value here prints advice weaker
    # than the code.
    ("zenohd router", "$ZENOHD_ROUTER_PID", "$ZENOHD_ROUTER_ID",
     "rmw_zenohd"),
)


def _bare_kill_failures(runner: str, role: str, pid_var: str, id_var: str,
                        comm: str) -> int:
    """No rung anywhere may be sent to this pid WITHOUT the identity check
    — the ladder arm below proves the right spellings are present, not
    that a wrong one is absent.

    A bare `kill -0` counts: it sends nothing, but it is the same defect
    wearing a poll's clothes, and a grace that watches a PID rather than a
    PROCESS hands the rung after it to whatever the kernel recycled that
    pid onto."""
    bare = [sig for sig in _BARE_KILL_SIGS
            if f'kill -{sig} "{pid_var}"' in runner]
    if not bare:
        return 0
    print(f"FAIL  [signal ladder: run_bench.sh sends {bare} to "
          f"{pid_var} ({role}) with a BARE `kill`, bypassing the identity "
          f"check. A pid is reusable — a node is an `&` child this "
          f"shell's SIGCHLD handler reaps asynchronously, and a daemon "
          f"pid is recorded at START and escalated at TEARDOWN, a whole "
          f"cell later — so the kernel may have handed that pid to an "
          f"unrelated process, which this script, running as root in the "
          f"container, would then signal. Use "
          f"`kill_owned_pid \"{pid_var}\" \"{id_var}\" \"{comm}\" "
          f"<SIG>` to signal it, or "
          f"`node_still_ours \"{pid_var}\" \"{id_var}\"` to poll it]")
    return 1


_SPAWN_LINE = r"^\s*{role}_pid=\$!\s*$"
_CAPTURE_LINE = r"^\s*{role}_id=\$\(pid_identity \"\${role}_pid\"\)\s*$"


def _identity_capture_reason(runner: str, role: str,
                             pid_var: str) -> "Optional[str]":
    """Is every spawn of this role IMMEDIATELY followed by its capture?

    Returns None when it is, else why not. PURE — it returns the reason
    rather than printing it, so the fixture arm can drive its must-FAIL
    vectors without emitting FAIL lines a reader would count as real.

    Without a capture the ladder arm is satisfied by a script whose
    signals are all no-ops: `kill_owned_pid` returns 1 on an empty
    `want_id` before it signals anything, so a missing capture turns the
    whole graceful ladder off silently and the reap degrades to the
    `pkill` backstop — the ungraceful kill the ladder exists to replace,
    with the receipt lost exactly as before.

    BOUND to the spawn, not COUNTED against it. The first version of this
    check compared file-wide totals, and a total is not a binding: MOVING
    a capture out of one `RECV_PATH` branch and duplicating it in another
    keeps the count equal and leaves a branch spawning a child nothing can
    signal. That was built and RUN against this checker, which reported
    237 ok / 0 FAIL while the DEFAULT `rclcpp` path had no `latency_id` at
    all. An arm satisfiable by the script it exists to reject is worse
    than no arm, because the green line is read as evidence.

    IMMEDIATELY is the contract, not merely the checkable approximation:
    the identity is only valid while the pid is still ours, so ANY
    statement between `$!` and the capture is a window in which the child
    can exit and the kernel recycle the pid — and the capture would then
    record a STRANGER's start time, which is worse than no capture at
    all, since every rung would pass the identity gate and signal the
    wrong process."""
    lines = runner.split("\n")
    role_name = pid_var[1:-4]                    # `$latency_pid` -> `latency`
    spawn = re.compile(_SPAWN_LINE.format(role=role_name))
    capture = re.compile(_CAPTURE_LINE.format(role=role_name))
    spawns = [i for i, line in enumerate(lines) if spawn.match(line)]
    if not spawns:
        return (f"run_bench.sh has no `{role_name}_pid=$!` spawn at all — "
                f"this check cannot bind captures to spawns, so it fails "
                f"closed rather than reporting the {role} role clean")
    unbound = [i + 1 for i in spawns
               if not (i + 1 < len(lines) and capture.match(lines[i + 1]))]
    if unbound:
        return (f"the {role} role is spawned at line(s) {unbound} without "
                f"`{role_name}_id=$(pid_identity \"{pid_var}\")` on the "
                f"very next line. `kill_owned_pid` refuses an EMPTY "
                f"want_id before it signals anything, so that branch's "
                f"rungs are all silent no-ops: the node is never asked to "
                f"stop gracefully, the reap falls through to the "
                f"`pkill -KILL` backstop, and the DELIVERY receipt is lost "
                f"exactly as it was before the ladder existed. Counting "
                f"captures file-wide does NOT catch this — moving one out "
                f"of a RECV_PATH branch and duplicating it in another "
                f"keeps the total equal — which is why each spawn is bound "
                f"to the line after it")
    return None


def _daemon_capture_reason(runner: str, pid_var: str,
                           id_var: str) -> "Optional[str]":
    """Is every assignment of this DAEMON pid immediately followed by its
    identity capture? Returns None when it is, else why not. PURE.

    Without the capture the identity is `""` for the run, and BOTH halves
    of the gate turn themselves off silently: `node_still_ours` falls back
    to a bare `kill -0` (the pre-fix behaviour, at every poll), and
    `kill_owned_pid` refuses an empty identity BEFORE signalling — so the
    daemon is never signalled at teardown at all. Measured on the shipped
    tree: deleting one line reverted six call sites and left every arm in
    this file green.

    The node roles have their own binding check (`_identity_capture_reason`),
    which keys on `<role>_pid=$!`. A daemon's pid does not always come from
    `$!` — the zenoh router's comes from `zenohd_live_pid` — so this asks
    the more general question: wherever the pid is SET to anything other
    than empty, the very next line records the identity of that pid."""
    pid_name, id_name = pid_var.lstrip("$"), id_var.lstrip("$")
    lines = runner.split("\n")
    assign = re.compile(r"^\s*" + pid_name + r"=(.*)$")
    capture = re.compile(r"^\s*" + id_name + r'="?\$\(pid_identity '
                         r'"?\$\{?' + pid_name + r'\}?"?\)"?\s*$')
    # The empty-value test is done in PYTHON, not as a lookahead: written
    # as one it needed a `""` inside a regex inside a generated literal,
    # and the quoting collapsed it into an alternative that matched the
    # empty string — so the lookahead was always satisfied and the
    # pattern matched NOTHING. The arm's own vectors caught it, which is
    # what the "must be BOUND" half is for.
    at = []
    for i, ln in enumerate(lines):
        m = assign.match(ln)
        if m is None:
            continue
        if m.group(1).strip() in ('""', "''", ""):
            continue          # CLEARING the pid; no identity to record
        at.append(i)
    if not at:
        return (f"run_bench.sh never assigns {pid_var} a pid at all — this "
                f"check cannot bind a capture to an assignment, so it "
                f"fails closed rather than reporting the daemon gated")
    unbound = [i + 1 for i in at
               if not (i + 1 < len(lines) and capture.match(lines[i + 1]))]
    if unbound:
        return (f"{pid_var} is assigned at line(s) {unbound} without "
                f"`{id_var}=\"$(pid_identity \"{pid_var}\")\"` on the very "
                f"next line. The identity then stays empty, "
                f"`node_still_ours` degrades to the bare `kill -0` this "
                f"change removed at every poll, and `kill_owned_pid` "
                f"refuses an empty identity BEFORE signalling — so the "
                f"daemon is never signalled at teardown while every "
                f"spelling in the file still reads as identity-gated")
    return None


def _identity_capture_failures(runner: str, role: str, pid_var: str) -> int:
    """The reporting half of `_identity_capture_reason`."""
    why = _identity_capture_reason(runner, role, pid_var)
    if why is None:
        return 0
    print(f"FAIL  [signal ladder: {why}]")
    return 1


def _signal_ladder_failures(runner: str, role: str, pid_var: str,
                            rung_sigs: "Tuple[str, ...]") -> int:
    """Does `run_bench.sh` climb the signal ladder for ONE role?

    Three claims, and the first alone is what this arm used to make for
    ping and pong: that the tokens appear. Presence is not the contract —
    a KILL with a decoy INT somewhere earlier satisfies "INT appears"
    while eating the receipt just the same, and a rung with no grace after
    it is decoration, because the next rung lands before the node can act
    on the first.

      (i)   every rung is sent to THIS role's pid;
      (ii)  in order — INT before TERM before KILL;
      (iii) a bounded wait between each rung and the next — parsed as a
            LOOP, not as co-occurring tokens: a numeric ceiling in its
            condition, a `kill -0` on THIS role's pid inside it, and a
            `sleep` in its body. See `_grace_loop_failure`.

    INT first is MEASURED, not assumed: on the jazzy bench image these
    binaries call plain `rclcpp::init(argc, argv)`, which installs a
    SIGINT handler and nothing else — `kill -INT` gives rc 2 with the
    DELIVERY receipt printed, `kill -TERM` gives rc 143 with none, and
    `ping_node` under TERM likewise exits 143 with no `DELIVERY role=ping`
    line.
    """
    found = [(sig, runner.find(_owned_rung(pid_var, sig)))
             for sig in rung_sigs]
    missing = [sig for sig, at in found if at < 0]
    if missing:
        print(f"FAIL  [signal ladder: run_bench.sh never sends {missing} to "
              f"{pid_var}. Measured on the bench image: these binaries "
              f"install a handler for SIGINT ONLY (plain rclcpp::init), so "
              f"INT yields rc 2 WITH the receipt and TERM yields rc 143 "
              f"with none. Without the INT rung the {role} role's "
              f"timeout-path receipt is lost — which for the latency sink "
              f"is the ONE shape the stamp-gate counters exist for]")
        return 1
    ordered = [sig for sig, _ in sorted(found, key=lambda r: r[1])]
    if ordered != list(rung_sigs):
        print(f"FAIL  [signal ladder: the {role} rungs are sent in the order "
              f"{', '.join(ordered)} — it must be "
              f"{', '.join(rung_sigs)}. A harsher signal ahead of the "
              f"graceful ones eats the receipt however many follow it]")
        return 1
    failures = 0
    for (sig_a, at_a), (sig_b, at_b) in zip(found, found[1:]):
        why = _grace_loop_failure(runner[at_a:at_b], pid_var, role,
                                  sig_a, sig_b)
        if why is not None:
            print(f"FAIL  [signal ladder: {why}. A signal with no grace "
                  f"after it is decoration: the next rung lands before the "
                  f"node can act on the first, so the ladder reduces to "
                  f"its last rung]")
            failures += 1
    return failures


def check_sample_gate_accounting() -> int:
    """An echo that yields no sample must be COUNTED and REPORTED, not
    discarded in silence.

    All three latency sinks held the same guard inline —
    `if (send_ns == 0 || now_ns <= send_ns) { return; }` — and dropped the
    echo with no counter and no line. That is a round trip the chain
    really completed and the cell does not count, so the run needs more
    echoes than it reports asking for; and a cell whose stamps are ALL
    unusable never reaches finalize at all, so nothing is ever printed
    about why it timed out.

    Two conditions, kept APART because their remedies differ: `send_ns ==
    0` is a publisher that did not stamp (a wiring fault), while `now_ns
    <= send_ns` is a duplicate or non-monotone stamp (clock behaviour) —
    and the subtraction is UNSIGNED, so that second one would wrap to
    ~1.8e19 ns rather than merely mislead.

    Three claims, and fixing any one alone fixes nothing:

    (a) the DECISION — a compiled hand oracle
        (ros2/ros2_rtt_bench/test/sample_gate_oracle.cpp) over the
        shipping, ROS-free sample_gate.hpp. Every expectation is a
        `static_assert`, so the COMPILER is the runner: no ROS distro, no
        binary to execute.

    (b) the CALL SITES — that each sink routes through that decision,
        bumps its OWN counter on each drop arm, BAILS on both, and prints
        both counts on its `DELIVERY role=latency` line. Asserted over the
        shipped text, because reaching the real path needs a ROS 2
        container.

    (c) the CLASS — that no fourth copy of the inline guard survives
        anywhere in the sink sources."""
    errors = 0

    # ---- (a0) the arm inspector must REPORT, never crash ----------------
    # `_drop_arm_failures` is the only thing standing between a regressed
    # sink and a green run, so it has to survive the hostile shapes it is
    # meant to describe. The inline version did not: an arm whose body is
    # EMPTY — a fall-through arm, or one holding only a comment, which
    # `_cxx_code_only` blanks to nothing — made it index `[0]` into an
    # empty line list and raise IndexError, so the checker died with a
    # traceback and every arm after it in main() was skipped. A checker
    # that crashes on the exact regression it exists to report is worse
    # than one that misses it: the crash reads as a broken tool.
    #
    # Driven as a PURE function over synthetic bodies, so these shapes
    # need no sink on disk and cannot perturb the real tree.
    _HOSTILE_BODIES = (
        ("", "an EMPTY body (a fall-through arm)"),
        ("\n\n   \n", "whitespace only (what a comment-only arm blanks to)"),
        ("\n        break;\n", "a bare bail, no counter and no log"),
    )
    for hostile, what in _HOSTILE_BODIES:
        try:
            got = _drop_arm_failures("fixture.cpp", "kUnstamped",
                                     "unstamped_", "nonpositive_rtt_",
                                     "NO stamp", hostile, "return")
        except Exception as e:                        # noqa: BLE001
            print(f"FAIL  [sample gate arm inspector: {what} made "
                  f"_drop_arm_failures raise {type(e).__name__} ({e}) "
                  f"instead of describing it — the checker would die with "
                  f"a traceback on the very shape this arm exists to "
                  f"report, taking every later check with it]")
            errors += 1
            continue
        if not got:
            print(f"FAIL  [sample gate arm inspector: {what} was reported "
                  f"as CLEAN — an arm that neither counts nor logs nor "
                  f"bails is the silent drop this change removes]")
            errors += 1
        elif not any("does not OPEN with `++unstamped_;`" in m for m in got):
            print(f"FAIL  [sample gate arm inspector: {what} did not "
                  f"produce the missing-counter failure; got {got!r}]")
            errors += 1
    # ...and the anti-tautology half: a WELL-FORMED arm must come back
    # clean, or every verdict above is satisfied by an inspector that
    # complains about everything.
    healthy = ("\n        ++unstamped_;\n"
               "        if (unstamped_ == 1) {\n"
               "          RCLCPP_WARN(this->get_logger(), \"NO stamp\");\n"
               "        }\n"
               "        return;\n")
    healthy_got = _drop_arm_failures("fixture.cpp", "kUnstamped",
                                     "unstamped_", "nonpositive_rtt_",
                                     "NO stamp", healthy, "return")
    if healthy_got:
        print(f"FAIL  [sample gate arm inspector: a well-formed arm was "
              f"reported as broken — {healthy_got!r}. Every hostile-shape "
              f"verdict above is vacuous against an inspector that "
              f"complains about everything]")
        errors += 1

    # ---- (b) the call sites. No compiler needed, so it runs first. ----
    for fname, opener, bail, after_expr, run_fn, spin_fn in _SAMPLE_GATE_SINKS:
        path = _ROS2_BENCH_SRC / "src" / fname
        try:
            text = _cxx_code_only(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError) as e:
            print(f"FAIL  [sample gate call site: cannot read {fname}: {e}]")
            errors += 1
            continue
        if '#include "sample_gate.hpp"' not in text:
            print(f"FAIL  [sample gate call site: {fname} does not include "
                  f"sample_gate.hpp — whatever it judges with is a second "
                  f"copy of the decision]")
            errors += 1
        try:
            judged = _extract_cxx_block(text, opener, f"sample gate/{fname}")
            report = _extract_cxx_block(text, "  void report_delivery()",
                                        f"sample gate/{fname}")
            final = _extract_cxx_block(text, "  void finalize()",
                                       f"sample gate/{fname}")
        except RuntimeError as e:
            print(f"FAIL  [sample gate call site: {e}]")
            errors += 1
            continue

        # TOLERANT of spacing, exact about the CALL. A literal `find` on one
        # spelling turns a reformat into a FALSE failure (safe direction,
        # but noise); a loose `classify_stamp_pair` search would accept the
        # call made somewhere other than the switch head.
        sw = re.search(r"switch\s*\(\s*classify_stamp_pair\s*\(\s*send_ns\s*,"
                       r"\s*now_ns\s*\)\s*\)", judged)
        if sw is None:
            print(f"FAIL  [sample gate call site: {fname} does not switch "
                  f"on classify_stamp_pair(send_ns, now_ns) — an inline "
                  f"condition here is the silent drop this change "
                  f"removes, and the compiled oracle says nothing about "
                  f"it]")
            errors += 1
            continue
        # BRACE-COUNTED, not a regex to the next `case`. Each drop arm
        # holds a log-once `if (...) { ... }` of its own, and a lookahead
        # for `\n}` ends the arm at THAT closing brace — before the
        # `return;`: a regex to the next `case` instead reports all six arms
        # missing their bail while every one of them has it. So: take the
        # switch's own balanced block, then split it on the case labels.
        try:
            sw_body = _brace_block_at(judged, sw.start(),
                                      f"sample gate/{fname}")
        except RuntimeError as e:
            print(f"FAIL  [sample gate call site: {e}]")
            errors += 1
            continue
        arms = {}
        labels = list(re.finditer(r"case\s+StampVerdict::(\w+)\s*:", sw_body))
        for n, m in enumerate(labels):
            end = (labels[n + 1].start() if n + 1 < len(labels)
                   else sw_body.rfind("}"))
            arms[m.group(1)] = sw_body[m.end():end]

        # NO `default:`. The enum is closed and the sinks rely on -Wswitch
        # to name a call site that has not grown an arm for a new verdict;
        # a `default:` label silences that warning permanently and routes
        # the unhandled verdict into whatever it happens to do.
        if re.search(r"\bdefault\s*:", sw_body):
            print(f"FAIL  [sample gate call site: {fname}'s switch carries "
                  f"a `default:` label — that silences the -Wswitch warning "
                  f"sample_gate.hpp relies on, so a verdict added later "
                  f"reaches this sink with no counter and nothing says so]")
            errors += 1

        # Each drop arm: its OWN counter FIRST, the sibling's counter
        # ABSENT, a LOG-ONCE guarded by that counter, and a BAIL.
        for verdict, counter, sibling, verdict_phrase in _DROP_ARMS:
            for msg in _drop_arm_failures(
                    fname, verdict, counter, sibling, verdict_phrase,
                    arms.get(verdict), bail):
                print(msg)
                errors += 1

        # The USABLE arm: it must TAKE the sample, and it must not bail.
        # The push lives INSIDE the arm on purpose — after the switch it is
        # reached by a verdict nobody wrote an arm for (the colcon build
        # has no -Werror, so -Wswitch is only a warning), and that records
        # the unsigned subtraction this whole gate exists to prevent.
        usable = arms.get("kUsable")
        if usable is None:
            print(f"FAIL  [sample gate call site: {fname} has no "
                  f"`case StampVerdict::kUsable:` arm]")
            errors += 1
        else:
            if "samples_.push_back(now_ns - send_ns);" not in usable:
                print(f"FAIL  [sample gate call site: {fname} takes the "
                      f"sample OUTSIDE the kUsable arm — a verdict added "
                      f"later with no arm here falls through the switch "
                      f"and records `now_ns - send_ns` unsigned, which is "
                      f"the wrap the gate exists to prevent. Move the "
                      f"push inside the arm]")
                errors += 1
            if re.search(r"\b" + bail + r"\s*;", usable):
                print(f"FAIL  [sample gate call site: {fname}'s kUsable arm "
                      f"`{bail};`s — that drops every sample, and the cell "
                      f"would time out with full counters and an empty "
                      f".bin]")
                errors += 1

        # ---- the REPORT ------------------------------------------------
        # POSITIONAL, because every presence check passes a swap: a format
        # string naming each key beside an argument list passing each
        # counter still reports every count under its neighbour's name.
        try:
            keys, args = _delivery_printf_pairs(report,
                                                f"sample gate/{fname}")
        except RuntimeError as e:
            print(f"FAIL  [sample gate report: {e}]")
            errors += 1
            keys, args = [], []
        if not keys:
            # NOT a skip. Every counter on that line is printed with `%zu`
            # by convention, and that convention is what lets this arm
            # check the format against the ARGUMENT LIST positionally.
            # With no key matched, everything below is inert and the arm's
            # green would mean nothing — measured: changing `%zu` to `%llu`
            # throughout scores errors = 0.
            print(f"FAIL  [sample gate report: {fname}'s DELIVERY line "
                  f"carries no `<key>=%zu` at all — nothing below this "
                  f"point can check anything, so this arm fails rather "
                  f"than reporting a green it has not earned]")
            errors += 1
        else:
            want = [k + "_" for k in keys]
            if args != want:
                print(f"FAIL  [sample gate report: {fname}'s DELIVERY line "
                      f"prints {keys} but passes {args} — each key must be "
                      f"fed by the member of the same name (expected "
                      f"{want}). A swap reports every count under its "
                      f"neighbour's label and no presence check can see "
                      f"it]")
                errors += 1
            for needed in ("unstamped", "nonpositive_rtt"):
                if needed not in keys:
                    print(f"FAIL  [sample gate report: {fname}'s DELIVERY "
                          f"line does not carry `{needed}=` — the counter "
                          f"exists and nothing reads it]")
                    errors += 1

        # ...and the receipt must be REACHABLE on a cell that never
        # finalizes, which is the shape the counters exist for: finalize()
        # runs only once the full sample population is collected, and it is
        # finalize() that ends the spin. At-most-once, so the healthy path
        # still prints exactly one line.
        if not re.search(r"if\s*\(\s*delivery_reported_\s*\)", report):
            print(f"FAIL  [sample gate report: {fname}'s report_delivery "
                  f"has no `if (delivery_reported_)` at-most-once guard — "
                  f"with two call sites the receipt would print twice]")
            errors += 1
        if "report_delivery();" not in final:
            print(f"FAIL  [sample gate report: {fname}'s finalize does not "
                  f"call report_delivery() — the healthy path prints no "
                  f"receipt at all]")
            errors += 1
        # POSITIONAL, not a count. The second site has to sit AFTER the
        # spin, or it does not do the job the split was made for.
        # Scoped to the function that CONTAINS the spin, not the whole
        # file: `while (rclcpp::ok() && !done_) {` occurs twice in the rcl
        # sink and a file-wide `find` matched the OUTER loop head, above
        # every call site, making this arm vacuous for that sink.
        try:
            spin_body = _extract_cxx_block(text, spin_fn,
                                           f"sample gate/{fname}")
        except RuntimeError as e:
            print(f"FAIL  [sample gate report: {e}]")
            errors += 1
            continue
        # TWO scopes, because these are two different claims and mixing
        # them is what made the positional half vacuous:
        #   file_sites — "a second call site exists at all"
        #   spin_sites — "one of them is reached AFTER the spin", which
        #                only means anything within the function that
        #                CONTAINS the spin.
        file_sites = re.findall(r"(?<!void )report_delivery\s*\(\s*\)", text)
        spin_sites = [m.start() for m in
                      re.finditer(r"(?<!void )report_delivery\s*\(\s*\)",
                                  spin_body)]
        spin_at = spin_body.find(after_expr)
        if len(file_sites) < 2:
            print(f"FAIL  [sample gate report: {fname} calls "
                  f"report_delivery() from {len(file_sites)} place(s) — it "
                  f"needs a SECOND site reached when the run ends WITHOUT "
                  f"finalizing, or a cell whose stamps are all unusable "
                  f"dies with its counters unprinted, which is exactly the "
                  f"shape they exist for]")
            errors += 1
        elif spin_at < 0:
            print(f"FAIL  [sample gate report: {fname} no longer contains "
                  f"`{after_expr}` — this arm anchors the second "
                  f"report_delivery() call on it and cannot place the call "
                  f"without it]")
            errors += 1
        elif not any(c > spin_at for c in spin_sites):
            print(f"FAIL  [sample gate report: no report_delivery() call in "
                  f"{fname}'s {spin_fn} comes AFTER `{after_expr}`. A call placed "
                  f"ahead of the spin latches delivery_reported_ at "
                  f"start-up, so finalize's call is a no-op and EVERY cell "
                  f"reports all-zero counts; and two calls inside "
                  f"finalize() leave the receipt as unreachable as it was "
                  f"before the split]")
            errors += 1

        # ...and the receipt's printf must be REACHED once the guard is
        # passed. A bare `return;` between the flag and the fprintf leaves
        # the whole line dead while every text check above still matches
        # (`-Wunreachable-code` is in neither -Wall nor -Wextra).
        after_flag = report.split("delivery_reported_ = true;", 1)
        if len(after_flag) == 2 and re.search(r"^\s*return\s*;", after_flag[1],
                                              re.M):
            body_to_print = after_flag[1].split("std::fprintf(", 1)[0]
            if re.search(r"^\s*return\s*;", body_to_print, re.M):
                print(f"FAIL  [sample gate report: {fname}'s "
                      f"report_delivery() returns before its fprintf — the "
                      f"receipt is dead code on every path, and the format "
                      f"string is still there for a text check to find]")
                errors += 1

    # ---- (b1) the sink SET is derived, not remembered ----------------
    # _SAMPLE_GATE_SINKS is a hand list of three, and every per-sink arm
    # above iterates it — so a FOURTH sink that includes the header,
    # switches on the shared decision and counts nothing at all would be
    # covered by no arm in this function while passing the class sweep
    # below (it does reach classify_stamp_pair). Derive the truth from the
    # include and require the table to match it exactly.
    try:
        # The SAME extension set the class sweep below uses. Globbing
        # `*.*pp` here reintroduced exactly the hole this arm's own
        # comment says it closes: a fourth sink added as .cc/.cxx/.h was
        # invisible to the derivation while the sweep did see it.
        including = sorted(
            path.name for path in _sink_candidate_files()
            if '#include "sample_gate.hpp"' in
            _cxx_code_only(path.read_text(encoding="utf-8")))
    except (OSError, UnicodeDecodeError) as e:
        print(f"FAIL  [sample gate sink set: cannot enumerate the sinks "
              f"({e}) — this arm fails closed rather than trusting the "
              f"hand table]")
        errors += 1
    else:
        tabled = sorted(f for f, _o, _b, _a, _r, _s in _SAMPLE_GATE_SINKS)
        if including != tabled:
            print(f"FAIL  [sample gate sink set: the files including "
                  f"sample_gate.hpp are {including} but _SAMPLE_GATE_SINKS "
                  f"lists {tabled}. Every per-sink arm iterates the TABLE, "
                  f"so a sink missing from it is judged by nothing — add "
                  f"it (with its bail and its post-spin anchor) rather "
                  f"than leaving the list to go stale]")
            errors += 1

    # ---- (b1b) EVERY post-construction exit reports -----------------
    # The receipt is only "reachable" if every way OUT of the run function
    # goes through it. A cell can die after the node is constructed and
    # before its spin is entered — a bootstrap-discovery timeout, a
    # SIGTERM landing mid-bootstrap (run_bench.sh's watchdog guarantees
    # TERM before KILL, not that the spin was ever reached), a pacing
    # start losing the same race — and on every one of those the sink used
    # to vanish without a word while this PR claimed the receipt was
    # reachable. A CLASS sweep, not a list of the sites that exist today:
    # any new early exit added below the constructor is caught the day it
    # lands.
    for fname, _o, _b, _a, run_fn, _s in _SAMPLE_GATE_SINKS:
        path = _ROS2_BENCH_SRC / "src" / fname
        try:
            code = _cxx_code_only(path.read_text(encoding="utf-8"))
            run_body = _extract_cxx_block(code, run_fn,
                                          f"sample gate/{fname}")
        except (OSError, UnicodeDecodeError, RuntimeError) as e:
            print(f"FAIL  [sample gate bootstrap exits: {fname}: {e} — this "
                  f"arm fails closed rather than assuming every exit "
                  f"reports]")
            errors += 1
            continue
        made = run_body.find("std::make_shared")
        if made < 0:
            print(f"FAIL  [sample gate bootstrap exits: {run_fn} in {fname} "
                  f"constructs no node — this arm anchors on the "
                  f"constructor and cannot tell a pre- from a "
                  f"post-construction exit without it]")
            errors += 1
            continue
        after = run_body[made:]
        spin_at = after.find(_a)
        unreported = []
        # STRUCTURAL, not line-anchored. Two shapes walk past a
        # line-anchored pattern:
        #   - an INLINE exit, `if (!rclcpp::ok()) return 2;`, because the
        #     pattern required `return` at the start of a line — so a
        #     reformat alone reintroduced the hole silently;
        #   - an EXPLICIT `throw` or `exit()`, which the pattern did not
        #     look for at all.
        # Each exit must carry its OWN receipt: the window searched is
        # from the END of the previous exit (or the constructor, for the
        # first) to this one, so no exit can borrow the receipt of
        # another. A backward character window was the earlier rule and a
        # `return 9;` inserted above the spin inherited the pacing arm's
        # call through it.
        exits = [m for m in re.finditer(
            r"\b(return\b[^;]*|throw\b[^;]*|(?:std::)?exit\s*\([^;]*)\;",
            after)]
        prev_end = 0
        for m in exits:
            expr = m.group(1)
            # A return that DELEGATES to the drain loop is not an early
            # exit: run() prints the receipt at its own tail.
            if "->run()" in expr:
                prev_end = m.end()
                continue
            # Nor is the HEALTHY tail: a return reached THROUGH the spin,
            # with the receipt already emitted between the two. That path
            # has its own pin (the positional call-site arm above), and
            # the composed sink prints ping's and pong's lines between its
            # receipt and its `return 0`.
            if (spin_at >= 0 and m.start() > spin_at
                    and "report_delivery()" in after[spin_at:m.start()]):
                prev_end = m.end()
                continue
            if "report_delivery()" not in after[prev_end:m.start()]:
                entry = (fname, expr.strip())
                if entry in _DECLARED_UNRECEIPTED_EXITS:
                    # DECLARED, not exempt-by-shape: it is written down
                    # with its reason below, and anything NOT on that list
                    # still fails. Silently skipping the shape would hide
                    # the next one too.
                    prev_end = m.end()
                    continue
                unreported.append(
                    (after[:m.start()].count("\n") + 1, expr.strip()[:34]))
            prev_end = m.end()
        if unreported:
            print(f"FAIL  [sample gate bootstrap exits: {fname}'s {run_fn} "
                  f"exits at (relative) line/expr {unreported} AFTER the "
                  f"node is constructed without calling report_delivery() "
                  f"first. A cell that dies between construction and the "
                  f"spin — a bootstrap timeout, or a SIGTERM landing "
                  f"mid-bootstrap — then produces NO receipt at all, and "
                  f"the counters this change adds are exactly what such a "
                  f"cell has to say. Zeros are the right answer there: an "
                  f"explicit zero receipt says the sink came up and saw "
                  f"nothing, which \"no DELIVERY receipts at all\" does "
                  f"not]")
            errors += 1

    # ---- (b1c) the finite-bound predicate, over hand vectors ---------
    # `_loop_counter_bounded` is what stands between a hanging reap and a
    # green checker, and it has now been wrong twice in the same way:
    # first it read `-gt 0` as a ceiling (a FLOOR, one character from a
    # bound), then it read the ceiling and the increment without reading
    # the condition they sit in, so `[ "$n" -lt "$MAX" ] || true` passed
    # with both halves intact and looped forever. Both were bypasses of a
    # check whose whole job is to refuse an unbounded wait, so the shapes
    # below are deliberately ADVERSARIAL rather than representative: the
    # real script has two loop spellings and neither is hostile, so every
    # hostile one has to be supplied by hand.
    for label, cond, body, want_ok in (
            # -- must PASS: the two real spellings, and the forms the
            #    grammar is meant to keep accepting.
            ("the real latency loop",
             '[ "$lat_grace" -lt 20 ] &&\n'
             '        node_still_ours "$latency_pid" "$latency_id"',
             "sleep 0.1; lat_grace=$((lat_grace + 1))", True),
            ("the real ping/pong loop", '[ "$grace" -lt 20 ]',
             "sleep 0.1; grace=$((grace + 1))", True),
            ("the -le spelling", '[ "$n" -le 20 ]', "n=$((n + 1))", True),
            ("a MIRRORED ceiling (20 -gt $n)", '[ 20 -gt "$n" ]',
             "n=$((n + 1))", True),
            ("a NAMED ceiling", '[ "$n" -lt "$MAX" ]', "n=$((n + 1))", True),
            ("the ((n++)) increment form", '[ "$n" -lt 20 ]',
             "((n++))", True),
            # Conjunction is SAFE and must stay accepted — three tests
            # `&&`-joined, none of which can outlive the ceiling. Without
            # this vector the grammar could be tightened to "exactly one
            # test" and the real latency loop would be the only thing
            # noticing.
            ("three &&-joined tests",
             '[ "$n" -lt 20 ] && node_still_ours "$probe_pid" "$probe_id"'
             ' && [ -n "$probe_pid" ]',
             "sleep 0.1; n=$((n + 1))", True),
            # The `!` refusal is scoped to the CONDITION. The real
            # ping/pong loop negates inside its body and must not be
            # caught by it.
            ("a negation in the BODY, not the condition",
             '[ "$n" -lt 20 ]',
             'if ! node_still_ours "$probe_pid" "$probe_id"; then break; fi\n'
             "sleep 0.1; n=$((n + 1))", True),
            # -- must FAIL: the condition's logical structure.
            ("the reported bypass: a ceiling ORed with true",
             '[ "$n" -lt "$MAX" ] || true',
             "sleep 0.1; n=$((n + 1))", False),
            ("a disjunction of two REAL tests",
             '[ "$n" -lt 20 ] || [ -f /tmp/keep-going ]',
             "sleep 0.1; n=$((n + 1))", False),
            ("a negated test", '! [ "$n" -ge 20 ]',
             "sleep 0.1; n=$((n + 1))", False),
            ("test's own -o disjunction",
             '[ "$n" -lt 20 -o -f /tmp/keep-going ]',
             "sleep 0.1; n=$((n + 1))", False),
            ("a non-test conjunct", '[ "$n" -lt 20 ] && true',
             "sleep 0.1; n=$((n + 1))", False),
            # bash takes the LAST command's status from a `;` list, so
            # this is always true — and it is why the conjunct pattern is
            # anchored at both ends.
            ("a ; list whose last command always succeeds",
             '[ "$n" -lt 20 ]; true', "sleep 0.1; n=$((n + 1))", False),
            ("a ; list of two REAL tests",
             '[ "$n" -lt 20 ]; [ -f /tmp/keep-going ]',
             "sleep 0.1; n=$((n + 1))", False),
            ("a pipeline headed by a real test",
             '[ "$n" -lt 20 ] | grep -q .',
             "sleep 0.1; n=$((n + 1))", False),
            ("a backgrounded test", '[ "$n" -lt 20 ] &',
             "sleep 0.1; n=$((n + 1))", False),
            ("an arithmetic condition", "(( n < MAX ))",
             "sleep 0.1; n=$((n + 1))", False),
            ("arithmetic with a stray || :", "(( n < MAX )) || :",
             "sleep 0.1; n=$((n + 1))", False),
            ("a ceiling read out of a command substitution",
             '[ "$n" -lt $(cat /tmp/max) ]', "n=$((n + 1))", False),
            # -- must FAIL: the comparison itself.
            ("a FLOOR, not a ceiling (-gt 0) with an increment",
             '[ "$grace" -gt 0 ]', "sleep 0.1; grace=$((grace + 1))", False),
            ("a -ne 0 floor with an increment", '[ "$n" -ne 0 ]',
             "n=$((n + 1))", False),
            ("the mirrored spelling reversed (20 -lt $n)",
             '[ 20 -lt "$n" ]', "n=$((n + 1))", False),
            # -- must FAIL: the counter and the body.
            ("a ceiling nothing moves toward", '[ "$n" -lt 20 ]',
             "sleep 0.1", False),
            ("an increment of the WRONG variable", '[ "$n" -lt 20 ]',
             "m=$((m + 1))", False),
            ("a ceiling on the WRONG variable", '[ "$m" -lt 20 ]',
             "sleep 0.1; n=$((n + 1))", False),
            ("a DECREMENT under a rising ceiling", '[ "$n" -lt 20 ]',
             "sleep 0.1; n=$((n - 1))", False),
            ("no bound at all (while true)", "true",
             "sleep 0.1; n=$((n + 1))", False),
    ):
        got_ok = _loop_counter_bounded(cond, body) is None
        if got_ok != want_ok:
            print(f"FAIL  [grace bound oracle: {label} was judged "
                  f"{'BOUNDED' if got_ok else 'UNBOUNDED'} and must be "
                  f"{'BOUNDED' if want_ok else 'UNBOUNDED'}. A floor "
                  f"(`-gt 0`) is not a ceiling, a ceiling nothing moves "
                  f"toward is not a bound, and a ceiling under a condition "
                  f"that is true regardless (`|| true`) is not a bound "
                  f"either — each mistake lets the checker approve a reap "
                  f"that hangs]")
            errors += 1

    # ---- (b1d) the same predicate at SLICE level --------------------
    # Two bypasses live above the condition rather than inside it, so no
    # (cond, body) pair can express them: a bounded loop NESTED inside an
    # unbounded one (the wait still never ends, but the inner loop looks
    # perfect), and an `until` loop, which is not merely un-parsed — it
    # INVERTS the sense, so the same `[ "$g" -lt 20 ]` bound would exit
    # immediately instead of waiting. Both are driven through
    # `_grace_loop_failure`, which is the function that actually decides.
    for label, between, want_ok in (
            ("the real latency slice",
             'while [ "$g" -lt 20 ] && node_still_ours "$probe_pid" "$probe_id"; do\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", True),
            ("the real ping/pong slice (poll inside a nested if)",
             'while [ "$g" -lt 20 ]; do\n'
             '    if ! node_still_ours "$probe_pid" "$probe_id"; then break; fi\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", True),
            ("a bounded loop nested inside an unbounded one",
             "while true; do\n"
             '    while [ "$g" -lt 20 ] && node_still_ours "$probe_pid" "$probe_id"; do\n'
             "        sleep 0.1\n        g=$((g + 1))\n    done\n"
             "done\n", False),
            ("an until loop carrying the same bound",
             'until [ "$g" -ge 20 ]; do\n'
             '    node_still_ours "$probe_pid" "$probe_id" || break\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", False),
            ("the poll hoisted above the loop",
             'node_still_ours "$probe_pid" "$probe_id"\n'
             'while [ "$g" -lt 20 ]; do\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", False),
            ("the sleep hoisted above the loop",
             "sleep 0.1\n"
             'while [ "$g" -lt 20 ] && node_still_ours "$probe_pid" "$probe_id"; do\n'
             "    g=$((g + 1))\ndone\n", False),
            ("no loop at all, just a sleep",
             'sleep 2\nnode_still_ours "$probe_pid" "$probe_id"\n', False),
            # The IDENTITY half, at slice level: a grace that polls with a
            # bare `kill -0` is watching a PID, not a process, so the rung
            # after it can land on a recycled stranger. Refused.
            ("a grace that polls with a bare kill -0",
             'while [ "$g" -lt 20 ] && kill -0 "$probe_pid" 2>/dev/null; do\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", False),
            # ...and the SAME defect wearing the right function's name. A
            # one-argument `node_still_ours` is not a weaker check, it is
            # the FALLBACK — a bare `kill -0` — so this grace watches a
            # pid rather than a process just as the line above does.
            # Matching the call by its pid prefix accepted it; built and
            # RUN against the real script, the checker reported
            # 237 ok / 0 FAIL with both latency polls stripped this way.
            ("a grace whose poll drops the identity argument",
             'while [ "$g" -lt 20 ] && node_still_ours "$probe_pid"; do\n'
             "    sleep 0.1\n    g=$((g + 1))\ndone\n", False),
    ):
        got_ok = _grace_loop_failure(between, "$probe_pid", "probe",
                                     "INT", "TERM") is None
        if got_ok != want_ok:
            print(f"FAIL  [grace loop oracle: {label} was judged "
                  f"{'A GRACE' if got_ok else 'NOT A GRACE'} and must be "
                  f"{'A GRACE' if want_ok else 'NOT A GRACE'}. A loop "
                  f"nested inside an unbounded one does not bound the "
                  f"wait, and an `until` inverts the bound rather than "
                  f"carrying it]")
            errors += 1

    # ---- (b1e) captures BOUND to spawns, over hand vectors ----------
    # This arm's first version compared file-wide TOTALS, and a total is
    # not a binding: a capture MOVED out of one `RECV_PATH` branch and
    # duplicated in another keeps the count equal while a branch spawns a
    # child nothing can signal. That was not a hypothetical — it was built
    # and RUN against the real script, and this checker reported
    # 237 ok / 0 FAIL while the DEFAULT `rclcpp` path had no `latency_id`
    # at all. The counterexample is a vector here so the contract is
    # pinned by fixtures rather than only by the one script that happens
    # to be correct today.
    for label, text, want_ok in (
            ("a spawn bound to its capture",
             'probe_pid=$!\nprobe_id=$(pid_identity "$probe_pid")\n', True),
            ("two spawns, both bound",
             'probe_pid=$!\nprobe_id=$(pid_identity "$probe_pid")\n'
             'probe_pid=$!\nprobe_id=$(pid_identity "$probe_pid")\n', True),
            ("a spawn with no capture at all", "probe_pid=$!\n", False),
            # THE reported counterexample: totals equal, one spawn naked.
            ("a capture MOVED between branches (totals equal)",
             'probe_pid=$!\n'
             'probe_pid=$!\nprobe_id=$(pid_identity "$probe_pid")\n'
             'probe_id=$(pid_identity "$probe_pid")\n', False),
            ("a capture of the WRONG pid",
             'probe_pid=$!\nprobe_id=$(pid_identity "$other_pid")\n', False),
            # Adjacency is the contract, not the approximation: anything
            # between `$!` and the capture is a window in which the child
            # can exit and the pid be recycled, so the capture would
            # record a STRANGER's start time — worse than no capture,
            # since every rung would then pass the identity gate.
            ("a capture separated from its spawn by a statement",
             'probe_pid=$!\nsleep 0.1\n'
             'probe_id=$(pid_identity "$probe_pid")\n', False),
            ("no spawn at all (must fail CLOSED, not report clean)",
             'echo nothing\n', False),
    ):
        got_ok = _identity_capture_reason(text, "probe",
                                         "$probe_pid") is None
        if got_ok != want_ok:
            print(f"FAIL  [capture binding oracle: {label} was judged "
                  f"{'BOUND' if got_ok else 'UNBOUND'} and must be "
                  f"{'BOUND' if want_ok else 'UNBOUND'}. Counting captures "
                  f"file-wide passes a script whose default receive path "
                  f"spawns a node nothing can signal — an arm satisfiable "
                  f"by the script it exists to reject is worse than no "
                  f"arm, because the green line is read as evidence]")
            errors += 1

    # ---- (b2) the HARNESS half of the report's reachability ---------
    # The second call site only prints if the process is given a signal
    # it can ACT on. run_bench.sh's per-cell watchdog used to send the
    # latency node a straight SIGKILL — uncatchable, so rclcpp's spin
    # never returned and the receipt could not run, while four comments
    # and this arm's own `ok` line claimed it did. The C++ and the shell
    # are one contract; pinned here because nothing else spans them.
    try:
        runner = (_ROS2_BENCH_SRC.parent / "run_bench.sh").read_text(
            encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [sample gate watchdog: cannot read run_bench.sh ({e}) "
              f"— this arm fails closed rather than assuming the harness "
              f"signals gracefully]")
        errors += 1
    else:
        # THREE claims, because the order of two literals is not the
        # contract — a KILL with a decoy INT somewhere earlier satisfies
        # "INT appears before KILL" while eating the receipt just the same.
        #
        # (i) the RUNGS, in order: INT before TERM before KILL. INT first
        #     is MEASURED, not assumed: on the jazzy bench image these
        #     binaries call plain `rclcpp::init(argc, argv)`, which
        #     installs a SIGINT handler and nothing else — `kill -INT`
        #     gives rc 2 with the receipt printed, `kill -TERM` gives
        #     rc 143 with no receipt at all.
        # (ii) a WAIT between each rung and the next. Without it the
        #     signal has no time to be acted on and the ladder is
        #     decoration.
        # (iii) the same ladder on the ping/pong reap, which had the same
        #     defect for the same reason and has never printed its
        #     timeout-path receipts either.
        # ONE validator, three roles. The ping/pong half of this arm used
        # to search only for the INT and TERM tokens — never their ORDER,
        # never a wait between them — so reversing one role's signals or
        # deleting its grace passed, although the receipt depends on INT
        # arriving first and being acted on. That is the same vacuity the
        # latency half already had; sharing the validator is what stops
        # the two halves drifting apart again.
        for role, pid_var, rung_sigs in (
                ("latency", "$latency_pid", ("INT", "TERM", "KILL")),
                # ping and pong climb the SAME three rungs. They used to
                # stop at TERM and be left to the blanket `pkill -KILL -f
                # $BENCH_BIN_PATTERN` — the one reap in that file which
                # proves no ownership, and which now refuses itself where
                # its PID-namespace confinement is unproven (see the
                # backstop arm). A ladder whose last rung is a sweep that
                # can decline is a ladder that can end in a hang, so the
                # KILL rung is addressed to the pid this shell captured at
                # the node's spawn.
                ("ping", "$ping_pid", ("INT", "TERM", "KILL")),
                ("pong", "$pong_pid", ("INT", "TERM", "KILL")),
        ):
            errors += _signal_ladder_failures(runner, role, pid_var,
                                              rung_sigs)
            errors += _identity_capture_failures(runner, role, pid_var)
        # The BARE-kill sweep covers every recorded pid, nodes AND
        # daemons: the class is "a pid signalled or polled without the
        # identity that makes it checkable", and it does not care which
        # process the pid names.
        for role, pid_var, id_var, comm in _IDENTITY_GATED_PIDS:
            errors += _bare_kill_failures(runner, role, pid_var, id_var,
                                          comm)
        # The DAEMON captures. The three node roles are bound by
        # `_identity_capture_failures` above; the daemons were bound by
        # nothing, and an identity that is never captured turns every
        # gate on that pid off in silence.
        for label, text, want_ok in (
                ("an assignment bound to its capture",
                 'D_PID=$!\nD_ID="$(pid_identity "$D_PID")"\n', True),
                ("a pid from a command substitution, bound",
                 'D_PID="$(live_pid)"\nD_ID="$(pid_identity "$D_PID")"\n',
                 True),
                ("two assignments, both bound",
                 'D_PID=$!\nD_ID="$(pid_identity "$D_PID")"\n'
                 'D_PID="$(live_pid)"\nD_ID="$(pid_identity "$D_PID")"\n',
                 True),
                ("CLEARING the pid needs no capture",
                 'D_PID=$!\nD_ID="$(pid_identity "$D_PID")"\nD_PID=""\n',
                 True),
                ("an assignment with NO capture", 'D_PID=$!\n', False),
                ("a capture one statement later — the pid can be recycled "
                 "in that window and the identity would name a stranger",
                 'D_PID=$!\nsleep 0\nD_ID="$(pid_identity "$D_PID")"\n',
                 False),
                ("a capture of the WRONG pid",
                 'D_PID=$!\nD_ID="$(pid_identity "$OTHER_PID")"\n', False),
                ("two assignments, only one bound",
                 'D_PID=$!\nD_ID="$(pid_identity "$D_PID")"\n'
                 'D_PID="$(live_pid)"\n', False),
                ("no assignment at all (fails CLOSED)", 'echo hi\n', False),
        ):
            got_ok = _daemon_capture_reason(text, "$D_PID", "$D_ID") is None
            if got_ok != want_ok:
                print(f"FAIL  [daemon capture oracle: {label} was judged "
                      f"{'BOUND' if got_ok else 'UNBOUND'} and must be "
                      f"{'BOUND' if want_ok else 'UNBOUND'}]")
                errors += 1
        for role, pid_var, id_var, _comm in _IDENTITY_GATED_PIDS:
            if not pid_var[1:].isupper():
                continue               # the node roles, bound above
            why = _daemon_capture_reason(runner, pid_var, id_var)
            if why is not None:
                print(f"FAIL  [signal ladder: {why}]")
                errors += 1

    # ---- (b3) the colcon build's warning flags -----------------------
    # sample_gate.hpp, all three sinks and the no-`default:` arm above all
    # rest on ONE fact about a file none of them is in: the colcon build
    # compiles with `-Wall` (so `-Wswitch` names a call site missing an
    # arm) and WITHOUT `-Werror` (so it is a warning, which is why the
    # push had to move inside the kUsable arm). The oracle's own probe
    # builds a flag list it invents, so it cannot see a change here.
    try:
        cmake = (_ROS2_BENCH_SRC / "CMakeLists.txt").read_text(
            encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [sample gate build flags: cannot read CMakeLists.txt "
              f"({e}) — five comments rest on what it says]")
        errors += 1
    else:
        opts = re.search(r"add_compile_options\(([^)]*)\)", cmake)
        if opts is None:
            print("FAIL  [sample gate build flags: CMakeLists.txt has no "
                  "add_compile_options() — the -Wswitch the enum's "
                  "closedness relies on may not be enabled at all]")
            errors += 1
        else:
            flags = opts.group(1).split()
            if "-Wall" not in flags:
                print(f"FAIL  [sample gate build flags: the colcon build "
                      f"does not pass -Wall ({flags}) — -Wswitch is part "
                      f"of it, and without it a verdict added to "
                      f"StampVerdict reaches a sink with no counter and "
                      f"NOTHING says so]")
                errors += 1
            if any(f in ("-Wno-switch", "-Wno-all") for f in flags):
                print(f"FAIL  [sample gate build flags: the colcon build "
                      f"silences the switch warning ({flags}) — the closed "
                      f"enum's only enforcement at the call sites]")
                errors += 1

    # ---- (c) the class ----------------------------------------------
    # ONE guard, spelled identically in THREE files, each dropping in
    # silence: the duplication IS the finding, so the sweep has to be
    # wider than the one spelling that existed.
    #
    # (i) no COMPARISON of the pair anywhere but the header. The original
    # form was `if (send_ns == 0 || now_ns <= send_ns)`, and a sweep keyed
    # on that ordering is walked straight past by the same guard written
    # `if (now_ns <= send_ns || send_ns == 0)`. The sinks otherwise only
    # SUBTRACT the two and pass them as log arguments, so ANY comparison
    # of them outside sample_gate.hpp is a second decision.
    #
    # (ii) no sink judging the pair without the shared decision at all —
    # a FOURTH sink, or one rewritten around a helper of its own, would
    # satisfy (i) while re-opening the finding. Every file that mentions
    # `send_ns` must reach `classify_stamp_pair`.
    # NB `_cxx_code_only` blanks comments but KEEPS string literals, so
    # this pattern also reads WARN text. Safe as written — the shipped
    # messages spell `send_ns=0` with a single `=` and never write the
    # comparison out — but a future message quoting "now_ns <= send_ns"
    # in prose would false-FAIL this sweep. Fail-safe direction; noted so
    # the next author does not chase a phantom second decision.
    compare_re = re.compile(
        r"send_ns\s*==\s*0"
        r"|now_ns\s*<=?\s*send_ns"
        r"|send_ns\s*>=?\s*now_ns")
    strays, unjudged = [], []
    # Every C++ spelling, not just `*.*pp`: a fourth sink added as `.cc`,
    # `.cxx` or a `.h` would otherwise be swept by nothing. (The sweep is
    # still name-keyed on `send_ns`/`now_ns`, so a sink that renames both
    # locals AND does not include the header escapes it — that residual is
    # not covered by this sweep.)
    for path in _sink_candidate_files():
        if path.name == _SAMPLE_GATE.name:
            continue  # the decision itself; its header quotes the old shape
        try:
            body = _cxx_code_only(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError) as e:
            # Every other arm in this file prints and carries on; a
            # non-UTF-8 file here used to take the WHOLE run down with a
            # traceback (UnicodeDecodeError is a ValueError, not OSError).
            print(f"FAIL  [sample gate class sweep: cannot read "
                  f"{path.name} ({e}) — this arm fails closed rather than "
                  f"reporting no stray guards]")
            errors += 1
            continue
        if compare_re.search(body):
            strays.append(path.name)
        if "send_ns" in body and "classify_stamp_pair" not in body:
            unjudged.append(path.name)
    if strays:
        print(f"FAIL  [sample gate class sweep: {strays} COMPARE the stamp "
              f"pair outside sample_gate.hpp. One guard in three places, "
              f"each dropping silently, IS the finding — a second decision "
              f"re-opens it however it is spelled]")
        errors += 1
    if unjudged:
        print(f"FAIL  [sample gate class sweep: {unjudged} read `send_ns` "
              f"without reaching classify_stamp_pair — a sink that judges "
              f"the pair some other way passes every arm above while "
              f"dropping in silence, which is the finding itself]")
        errors += 1
    if errors == 0:
        # Gated on `errors`, not on `strays` alone: this line claims the
        # CALL SITES are wired as well as that no stray guard survives,
        # and an `ok` printed beside six FAILs above would be reporting a
        # claim the arms just refuted.
        print("ok    [sample gate: all three latency sinks route the drop "
              "decision through classify_stamp_pair over a default-less "
              "switch, OPEN each drop arm with its own counter, log it "
              "once, bail, take the sample inside the kUsable arm, and "
              "print every counter under its own key on a DELIVERY line "
              "reachable without finalize; no file compares the stamp pair "
              "or reads send_ns outside that decision]")

    # ---- (a) the compiled oracle. ----
    env_cxx = os.environ.get("CXX")
    cxx = ((shutil.which(env_cxx) if env_cxx else None) or
           shutil.which("clang++") or shutil.which("c++"))
    if cxx is None:
        print("skip  [sample gate oracle: no C++ compiler (clang++ / c++ / "
              "$CXX) — the call-site arms above still ran]")
        return errors
    global _SAMPLE_GATE_ORACLE_RAN
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
           "-fsyntax-only", str(_SAMPLE_GATE_ORACLE)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=300)
    except (OSError, subprocess.SubprocessError) as e:
        print(f"FAIL  [sample gate oracle: {' '.join(cmd)} could not run: "
              f"{e}]")
        return errors + 1
    if r.returncode != 0:
        failed = [ln for ln in (r.stdout + r.stderr).splitlines()
                  if _is_assertion_failure(ln) or "error:" in ln]
        print(f"FAIL  [sample gate oracle: rc={r.returncode}]\n     "
              + "\n     ".join(failed[:6] or
                               [(r.stdout + r.stderr).strip()[-400:]]))
        return errors + 1
    n_asserts = _cxx_code_only(
        _SAMPLE_GATE_ORACLE.read_text(encoding="utf-8")).count(
            "static_assert(")
    if n_asserts < _SAMPLE_GATE_MIN_ASSERTS:
        print(f"FAIL  [sample gate oracle: the oracle carries {n_asserts} "
              f"static_asserts, expected at least "
              f"{_SAMPLE_GATE_MIN_ASSERTS} — an empty file compiles clean, "
              f"so a gutted oracle would report this arm green having "
              f"checked nothing]")
        return errors + 1

    with tempfile.TemporaryDirectory(prefix="gate_") as td:
        ctl = Path(td)
        # NEGATIVE: a false assertion must fail the build and be
        # attributable (it also proves the line filter matches THIS
        # compiler's wording — clang and GCC disagree on it).
        neg_probe = ctl / "must_fail.cpp"
        neg_probe.write_text(
            '#include "%s"\n'
            "static_assert(ros2_rtt_bench::classify_stamp_pair(1, 2) ==\n"
            "              ros2_rtt_bench::StampVerdict::kUnstamped,\n"
            '              "item5 negative control");\n'
            % _SAMPLE_GATE.resolve(), encoding="utf-8")
        # EVERY probe below goes through `_probe_run`: a bare
        # subprocess.run RAISES on a spawn failure or a timeout, and a
        # CONTROL that crashes the checker is worse than one that fails —
        # it takes every later arm down with it.
        def _probe_run(path):
            try:
                return subprocess.run(cmd[:-1] + [str(path)],
                                      capture_output=True, text=True,
                                      encoding="utf-8", errors="replace",
                                      timeout=300)
            except (OSError, subprocess.SubprocessError) as e:
                return subprocess.CompletedProcess(
                    args=[], returncode=-1, stdout="",
                    stderr=f"probe could not run: {e}")

        neg = _probe_run(neg_probe)
        neg_lines = [ln for ln in (neg.stdout + neg.stderr).splitlines()
                     if _is_assertion_failure(ln)]
        # POSITIVE: a TRUE assertion over the same header must BUILD, or
        # the negative probe's failure is not a discriminating claim.
        true_probe = ctl / "must_build.cpp"
        true_probe.write_text(
            '#include "%s"\n'
            "static_assert(ros2_rtt_bench::classify_stamp_pair(1, 2) ==\n"
            "              ros2_rtt_bench::StampVerdict::kUsable,\n"
            '              "item5 positive control");\n'
            % _SAMPLE_GATE.resolve(), encoding="utf-8")
        aff = _probe_run(true_probe)

        # EXHAUSTIVENESS: sample_gate.hpp's own claim is that a verdict
        # added without a counter is named by -Wswitch at every call site.
        # Two probes, because "the exhaustive one built" says nothing on
        # its own: a switch covering every verdict must build clean, and
        # one MISSING an arm must not.
        sw_ok = ctl / "switch_exhaustive.cpp"
        sw_ok.write_text(
            '#include "%s"\n'
            "int probe(uint64_t a, uint64_t b) {\n"
            "  switch (ros2_rtt_bench::classify_stamp_pair(a, b)) {\n"
            "    case ros2_rtt_bench::StampVerdict::kUnstamped: return 1;\n"
            "    case ros2_rtt_bench::StampVerdict::kNonPositiveRtt:"
            " return 2;\n"
            "    case ros2_rtt_bench::StampVerdict::kUsable: break;\n"
            "  }\n"
            "  return 0;\n"
            "}\n" % _SAMPLE_GATE.resolve(), encoding="utf-8")
        sw_bad = ctl / "switch_partial.cpp"
        sw_bad.write_text(
            sw_ok.read_text(encoding="utf-8").replace(
                "    case ros2_rtt_bench::StampVerdict::kNonPositiveRtt:"
                " return 2;\n", ""), encoding="utf-8")
        sw_ok_r = _probe_run(sw_ok)
        sw_bad_r = _probe_run(sw_bad)

        # LINKAGE: perturb a COPY of the shipping header the way the
        # boundary could slip — `<=` widened to `<`, which re-admits the
        # equal-instants pair as a zero-nanosecond "latency" — and compile
        # a COPY of the oracle against it. A live oracle FAILS. A
        # disabled, vacuous, or drifted-onto-its-own-copy oracle passes,
        # and that is what this catches.
        (ctl / "src").mkdir()
        (ctl / "test").mkdir()
        gate_src = _SAMPLE_GATE.read_text(encoding="utf-8")
        boundary = "  if (now_ns <= send_ns) {"
        if boundary not in gate_src:
            print("FAIL  [sample gate oracle: the linkage control cannot "
                  "find the `now_ns <= send_ns` boundary to perturb — "
                  "classify_stamp_pair has been rewritten and this control "
                  "is no longer perturbing the thing it names]")
            return errors + 1
        (ctl / "src" / _SAMPLE_GATE.name).write_text(
            gate_src.replace(boundary, "  if (now_ns < send_ns) {"),
            encoding="utf-8")
        (ctl / "test" / _SAMPLE_GATE_ORACLE.name).write_text(
            _SAMPLE_GATE_ORACLE.read_text(encoding="utf-8"), encoding="utf-8")
        pos = _probe_run(ctl / "test" / _SAMPLE_GATE_ORACLE.name)

        # A SECOND linkage perturbation, because the boundary one moves
        # only the `<=`. The header's other load-bearing choice is the
        # ORDER of its two tests: `send_ns == 0` is asked FIRST so an
        # unstamped echo is reported as unstamped and never as a
        # non-positive round trip (with now_ns > 0 the two conditions
        # overlap for EVERY unstamped echo). Flipping them keeps both
        # verdicts reachable, keeps every boundary assertion true, and
        # silently re-attributes a publisher wiring fault to clock
        # behaviour — so a widened-boundary control alone does not cover
        # it.
        order_before = ("  if (send_ns == 0) {\n"
                        "    return StampVerdict::kUnstamped;\n"
                        "  }\n"
                        "  if (now_ns <= send_ns) {\n"
                        "    return StampVerdict::kNonPositiveRtt;\n"
                        "  }")
        order_after = ("  if (now_ns <= send_ns) {\n"
                       "    return StampVerdict::kNonPositiveRtt;\n"
                       "  }\n"
                       "  if (send_ns == 0) {\n"
                       "    return StampVerdict::kUnstamped;\n"
                       "  }")
        if order_before not in gate_src:
            print("FAIL  [sample gate oracle: the ORDER control cannot find "
                  "the two tests to swap — classify_stamp_pair has been "
                  "rewritten and this control no longer perturbs the thing "
                  "it names]")
            return errors + 1
        ordr_dir = ctl / "order"
        (ordr_dir / "src").mkdir(parents=True)
        (ordr_dir / "test").mkdir(parents=True)
        (ordr_dir / "src" / _SAMPLE_GATE.name).write_text(
            gate_src.replace(order_before, order_after), encoding="utf-8")
        (ordr_dir / "test" / _SAMPLE_GATE_ORACLE.name).write_text(
            _SAMPLE_GATE_ORACLE.read_text(encoding="utf-8"), encoding="utf-8")
        ordr = _probe_run(ordr_dir / "test" / _SAMPLE_GATE_ORACLE.name)

        if aff.returncode != 0:
            print(f"FAIL  [sample gate oracle: a TRUE static_assert over "
                  f"the shipping header did not build (rc={aff.returncode}) "
                  f"— the probe harness itself is broken, so the false "
                  f"probe's failure below proves nothing\n     "
                  + (aff.stdout + aff.stderr).strip()[-300:] + "]")
            errors += 1
        elif neg.returncode == 0:
            print("FAIL  [sample gate oracle: a deliberately false "
                  "static_assert COMPILED — this arm cannot fail, so its "
                  "green says nothing]")
            errors += 1
        elif not neg_lines:
            print(f"FAIL  [sample gate oracle: a false static_assert failed "
                  f"the build but this arm's line filter extracted nothing "
                  f"from {Path(cxx).name}'s diagnostics, so a real failure "
                  f"would report no attributable line]")
            errors += 1
        elif sw_ok_r.returncode != 0:
            print(f"FAIL  [sample gate exhaustiveness: a switch covering "
                  f"every verdict did not build (rc={sw_ok_r.returncode}) — "
                  f"the sinks' own shape does not compile\n     "
                  + (sw_ok_r.stdout + sw_ok_r.stderr).strip()[-300:] + "]")
            errors += 1
        elif sw_bad_r.returncode == 0 or not any(
                sp in (sw_bad_r.stdout + sw_bad_r.stderr)
                for sp in _WSWITCH_SPELLINGS):
            print(f"FAIL  [sample gate exhaustiveness: a switch MISSING a "
                  f"verdict compiled without -Wswitch "
                  f"(rc={sw_bad_r.returncode}) — sample_gate.hpp claims an "
                  f"unhandled verdict is named at every call site, and "
                  f"that claim is false on this compiler]")
            errors += 1
        elif not any(_is_assertion_failure(ln)
                     for ln in (pos.stdout + pos.stderr).splitlines()):
            print(f"FAIL  [sample gate oracle: widening the boundary to "
                  f"`now_ns < send_ns` did not trip a single ASSERTION "
                  f"(rc={pos.returncode}) — the oracle is disabled, "
                  f"vacuous, or no longer reading the shipping header, so "
                  f"its green says nothing about the code that ships"
                  + (". It did fail to build, which is not the same claim"
                     if pos.returncode != 0 else "") + "]")
            errors += 1
        elif not any(_is_assertion_failure(ln)
                     for ln in (ordr.stdout + ordr.stderr).splitlines()):
            print(f"FAIL  [sample gate oracle: SWAPPING the two tests in "
                  f"classify_stamp_pair did not trip a single ASSERTION "
                  f"(rc={ordr.returncode}) — an unstamped echo would then "
                  f"be reported as a non-positive round trip, sending an "
                  f"operator after the clock for a publisher that never "
                  f"stamped, and the oracle does not notice"
                  + (". It did fail to build, which is not the same claim"
                     if ordr.returncode != 0 else "") + "]")
            errors += 1
        else:
            _SAMPLE_GATE_ORACLE_RAN = True
            print(f"ok    [sample gate oracle: {n_asserts} static_asserts "
                  f"over the shipping sample_gate.hpp agree "
                  f"({Path(cxx).name} -Wall -Wextra -Wpedantic -Werror); a "
                  f"deliberately false one is caught and attributed, an "
                  f"unhandled verdict is named by -Wswitch, and both "
                  f"widening the `<=` boundary and swapping the two tests "
                  f"FAIL the oracle]")
    return errors


# ---------------------------------------------------------------------
# The compiled oracles are CMake targets
# ---------------------------------------------------------------------

_BENCH_CMAKELISTS = _ROS2_BENCH_SRC / "CMakeLists.txt"

# The two compiled hand oracles, and the reason each has to be built by
# the package rather than only by this checker. `check_image_stamp_codec`
# and `check_sample_gate_accounting` compile them on the DESK with
# clang++; that is this file's own driver and it says nothing about the
# container, which is where the measurement actually runs. Both files sat
# under test/ with a driver named in their headers, and colcon's
# CMakeLists compiled NEITHER — so the ROS 2 image built, shipped and ran
# without evaluating a single one of their static_asserts.
_ORACLE_CMAKE_SOURCES = {
    "test/stamp_codec_oracle.cpp":
        "the image class's builtin_interfaces/Time stamp codec",
    "test/sample_gate_oracle.cpp":
        "the latency sinks' stamp-pair gate",
}

# Words CMake puts in a target's argument list that are not sources.
_CMAKE_TARGET_KEYWORDS = frozenset({
    "STATIC", "SHARED", "MODULE", "OBJECT", "INTERFACE", "IMPORTED",
    "ALIAS", "GLOBAL", "UNKNOWN", "EXCLUDE_FROM_ALL", "WIN32",
    "MACOSX_BUNDLE",
})

def _cmake_code_only(text: str) -> str:
    """`text` with CMake comments blanked, newlines preserved.

    Both syntaxes: a bracket comment `#[[ … ]]` (which may span lines) and
    a `#`-to-end-of-line one. The bracket form is stripped FIRST, or its
    opening `#` would be read as a line comment and its body would survive
    as code. PURE — driven by hand vectors below, because a stripper that
    quietly does nothing makes every derivation over it vacuous."""
    out, i, n = [], 0, len(text)
    while i < n:
        # `#[` + any number of `=` + `[` … matching `]` + the SAME number
        # of `=` + `]`. Handling only `#[[` left `#[=[ … ]=]` as code: the
        # marker line's own `#` opened a line comment and every following
        # line of the commented-out block survived as a live command.
        bracket = re.match(r"#\[(=*)\[", text[i:i + 64]) if text[i] == "#" else None
        if bracket is not None:
            closer = "]" + bracket.group(1) + "]"
            end = text.find(closer, i + bracket.end())
            stop = n if end < 0 else end + len(closer)
            out.append("".join(c if c == "\n" else " "
                               for c in text[i:stop]))
            i = stop
        elif text[i] == "#":
            end = text.find("\n", i)
            stop = n if end < 0 else end
            out.append(" " * (stop - i))
            i = stop
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


# Block openers that make everything inside them CONDITIONAL on something
# this arm cannot evaluate, paired with their closers. A target inside one
# is NOT built by `colcon build --packages-select …` on its own.
_CMAKE_BLOCK_OPENERS = {"if": "endif", "foreach": "endforeach",
                        "while": "endwhile", "function": "endfunction",
                        "macro": "endmacro"}


def _cmake_target_sources(cmake_text: str) -> "dict":
    """Every source named by a target in `cmake_text`, mapped to the
    reason colcon would NOT compile it — or `None` when it is compiled
    unconditionally as part of the default build.

    PURE, and derived from the BUILD rather than from a list somebody
    maintains: a translation unit that is not named by an
    `add_executable` / `add_library` is not compiled by colcon, whatever
    else the package contains.

    The REASON column is the half a name match cannot supply. A set of "names
    appearing as a source argument somewhere" reports a target wrapped in
    `if(BUILD_TESTING)` — or in `if(FALSE)`, or in a `function()` nobody
    calls, or carrying `EXCLUDE_FROM_ALL` — as compiled. That is exactly
    the state this package's CMakeLists comment forbids by name ("a target
    that looks wired and compiles nowhere"), so a derivation blind to it
    would bless the defect it exists to refuse. Measured before the fix:
    wrapping both oracle targets in `if(BUILD_TESTING)` left the arm
    green.

    Paths are keyed exactly as written (package-relative, as this
    CMakeLists spells them). Target names and CMake's own keywords are
    dropped by EXTENSION, so `add_library(x ALIAS y)` contributes
    nothing."""
    code = _cmake_code_only(cmake_text)
    out: "dict" = {}
    # Where each conditional/function block opens, so a target's position
    # can be tested against them. Nesting is counted, never assumed.
    blocks: "list" = []          # (open_at, close_at, kind)
    stack: "list" = []
    for m in re.finditer(r"\b(end)?(if|foreach|while|function|macro)\s*\(",
                         code):
        if m.group(1) is None:
            stack.append((m.start(), m.group(2)))
        elif stack:
            # `endif` closes the nearest `if`; a mismatched closer is
            # ignored rather than silently rebalancing the stack.
            for k in range(len(stack) - 1, -1, -1):
                if stack[k][1] == m.group(2):
                    blocks.append((stack[k][0], m.end(), m.group(2)))
                    del stack[k:]
                    break
    # An UNCLOSED opener guards everything after it — fail closed.
    for at, kind in stack:
        blocks.append((at, len(code), kind))

    for m in re.finditer(r"\badd_(?:executable|library)\s*\(", code):
        depth, j = 1, m.end()
        while j < len(code) and depth:
            if code[j] == "(":
                depth += 1
            elif code[j] == ")":
                depth -= 1
            j += 1
        if depth:
            continue                      # unbalanced: nothing to derive
        args = code[m.end():j - 1].replace('"', " ").split()
        guard = next((f"it is inside a `{k}()` block"
                      for a, b, k in blocks if a < m.start() < b), None)
        if guard is None and "EXCLUDE_FROM_ALL" in args:
            guard = "the target carries EXCLUDE_FROM_ALL"
        for tok in args[1:]:              # args[0] is the target name
            if tok in _CMAKE_TARGET_KEYWORDS or tok.startswith("$"):
                continue
            if not tok.endswith(_CMAKE_SOURCE_SUFFIXES):
                # A token that LOOKS like a file and carries an extension
                # this accounting does not know. Recorded rather than
                # skipped: the suffix list and the src/ walk are derived
                # from ONE table now, and agreeing tables are only as
                # total as the set they agree on — a `src/x.c` named by a
                # target was in neither, so it escaped the accounting
                # whose whole point is to be inescapable.
                if "." in tok.rsplit("/", 1)[-1]:
                    key = tok[2:] if tok.startswith("./") else tok
                    out.setdefault(
                        key, "its extension is not one this accounting "
                             "recognises (see _SINK_EXTENSIONS)")
                continue
            key = tok[2:] if tok.startswith("./") else tok
            # A source named by SEVERAL targets is built if ANY of
            # them builds it unconditionally.
            if key not in out or out[key] is not None:
                out[key] = guard
    return out


# Sources this package's CMakeLists must name whatever else changes.
# Not an inventory of the build — an ANTI-INERT floor: every claim below
# is "X is missing from the derived set", and a derivation that returned
# the EMPTY set would make all of them fire for the wrong reason while a
# parser that silently stopped working looked like a real regression.
_KNOWN_BUILT_SOURCES = ("src/ping_node.cpp", "src/pong_node.cpp",
                        "src/latency_node.cpp")


def check_oracles_are_cmake_targets() -> int:
    """colcon must COMPILE both hand oracles.

    Each oracle's verdict is its own build's exit code — every expectation
    in it is a `static_assert` over a shipping ROS-free header — so a file
    nothing compiles asserts nothing, however many static_asserts it
    holds. Both were in exactly that state: named as the decision's proof
    by comments in the headers AND by this checker's own arms, and built
    by no CMake target at all.

    What this arm does NOT claim: that the container build succeeds. It
    reads the package's CMakeLists and asserts the wiring; the desk
    already compiles the same two files (the oracle arms above), and the
    container is where the two facts meet."""
    errors = 0

    # ---- (a) the comment stripper, over hand vectors ------------------
    for label, text, want in (
            ("a line comment", "add_library(a x.cpp) # add_library(b y.cpp)",
             {"x.cpp"}),
            ("a bracket comment hiding a target",
             "#[[ add_library(b y.cpp) ]]\nadd_library(a x.cpp)",
             {"x.cpp"}),
            ("a MULTI-LINE bracket comment",
             "#[[\nadd_library(b y.cpp)\n]]\nadd_library(a x.cpp)",
             {"x.cpp"}),
            # CMake bracket comments take ANY number of `=` between the
            # brackets. Handling only `#[[` left `#[=[ … ]=]` as CODE, so
            # commenting the oracle targets out with the equals form
            # reported them compiled.
            ("an `#[=[ … ]=]` bracket comment",
             "#[=[\nadd_library(b y.cpp)\n]=]\nadd_library(a x.cpp)",
             {"x.cpp"}),
            ("an `#[==[ … ]==]` bracket comment (any count)",
             "#[==[\nadd_library(b y.cpp)\n]==]\nadd_library(a x.cpp)",
             {"x.cpp"}),
            ("a `]=]` that does not close a `#[[`",
             "#[[\nadd_library(b y.cpp)\n]=]\nadd_library(a x.cpp)\n]]",
             set()),
            ("an unterminated bracket comment (fails CLOSED — everything "
             "after it is a comment)",
             "add_library(a x.cpp)\n#[[ add_library(b y.cpp)",
             {"x.cpp"}),
    ):
        got = set(_cmake_target_sources(text))
        if got != want:
            print(f"FAIL  [oracle targets: the CMake comment stripper on "
                  f"{label} derived {sorted(got)}, expected {sorted(want)} "
                  f"— a target that only exists inside a comment is not "
                  f"built, and one hidden by a stripper that over-reaches "
                  f"is reported missing when it is there]")
            errors += 1

    # ---- (b) the derivation itself, over hand vectors -----------------
    for label, text, want in (
            ("an executable", "add_executable(ping src/ping.cpp)",
             {"src/ping.cpp"}),
            ("an OBJECT library — the keyword is not a source",
             "add_library(o OBJECT test/o.cpp)", {"test/o.cpp"}),
            ("two sources on one target",
             "add_library(o OBJECT a.cpp b.cc)", {"a.cpp", "b.cc"}),
            ("an ALIAS target, which compiles nothing",
             "add_library(a ALIAS b)", set()),
            ("a quoted source", 'add_executable(p "src/p.cpp")',
             {"src/p.cpp"}),
            ("a target whose NAME ends in a source suffix — the first "
             "argument is never a source",
             "add_executable(weird.cpp src/real.cpp)", {"src/real.cpp"}),
            ("nested parens in the argument list",
             "add_executable(p src/p.cpp ${EXTRA})", {"src/p.cpp"}),
            ("an UNBALANCED call (fails closed, derives nothing)",
             "add_executable(p src/p.cpp", set()),
            ("a command that merely mentions a source",
             'target_link_libraries(p "src/other.cpp")', set()),
    ):
        got = set(_cmake_target_sources(text))
        if got != want:
            print(f"FAIL  [oracle targets: the CMake source derivation on "
                  f"{label} returned {sorted(got)}, expected "
                  f"{sorted(want)}]")
            errors += 1

    # ---- (b2) NAMED is not BUILT -------------------------------------
    # The reason column, over the shapes that make a named target compile
    # nowhere. None of these shapes are caught by a plain set-returning
    # check, which is why the column exists.
    for label, text, want_reason in (
            ("a plain top-level target", "add_library(o OBJECT t/o.cpp)",
             None),
            ("a target inside `if(BUILD_TESTING)` — the shape this "
             "package's own CMakeLists comment forbids by name",
             "if(BUILD_TESTING)\nadd_library(o OBJECT t/o.cpp)\nendif()",
             "it is inside a `if()` block"),
            ("a target inside `if(FALSE)`",
             "if(FALSE)\nadd_library(o OBJECT t/o.cpp)\nendif()",
             "it is inside a `if()` block"),
            ("a target inside a function nobody calls",
             "function(f)\nadd_library(o OBJECT t/o.cpp)\nendfunction()",
             "it is inside a `function()` block"),
            ("a target after a CLOSED conditional is built again",
             "if(X)\nmessage(hi)\nendif()\nadd_library(o OBJECT t/o.cpp)",
             None),
            ("EXCLUDE_FROM_ALL",
             "add_library(o OBJECT EXCLUDE_FROM_ALL t/o.cpp)",
             "the target carries EXCLUDE_FROM_ALL"),
            ("an UNCLOSED conditional guards everything after it",
             "if(X)\nadd_library(o OBJECT t/o.cpp)",
             "it is inside a `if()` block"),
            ("a nested conditional closes only its own level",
             "if(A)\nif(B)\nmessage(hi)\nendif()\n"
             "add_library(o OBJECT t/o.cpp)\nendif()",
             "it is inside a `if()` block"),
            ("the same source built BOTH ways is built",
             "if(X)\nadd_library(a OBJECT t/o.cpp)\nendif()\n"
             "add_library(b OBJECT t/o.cpp)", None),
    ):
        got = _cmake_target_sources(text).get("t/o.cpp", "ABSENT")
        if got != want_reason:
            print(f"FAIL  [oracle targets: {label} was derived as "
                  f"{got!r}, expected {want_reason!r}. A source NAMED by a "
                  f"target and a source colcon COMPILES are different "
                  f"sets, and the difference is the defect this arm "
                  f"exists to refuse]")
            errors += 1

    # ---- (c) the real package ----------------------------------------
    try:
        cmake = _BENCH_CMAKELISTS.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [oracle targets: cannot read "
              f"{_BENCH_CMAKELISTS.name} ({e}) — this arm fails closed "
              f"rather than reporting the oracles wired]")
        return errors + 1
    built = _cmake_target_sources(cmake)
    missing_known = [s for s in _KNOWN_BUILT_SOURCES
                     if built.get(s, "ABSENT") is not None]
    if missing_known:
        print(f"FAIL  [oracle targets: the derivation cannot find "
              f"{missing_known} in this package's own CMakeLists — the "
              f"shipping binaries are certainly built, so the parser is "
              f"broken and every verdict below it would fire for the "
              f"wrong reason]")
        return errors + 1
    if len(_ORACLE_CMAKE_SOURCES) != 2:
        # A drift floor: the dict is BOTH the required set and (for the
        # sink accounting's claim 1) the exemption set, so deleting an
        # entry alongside its target would leave nothing demanding that
        # oracle be built — the exact state item 4 closed.
        print(f"FAIL  [oracle targets: _ORACLE_CMAKE_SOURCES lists "
              f"{len(_ORACLE_CMAKE_SOURCES)} oracle(s); this package has "
              f"two, and removing an entry silently retires the arm that "
              f"requires its target]")
        errors += 1
    for rel, what in sorted(_ORACLE_CMAKE_SOURCES.items()):
        if rel in built and built[rel] is not None:
            print(f"FAIL  [oracle targets: {rel} — the hand oracle for "
                  f"{what} — is named by a target, but {built[rel]}, so "
                  f"`colcon build --packages-select …` does not compile "
                  f"it. A target that looks wired and compiles nowhere is "
                  f"the state item 4 closed, and this package's own "
                  f"CMakeLists comment forbids it by name]")
            errors += 1
        elif rel not in built:
            print(f"FAIL  [oracle targets: {rel} — the hand oracle for "
                  f"{what} — is named by no add_executable/add_library in "
                  f"{_BENCH_CMAKELISTS.name}, so colcon never compiles "
                  f"it. Its verdict IS its build's exit code, so an "
                  f"unbuilt oracle asserts NOTHING in the container no "
                  f"matter how many static_asserts it carries — while "
                  f"two headers and two arms in this file name it as "
                  f"their proof. Add "
                  f"`add_library(<name> OBJECT {rel})`]")
            errors += 1
        elif not (_ROS2_BENCH_SRC / rel).exists():
            print(f"FAIL  [oracle targets: {_BENCH_CMAKELISTS.name} names "
                  f"{rel} but that file does not exist — the container "
                  f"build would fail at configure time, and this arm "
                  f"would otherwise report the oracle wired]")
            errors += 1
    if errors == 0:
        print(f"ok    [oracle targets: colcon compiles both hand oracles "
              f"({', '.join(sorted(_ORACLE_CMAKE_SOURCES))}) — each named "
              f"by a target in {_BENCH_CMAKELISTS.name}, so a failed "
              f"static_assert breaks the container build rather than "
              f"waiting for a desk run of this checker]")
    return errors


# ---------------------------------------------------------------------
# The sink set is ACCOUNTED, not recognised by tokens
# ---------------------------------------------------------------------

# Every file under the bench's src/ that is NOT a latency sink, with the
# reason. A DECLARED inventory, because "is this a sink?" has no shape a
# text search can settle: the sweep that used to answer it was keyed on
# the identifiers `send_ns`/`now_ns` and on `#include "sample_gate.hpp"`,
# and a fourth sink that renamed both locals and included nothing
# satisfied neither. Both of those are properties the NEW FILE controls.
# What it does not control is the BUILD: a translation unit colcon never
# compiles is not a sink, and one it does compile must be named here or
# in _SAMPLE_GATE_SINKS. Adding a sink is then a deliberate edit to a
# declaration rather than a rename that nothing notices.
_DECLARED_NON_SINK_SOURCES = {
    "ping_node.cpp":
        "the PUBLISHER role: it stamps and sends, and judges no echo",
    "pong_node.cpp":
        "the ECHO role: it re-publishes what it receives and measures "
        "nothing",
    "pong_node_rcl.cpp":
        "the ECHO role on the loan receive path — same, via "
        "rcl_take_loaned_message",
    "common.hpp":
        "shared env/CLI plumbing and the sample dump; no stamp pair "
        "reaches it",
    "msg_class_dispatch.hpp":
        "the TYPE-CLASS adapter (pod vs image): it encodes and decodes "
        "the stamp, and never decides whether a pair is usable",
    "pod_dispatch.hpp":
        "the Pod<N> template instantiation table",
    "sample_gate.hpp":
        "THE decision itself — every sink routes through it",
    "stamp_codec.hpp":
        "the image class's builtin_interfaces/Time codec",
}


def _sink_accounting_failures(*, built: "set", src_names: "set",
                              tabled: "set", declared_non_sinks: "set",
                              oracle_sources: "set",
                              unrecognised: "set" = frozenset()) -> "List[str]":
    # KEYWORD-ONLY. Five same-typed sets, and two of them (`built`,
    # `oracle_sources`) are package-relative PATHS while the other three
    # are bare BASENAMES — adjacent parameters from different key spaces,
    # where a transposition compiles and quietly loosens the arm.
    """Everything unaccounted, as messages. PURE, so the hostile trees
    below can be driven without touching the real package.

    Four claims, and each closes a different way round the sweep:

      1. every source the BUILD compiles is under src/ or is a declared
         oracle — otherwise a sink could live outside the directory every
         other arm globs;
      2. every file under src/ is EITHER a judged sink or a declared
         non-sink, and never both;
      3. no declaration names a file that is not there — a stale entry
         pre-authorises a future file of that name, which is how an
         exemption outlives the thing it excused;
      4. every judged sink is actually BUILT — a sink colcon does not
         compile is a sink the container never runs, and the per-sink
         arms above would be checking a file that ships nowhere.
    """
    out: "List[str]" = []
    for src in sorted(built):
        if src in oracle_sources or src.startswith("src/"):
            continue
        out.append(
            f"FAIL  [sink accounting: the build compiles {src!r}, which is "
            f"neither under src/ nor a declared oracle. Every arm that "
            f"judges a sink globs src/, so a sink placed outside it is "
            f"judged by nothing while colcon builds and ships it]")
    for name in sorted(src_names):
        in_table = name in tabled
        declared = name in declared_non_sinks
        if in_table and declared:
            out.append(
                f"FAIL  [sink accounting: src/{name} is BOTH a judged sink "
                f"and a declared non-sink — the two lists disagree about "
                f"the same file, and the per-sink arms would judge a file "
                f"this list says needs no judging]")
        elif not in_table and not declared:
            out.append(
                f"FAIL  [sink accounting: src/{name} is accounted for by "
                f"nothing. It is a translation unit in the bench package "
                f"that is neither listed in _SAMPLE_GATE_SINKS (so no "
                f"per-sink arm judges its drop arms, its counters or its "
                f"DELIVERY receipt) nor declared a non-sink with a "
                f"reason. A fourth latency sink that renames send_ns / "
                f"now_ns and includes no shared header escapes every "
                f"TOKEN sweep in this file — it cannot escape being a "
                f"file. Add it to _SAMPLE_GATE_SINKS with its bail and "
                f"its post-spin anchor, or to _DECLARED_NON_SINK_SOURCES "
                f"with the reason it judges no echo]")
    for name in sorted(set(declared_non_sinks) | set(tabled)):
        if name not in src_names:
            where = ("_SAMPLE_GATE_SINKS" if name in tabled
                     else "_DECLARED_NON_SINK_SOURCES")
            out.append(
                f"FAIL  [sink accounting: {where} names src/{name}, which "
                f"does not exist. A declaration that outlives its file "
                f"pre-authorises the next file of that name — the "
                f"exemption arrives before the code it excuses]")
    for src in sorted(unrecognised):
        out.append(
            f"FAIL  [sink accounting: the build compiles {src!r}, whose "
            f"extension this accounting does not recognise, so it appears "
            f"in neither the built set nor the src/ walk and is judged by "
            f"nothing. A sink can be written in any language the package "
            f"compiles; add the extension to _SINK_EXTENSIONS (the src/ "
            f"walk and the CMake suffix set are derived from it) rather "
            f"than leaving a shape nothing accounts for]")
    for name in sorted(tabled):
        if name in src_names and f"src/{name}" not in built:
            out.append(
                f"FAIL  [sink accounting: src/{name} is judged as a sink "
                f"but is named by no CMake target, so colcon never "
                f"compiles it. Every per-sink arm above would be reading "
                f"a file the container does not ship]")
    return out


def check_sink_set_is_accounted() -> int:
    """No latency sink can escape by renaming.

    `check_sample_gate_accounting` derives its sink set from an
    `#include`, sweeps for a second decision by the identifiers
    `send_ns`/`now_ns`, and iterates a hand table for everything else —
    three keys, all of them properties a NEW FILE chooses for itself. This
    arm keys on the two it does not: that it is a file in the package, and
    that the build compiles it.

    The token sweep in the other arm is NOT made redundant by this one and
    stays: it catches a second decision growing inside a file that is
    already declared (a comparison appearing in ping_node.cpp), which is a
    different failure from a file nobody accounted for."""
    errors = 0

    # ---- (a) the pure accounting, over hostile trees ------------------
    base_built = {"src/ping_node.cpp", "src/latency_node.cpp",
                  "test/oracle.cpp"}
    base_src = {"ping_node.cpp", "latency_node.cpp"}
    base_tab = {"latency_node.cpp"}
    base_dec = {"ping_node.cpp"}
    base_orc = {"test/oracle.cpp"}
    for label, built, src, tab, dec, unrec, want_clean, needle in (
            ("a correctly accounted tree", base_built, base_src, base_tab,
             base_dec, set(), True, ""),
            # THE finding: a fourth sink that renames its locals and
            # includes nothing. Invisible to every token sweep; it is
            # still a file, and a built one.
            ("a renamed fourth sink nothing declares",
             base_built | {"src/sneaky.cpp"}, base_src | {"sneaky.cpp"},
             base_tab, base_dec, set(), False, "accounted for by nothing"),
            ("a sink placed OUTSIDE src/",
             base_built | {"extra/sneaky.cpp"}, base_src, base_tab,
             base_dec, set(), False,
             "neither under src/ nor a declared oracle"),
            ("a file declared BOTH ways", base_built, base_src, base_tab,
             base_dec | {"latency_node.cpp"}, set(), False,
             "BOTH a judged sink"),
            ("a declaration whose file is gone", base_built, base_src,
             base_tab, base_dec | {"ghost.cpp"}, set(), False,
             "does not exist"),
            ("a judged sink the build does not compile",
             base_built - {"src/latency_node.cpp"}, base_src, base_tab,
             base_dec, set(), False, "colcon never compiles it"),
            # A sink in a language the suffix tables do not list: in
            # NEITHER the built set nor the src/ walk, so every claim
            # above is silent about it.
            ("a source whose extension nothing recognises", base_built,
             base_src, base_tab, base_dec, {"src/sneaky.c"}, False,
             "extension this accounting does not recognise"),
    ):
        got = _sink_accounting_failures(
            built=built, src_names=src, tabled=tab, declared_non_sinks=dec,
            oracle_sources=base_orc, unrecognised=unrec)
        if want_clean and got:
            print(f"FAIL  [sink accounting oracle: {label} was reported "
                  f"BROKEN — {got!r}. Every verdict below is vacuous "
                  f"against an accounting that complains about "
                  f"everything]")
            errors += 1
        elif not want_clean and not any(needle in m for m in got):
            print(f"FAIL  [sink accounting oracle: {label} was reported "
                  f"CLEAN (or without {needle!r}); got {got!r}]")
            errors += 1

    # ---- (b) the real package ----------------------------------------
    try:
        cmake = _BENCH_CMAKELISTS.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [sink accounting: cannot read "
              f"{_BENCH_CMAKELISTS.name} ({e}) — this arm fails closed]")
        return errors + 1
    built = {src for src, why in _cmake_target_sources(cmake).items()
             if why is None}
    if not any(s in built for s in _KNOWN_BUILT_SOURCES):
        print(f"FAIL  [sink accounting: the build derivation found none of "
              f"{list(_KNOWN_BUILT_SOURCES)} — the parser is broken, and "
              f"an empty build set would report every sink uncompiled]")
        return errors + 1
    src_names = {p.name for p in _sink_candidate_files()}
    if not src_names:
        print("FAIL  [sink accounting: no C++ sources found under the "
              "bench's src/ — this arm fails closed rather than reporting "
              "an empty package fully accounted]")
        return errors + 1
    derived = _cmake_target_sources(cmake)
    real = _sink_accounting_failures(
        built=built, src_names=src_names,
        tabled={f for f, _o, _b, _a, _r, _s in _SAMPLE_GATE_SINKS},
        declared_non_sinks=set(_DECLARED_NON_SINK_SOURCES),
        oracle_sources=set(_ORACLE_CMAKE_SOURCES),
        unrecognised={src for src, why in derived.items()
                      if why and "extension" in why})
    for msg in real:
        print(msg)
    errors += len(real)
    if errors == 0:
        print(f"ok    [sink accounting: all {len(src_names)} C++ sources "
              f"in the bench package are accounted for — "
              f"{len(_SAMPLE_GATE_SINKS)} judged sinks (each built by a "
              f"CMake target) and {len(_DECLARED_NON_SINK_SOURCES)} "
              f"declared non-sinks with reasons; nothing the build "
              f"compiles sits outside src/, and no declaration outlives "
              f"its file]")
    return errors


# ---------------------------------------------------------------------
# The blanket backstop asserts its own confinement
# ---------------------------------------------------------------------

def _sh_code_only(text: str) -> str:
    """`text` with bash `#` comments blanked, newlines and columns kept.

    The same discipline `_cxx_code_only` and `_cmake_code_only` apply, for
    the same reason: this file's own comments NAME the tokens its checks
    look for, so a needle searched over raw source is satisfied by the
    prose that explains it. A `#` inside single or double quotes is kept —
    the shipped refusal text quotes a pattern containing none, but a
    stripper that blanked from a quoted `#` would eat real code.

    Not exact bash lexing (no here-doc or `$'...'` modelling); it is used
    to check that a NAME appears as a command, where over-keeping is the
    safe direction."""
    out = []
    for line in text.split("\n"):
        q = None
        for k, ch in enumerate(line):
            if q is not None:
                if ch == q:
                    q = None
            elif ch in "'\"":
                q = ch
            elif ch == "#" and (k == 0 or line[k - 1] in " \t"):
                line = line[:k] + " " * (len(line) - k)
                break
        out.append(line)
    return "\n".join(out)


_BACKSTOP_FUNNEL = "reap_bench_binaries"
_BACKSTOP_GATE = "backstop_confined_by"
# The one spelling of the blanket sweep. Matched as the PATTERN VARIABLE
# rather than as a whole command line, so re-spelling the flags
# (`pkill -9 -f`, `pkill --signal KILL -f`) cannot dodge the funnel check.
_BACKSTOP_SWEEP = '-f "$BENCH_BIN_PATTERN"'


def check_backstop_is_namespace_gated() -> int:
    """The blanket bench-binary sweep must assert its own confinement.

    `pkill -KILL -f "$BENCH_BIN_PATTERN"` kills by install-path regex and
    proves NO ownership: it signals every matching process in its PID
    namespace, ours or not. Its safety rests entirely on a deployment
    property — bench.py runs one container per cell, never `--pid host` —
    that nothing in the script asserted. Decision: a direct
    invocation is not a supported entry point. The fix is therefore to
    ASSERT the property rather than to prove pid ownership for a pattern
    that cannot carry one.

    Not deleted, and the reason is in the ladder arms above: in the
    degraded state where `ps` answers nothing, every `kill_owned_pid` rung
    refuses on an empty identity, and removing the sweep would turn a
    blunt reap into a hang. The ping/pong ladder gained its own KILL rung
    so the sweep is a backstop rather than the mechanism."""
    errors = 0
    try:
        runner = _RUN_BENCH.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [backstop gate: cannot read run_bench.sh ({e}) — "
              f"this arm fails closed rather than reporting the sweep "
              f"gated]")
        return 1

    # ---- (a) the sweep exists ONLY inside the funnel -------------------
    # A funnel is what makes the gate uncopyable: the next teardown that
    # wants a blanket reap has to call the function that carries the
    # check, because there is no second spelling to copy.
    try:
        funnel = _extract_bash_function(_RUN_BENCH, _BACKSTOP_FUNNEL)
    except RuntimeError as e:
        print(f"FAIL  [backstop gate: {e} — the blanket sweep has no "
              f"single funnel, so the namespace assertion can be copied "
              f"without it]")
        return errors + 1
    outside = _sh_code_only(runner.replace(funnel, ""))
    # Anchored on COMMAND POSITION, not on the absence of the word
    # `echo`. The previous filter excluded any line mentioning `echo` so
    # it would not trip over the funnel's own refusal text — and a real
    # second sweep written `pkill -KILL -f "$BENCH_BIN_PATTERN" || echo
    # swept` was then skipped, which was measured green. Comments are
    # already blanked above, and the refusal line spells the pattern in
    # SINGLE quotes, so neither needs an exclusion.
    _sweep_re = re.compile(r"(?:^|[;&|]|\bthen\b|\bdo\b|\belse\b)\s*"
                           r"pkill\b[^\n]*" + re.escape(_BACKSTOP_SWEEP))
    stray = [ln.strip() for ln in outside.splitlines()
             if _sweep_re.search(ln)]
    if stray:
        print(f"FAIL  [backstop gate: run_bench.sh spells the blanket "
              f"sweep outside {_BACKSTOP_FUNNEL}(): {stray}. That copy "
              f"carries no namespace assertion, and the file then holds "
              f"BOTH spellings of the same idea — which is how the next "
              f"teardown copies the unguarded one]")
        errors += 1
    if _BACKSTOP_SWEEP not in funnel or "pkill" not in funnel:
        print(f"FAIL  [backstop gate: {_BACKSTOP_FUNNEL}() no longer "
              f"performs the blanket sweep — deleting it converts a blunt "
              f"reap into a HANG in the degraded no-`ps` state, where "
              f"every kill_owned_pid rung refuses on an empty identity "
              f"and this is the only thing that ends the cell]")
        errors += 1
    else:
        gate_at = funnel.find(f"{_BACKSTOP_GATE} ")
        sweep_at = funnel.find(_BACKSTOP_SWEEP)
        if gate_at < 0 or gate_at > sweep_at:
            print(f"FAIL  [backstop gate: {_BACKSTOP_FUNNEL}() runs the "
                  f"blanket sweep without calling {_BACKSTOP_GATE} first "
                  f"— the sweep's ONLY confinement is the PID namespace, "
                  f"so an ungated call signals whatever matches wherever "
                  f"this shell can see it]")
            errors += 1

    # ---- (a2) the funnel is CALLED ------------------------------------
    # Spelling the sweep once inside a gated funnel says nothing if
    # nothing reaches the funnel. Measured: deleting BOTH call sites
    # turned `reap_bench_binaries` into dead code and every other
    # assertion in this arm still passed.
    for fn in ("cleanup", "run_one"):
        try:
            body = _extract_bash_function(_RUN_BENCH, fn)
        except RuntimeError as e:
            print(f"FAIL  [backstop gate: {e} — this arm cannot bind the "
                  f"sweep to a reap path without it]")
            errors += 1
            continue
        if not re.search(r"^\s*" + _BACKSTOP_FUNNEL + r"\b",
                         _sh_code_only(body), re.M):
            print(f"FAIL  [backstop gate: {fn}() does not call "
                  f"{_BACKSTOP_FUNNEL} — the funnel is dead code and "
                  f"NOTHING reaps a stray bench binary on that path, "
                  f"while every other assertion in this arm still "
                  f"passes]")
            errors += 1

    # ---- (b) the DECISION, driven under real bash ---------------------
    # The gate is split into a pure decision and a one-line read so the
    # decision can be exercised on a host with no /proc at all. Every
    # vector is PID 1's argv as some deployment would produce it, one
    # argument per line.
    try:
        gate_src = _extract_bash_function(_RUN_BENCH, _BACKSTOP_GATE)
    except RuntimeError as e:
        print(f"FAIL  [backstop gate: {e}]")
        return errors + 1
    for label, argv, want_confined in (
            ("the bench image's own CMD (the Dockerfile shape)",
             "/bin/bash\n-lc\nsource /opt/ros/jazzy/setup.bash && "
             "/bench/run_bench.sh", True),
            ("the script exec'd directly as PID 1",
             "/bench/run_bench.sh", True),
            ("the script with arguments",
             "/bin/bash\n/bench/run_bench.sh --flag", True),
            ("a bare interactive shell (`docker run … bash`, an "
             "UNSUPPORTED entry point)", "/bin/bash\n-l", False),
            ("the host's init, or a container given --pid host",
             "/sbin/init\nsplash", False),
            ("systemd", "/usr/lib/systemd/systemd", False),
            ("PID 1 argv unreadable — the property is NOT established, "
             "and must not be assumed", "", False),
            # A log file NAMED after the script must not answer for it.
            ("a process merely holding a path that STARTS with the name",
             "/usr/bin/tail\n-f\n/var/log/run_bench.sh.log", False),
            ("a word ENDING in the name but not a path component",
             "/usr/bin/notrun_bench.sh", False),
    ):
        script = (gate_src + "\n"
                  + f'if {_BACKSTOP_GATE} "$1" "$2"; then echo CONFINED; '
                    f'else echo REFUSED; fi\n')
        try:
            r = subprocess.run(["bash", "-c", script, "bash", argv,
                                "run_bench.sh"],
                               capture_output=True, text=True,
                               encoding="utf-8", errors="replace",
                               timeout=60)
        except (OSError, subprocess.SubprocessError) as e:
            print(f"FAIL  [backstop gate oracle: {label} could not be "
                  f"driven ({e})]")
            errors += 1
            continue
        got = r.stdout.strip()
        want = "CONFINED" if want_confined else "REFUSED"
        if got != want:
            print(f"FAIL  [backstop gate oracle: {label} was judged "
                  f"{got!r}, must be {want!r}. PID 1's argv naming THIS "
                  f"script is the whole claim — it says this shell is the "
                  f"init of its own process tree, which is what confines "
                  f"the blanket sweep; an argv it cannot read, or one "
                  f"belonging to somebody else's init, is NOT established "
                  f"and must refuse]")
            errors += 1

    # ---- (b3) the FUNNEL itself, driven under real bash ---------------
    # The claims above are about the decision. This one is about the
    # thing that ships: does the sweep RUN when the property holds, and
    # NOT run when it does not? Three shapes survive a text-only
    # version of this arm —
    # `if ! backstop_confined_by`, calling the gate and ignoring its
    # verdict, and a `:`-bodied `if` with the sweep after the `fi`. A
    # `find()` ordering check cannot see any of them; running the
    # function can.
    #
    # `pkill` and the argv read are stubbed, so nothing on this desk is
    # signalled and no /proc is needed.
    try:
        funnel_src = _extract_bash_function(_RUN_BENCH, _BACKSTOP_FUNNEL)
    except RuntimeError as e:
        print(f"FAIL  [backstop gate: {e}]")
        return errors + 1
    for label, argv, identity_reaped, want_swept, want_needles in (
            ("inside the bench container",
             "/bin/bash\n-lc\nsource /opt/ros/jazzy/setup.bash && "
             "/bench/run_bench.sh", "1", True, ()),
            ("on the host / under --pid host", "/sbin/init", "1", False,
             ("NOT running the blanket", "PID 1 is not this script",
              "WERE reaped by pid identity", "Strays, if any: pkill",
              "NOT a supported entry")),
            ("with PID 1's argv unreadable", "", "1", False,
             ("could not be READ", "UNKNOWN, not refuted")),
            # The refusal must not CLAIM the cell was reaped when the
            # caller says it was not: with no `ps` every kill_owned_pid
            # rung refuses before signalling, and that is precisely the
            # state somebody is in when they read this message.
            ("on the host, with nothing reaped by identity", "/sbin/init",
             "0", False,
             ("were NOT reaped by identity", "Expect a hang")),
    ):
        script = (
            'BENCH_BIN_PATTERN="bench/probe/(a|b)"\n'
            'BACKSTOP_REFUSAL_REPORTED=0\n'
            + gate_src + "\n"
            + 'pid_one_cmdline() { printf %s "$CER_ARGV"; }\n'
            # The stub writes to STDOUT and the refusal goes to STDERR,
            # so the two are read apart without opening a third fd (an
            # unopened `>&3` write would put "Bad file descriptor" onto
            # the very stream the arm inspects, and report the healthy
            # path as warning).
            + 'pkill() { echo "SWEPT $*"; return 1; }\n'
            + funnel_src + "\n"
            # TWICE. The refusal is latched once per RUN, and the funnel
            # is reached on every cell and every teardown — so a banner
            # that repeats is a per-cell flood. The latch was pinned by a
            # needle over the function SOURCE, which a variant replacing
            # the `if` with `true` walked straight past; here the second
            # call either prints again or it does not.
            + f'{_BACKSTOP_FUNNEL} "$1"; echo "RC=$?"\n'
            + f'{_BACKSTOP_FUNNEL} "$1"; echo "RC2=$?"\n')
        try:
            r = subprocess.run(
                ["bash", "-c", script + "\n", "bash", identity_reaped],
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=60,
                env=dict(os.environ, CER_ARGV=argv))
        except (OSError, subprocess.SubprocessError) as e:
            print(f"FAIL  [backstop funnel oracle: {label} could not be "
                  f"driven ({e})]")
            errors += 1
            continue
        # fd 3 is folded into stdout by bash's default; the stub writes
        # there so the operator-facing stderr can be read on its own.
        swept = "SWEPT" in r.stdout
        if swept != want_swept:
            print(f"FAIL  [backstop funnel oracle: {label} "
                  f"{'SWEPT' if swept else 'did NOT sweep'} and must "
                  f"{'sweep' if want_swept else 'NOT sweep'}. This is the "
                  f"whole of item 23: an inverted gate, a gate whose "
                  f"verdict is ignored, and a sweep moved below the `fi` "
                  f"all satisfy a text check and all reach here]")
            errors += 1
        # Once per run, whatever the funnel is asked. `want_needles` is
        # non-empty exactly on the refusal vectors.
        if want_needles:
            said = r.stderr.count("NOT running the blanket")
            if said != 1:
                print(f"FAIL  [backstop funnel oracle: {label} printed the "
                      f"refusal banner {said} time(s) across TWO calls — "
                      f"it must be latched to exactly one per run. The "
                      f"funnel is reached on every cell and every "
                      f"teardown, so a banner that repeats is a flood, and "
                      f"one that never prints is a reap that declined in "
                      f"silence]")
                errors += 1
        for needle in want_needles:
            if needle not in r.stderr:
                print(f"FAIL  [backstop funnel oracle: {label} produced no "
                      f"{needle!r} on stderr. This is asserted against the "
                      f"REAL output, not against the function's source: a "
                      f"needle searched in the body is satisfied by the "
                      f"comment that explains it, and gutting the echo "
                      f"survived exactly that way]")
                errors += 1
        if want_swept and r.stderr.strip():
            print(f"FAIL  [backstop funnel oracle: {label} swept AND "
                  f"warned ({r.stderr.strip()[:120]!r}) — the refusal "
                  f"banner must not fire on the healthy path]")
            errors += 1

    # ---- (b2b) reap_verdict, driven under real bash -------------------
    # The flag that decides whether the refusal may CLAIM this cell was
    # reaped. It was pinned by nothing, and its first version answered
    # the question with a bare `kill -0` on the pid — a PID-only liveness
    # test inside the identity-gated path, which a recycled pid answers.
    # Refused on review. The rule now is that an empty identity proves
    # nothing, so nothing is claimed; these vectors are what keep a
    # liveness probe from creeping back in as a "refinement".
    try:
        verdict_src = _extract_bash_function(_RUN_BENCH, "reap_verdict")
    except RuntimeError as e:
        print(f"FAIL  [reap verdict: {e} — the refusal banner's claim that "
              f"this cell's nodes were reaped rests on it]")
        errors += 1
    else:
        for label, args, want in (
                ("signalled, with an identity", ("0", "1", "123", "ID"), "1"),
                ("not ours / already gone, with an identity",
                 ("1", "1", "123", "ID"), "1"),
                ("ours and the signal FAILED", ("2", "1", "123", "ID"), "0"),
                # THE one this exists for: with no identity kill_owned_pid
                # returns 1 WITHOUT signalling, so rc 1 says nothing.
                ("no identity — nothing can be proven reaped",
                 ("1", "1", "123", ""), "0"),
                ("no identity, and the rung reported failure",
                 ("2", "1", "123", ""), "0"),
                ("an empty pid is nothing to reap",
                 ("1", "1", "", ""), "1"),
                ("a verdict already cleared stays cleared",
                 ("0", "0", "123", "ID"), "0"),
        ):
            script = (verdict_src + "\n"
                      + 'reap_verdict "$1" "$2" "$3" "$4"\n')
            try:
                r = subprocess.run(["bash", "-c", script, "bash", *args],
                                   capture_output=True, text=True,
                                   encoding="utf-8", errors="replace",
                                   timeout=60)
            except (OSError, subprocess.SubprocessError) as e:
                print(f"FAIL  [reap verdict oracle: {label} could not be "
                      f"driven ({e})]")
                errors += 1
                continue
            got = r.stdout.strip()
            if got != want:
                print(f"FAIL  [reap verdict oracle: {label} answered "
                      f"{got!r}, must be {want!r}. A 1 here lets the "
                      f"backstop's refusal say this cell's nodes WERE "
                      f"reaped by identity; with no identity every rung "
                      f"is a silent no-op, so that claim would be false "
                      f"in exactly the state somebody reads it in]")
                errors += 1
        # ---- the LAST rung's status must REACH the verdict -----------
        # The vectors above judge the decision; this judges whether the
        # decision is ever told the truth. The latency role's rung status
        # was discarded with `|| true` and the verdict handed a LITERAL 1,
        # so an rc 2 — "it IS ours and the signal FAILED" — published as a
        # reap that did not happen. Reproduced against the shipped
        # function: `reap_verdict 1 1 4242 ID` answers 1 where the real
        # `reap_verdict 2 1 4242 ID` answers 0.
        #
        # Stated at its real strength: a rung's status is only load-bearing
        # for the rung that DECIDES, which is each role's LAST one. The
        # graceful INT/TERM rungs above it may still discard theirs — a
        # failure there is superseded by the KILL whose status IS read —
        # and this arm says so by checking only the last.
        try:
            run_one = _sh_code_only(_extract_bash_function(_RUN_BENCH,
                                                           "run_one"))
        except RuntimeError as e:
            print(f"FAIL  [reap verdict: {e} — nothing else checks that "
                  f"the reap verdict is told what the rungs actually did]")
            errors += 1
        else:
            for role, pid_var in (("latency", "$latency_pid"),
                                  ("ping", "$ping_pid"),
                                  ("pong", "$pong_pid")):
                rung = f'kill_owned_pid "{pid_var}" "{pid_var[:-4]}_id" "" KILL'
                at = run_one.rfind(rung)
                if at < 0:
                    print(f"FAIL  [reap verdict: run_one sends the {role} "
                          f"role no KILL rung by identity — the verdict "
                          f"then rests on nothing]")
                    errors += 1
                    continue
                after = run_one[at + len(rung):at + len(rung) + 200]
                if re.match(r"[^\n]*\|\|\s*true", after):
                    print(f"FAIL  [reap verdict: the {role} role's LAST "
                          f"rung discards its status with `|| true`. That "
                          f"status is the only evidence the node was ended "
                          f"at all, and the verdict below decides whether "
                          f"the backstop's refusal may CLAIM this cell was "
                          f"reaped — so discarding it publishes a reap "
                          f"that may not have happened]")
                    errors += 1
                    continue
                # Either straight into the verdict, or through a variable
                # assigned from `$?` on the very next line (the latency
                # rung fires inside the timeout branch and its status has
                # to be carried out of it).
                #
                # TEXTUAL, and that is all it is: it sees that a capture
                # is WRITTEN and that the name appears in a later call. It
                # cannot see whether the value SURVIVES to that call —
                # `latency_kill_rc=$?` followed by `latency_kill_rc=1`
                # satisfies every clause here. Measured on the shipped
                # tree: that two-line edit left this whole checker at
                # 251 ok / 0 FAIL while a real rc 2 was published as a
                # reap. The behavioural arm below is what actually decides
                # it; this one survives to name the shape precisely when
                # it is wrong, which a value-only probe cannot.
                nxt = after.lstrip("\n").split("\n")[0].strip()
                direct = nxt.startswith("identity_reaped=$(reap_verdict \"$?\"")
                carried = re.fullmatch(r"(\w+)=\$\?", nxt)
                if not direct and not (
                        carried and f'reap_verdict "${carried.group(1)}"'
                        in run_one):
                    print(f"FAIL  [reap verdict: the {role} role's LAST "
                          f"rung is followed by {nxt!r}, which neither "
                          f"hands `$?` to reap_verdict nor records it in a "
                          f"variable the verdict is later given. A LITERAL "
                          f"there is the defect this arm exists for: it "
                          f"tells the verdict the rung succeeded whatever "
                          f"it did]")
                    errors += 1

        if "kill -0" in _sh_code_only(verdict_src):
            print(f"FAIL  [reap verdict: it probes liveness with a bare "
                  f"`kill -0`. That is a PID-only test inside the "
                  f"identity-gated path — a recycled pid answers it — and "
                  f"it is not needed: an empty identity already proves "
                  f"nothing was reaped, which is the simpler rule and the "
                  f"one this file's `node_still_ours` comment reserves "
                  f"its own fallback for]")
            errors += 1

    # ---- (b2b2) the captured status must SURVIVE to the verdict ------
    # The arm above is textual: it sees a capture written and the name
    # used later. It cannot see whether the value survives in between —
    # `latency_kill_rc=$?` followed by `latency_kill_rc=1` satisfies it,
    # and MEASURED on the shipped tree that two-line edit left the whole
    # checker at 251 ok / 0 FAIL while a real rc 2 was published as a
    # reap. So the deciding arm runs the SHIPPED LINES: the latency
    # rung's capture and the whole reap block, in order, with the
    # signalling and the backstop stubbed, and asks what flag the backstop
    # is actually handed. An overwrite, a reorder, a literal and a
    # dropped capture all change that flag; none of them can change it
    # back.
    try:
        run_body = _sh_code_only(_extract_bash_function(_RUN_BENCH, "run_one"))
        rung_at = run_body.rindex(
            'kill_owned_pid "$latency_pid" "$latency_id" "" KILL')
        # EVERYTHING from the rung to the branch's `break`, not the two
        # lines that happen to be there today. A two-line slice does not
        # execute a THIRD line inserted after the capture, so the
        # finding's own counterexample — `latency_kill_rc=$?` followed by
        # `latency_kill_rc=1` — ran outside the probe and the arm stayed
        # green. Measured before widening it.
        capture = run_body[rung_at:run_body.index("break", rung_at)]
        block_at = run_body.rindex("    local identity_reaped=1")
        block_end = run_body.index('reap_bench_binaries "$identity_reaped"',
                                   block_at)
        reap_block = run_body[block_at:block_end] + \
            'reap_bench_binaries "$identity_reaped"'
    except (RuntimeError, ValueError) as e:
        print(f"FAIL  [reap propagation: cannot slice run_one's latency "
              f"capture and reap block ({e}) — this arm drives the SHIPPED "
              f"lines, and fails closed rather than trusting the textual "
              f"check above]")
        errors += 1
    else:
        for label, kill_rc, want_flag in (
                ("a FAILED KILL on a node still ours", "2", "0"),
                ("a KILL that signalled", "0", "1"),
                ("a node already gone", "1", "1"),
        ):
            script = (
                verdict_src + "\n"
                + f'kill_owned_pid() {{ return {kill_rc}; }}\n'
                + 'node_still_ours() { return 1; }\n'
                + 'reap_bench_binaries() { echo "FLAG=$1"; }\n'
                # ping and pong are left EMPTY so only the latency role's
                # journey is observed; reap_verdict answers "nothing to
                # reap" for an empty pid whatever the rung returned.
                + 'probe() {\n'
                + '  local latency_pid=4242 latency_id=ID\n'
                + '  local ping_pid="" ping_id="" pong_pid="" pong_id=""\n'
                + '  local latency_kill_rc=1\n'
                + capture + "\n"
                + reap_block + "\n}\nprobe\n")
            try:
                r = subprocess.run(["bash", "-c", script],
                                   capture_output=True, text=True,
                                   encoding="utf-8", errors="replace",
                                   timeout=60)
            except (OSError, subprocess.SubprocessError) as e:
                print(f"FAIL  [reap propagation oracle: {label} could not "
                      f"be driven ({e})]")
                errors += 1
                continue
            got = next((ln.split("=", 1)[1] for ln in r.stdout.splitlines()
                        if ln.startswith("FLAG=")), None)
            if got != want_flag:
                print(f"FAIL  [reap propagation oracle: with {label} the "
                      f"backstop was handed FLAG={got!r}, must be "
                      f"{want_flag!r}. These are run_one's OWN lines — the "
                      f"latency rung's capture and the reap block — run in "
                      f"order with the signalling stubbed, so a capture "
                      f"that is overwritten, reordered, dropped or "
                      f"replaced by a literal shows up here as the wrong "
                      f"flag. FLAG=1 with a failed KILL is the backstop "
                      f"telling an operator this cell was reaped while a "
                      f"node is still running"
                      + (f"; stderr={r.stderr.strip()[:120]!r}"
                         if r.stderr.strip() else "") + "]")
                errors += 1

    # ---- (b2c) a FAILED final signal must not read as a reap ---------
    # The two halves composed, end to end and under real bash, which is
    # what the review asked for: a last rung that FAILED on a node still
    # ours (rc 2) must reach the verdict as "not reaped", and the
    # backstop's refusal must then say so rather than claiming the cell
    # was reaped by identity. Each half is pinned above; nothing pinned
    # the join, and the join is where the literal 1 lived.
    for label, kill_rc, want_claim in (
            ("the final KILL FAILED on a node still ours", "2",
             "were NOT reaped by identity"),
            ("the final KILL was refused for want of an identity", "1",
             "were NOT reaped by identity"),
            ("the node was already gone", "1", "WERE reaped by pid identity"),
    ):
        want_id = "" if kill_rc == "1" and "want of" in label else "ID"
        script = (
            'BENCH_BIN_PATTERN="bench/probe/(a|b)"\n'
            'BACKSTOP_REFUSAL_REPORTED=0\n'
            + verdict_src + "\n" + gate_src + "\n"
            + 'pid_one_cmdline() { printf %s "$CER_ARGV"; }\n'
            + 'pkill() { echo "SWEPT"; return 1; }\n'
            + funnel_src + "\n"
            + 'v=$(reap_verdict "$1" 1 4242 "$2")\n'
            + f'{_BACKSTOP_FUNNEL} "$v"\n')
        try:
            r = subprocess.run(
                ["bash", "-c", script, "bash", kill_rc, want_id],
                capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=60,
                env=dict(os.environ, CER_ARGV="/sbin/init"))
        except (OSError, subprocess.SubprocessError) as e:
            print(f"FAIL  [reap composition oracle: {label} could not be "
                  f"driven ({e})]")
            errors += 1
            continue
        if want_claim not in r.stderr:
            print(f"FAIL  [reap composition oracle: with {label}, the "
                  f"backstop's refusal does not say {want_claim!r} "
                  f"({r.stderr.strip()[-200:]!r}). A teardown that reports "
                  f"a reap it did not perform is the accounting this "
                  f"check exists to keep accurate, and the refusal is the "
                  f"one place an operator reads it]")
            errors += 1

    # ---- (b3b) the READ, structurally --------------------------------
    # (b3) stubs `pid_one_cmdline`, so nothing there exercises the real
    # read — and it cannot: the path is `/proc/1/cmdline`, which this desk
    # does not have. Stated at its real strength: this is a SOURCE check
    # on a one-line function, not a behavioural one. It exists because the
    # split into a pure decision and a one-line read is what makes (b3)
    # possible, and the read must keep the shape the decision expects —
    # one ARGUMENT per line, from PID 1.
    try:
        reader = _sh_code_only(
            _extract_bash_function(_RUN_BENCH, "pid_one_cmdline"))
    except RuntimeError as e:
        print(f"FAIL  [backstop gate: {e} — the decision is driven with "
              f"the read stubbed, so nothing else checks that a read "
              f"exists at all]")
        errors += 1
    else:
        for needle, why in (
                ("/proc/1/cmdline", "read PID 1's argv"),
                ("tr ", "split the NUL-separated argv into lines"),
                ("\\0", "translate the NUL separators — the decision "
                          "matches one ARGUMENT per line, and an unsplit "
                          "blob would let a path fragment answer for a "
                          "whole argument"),
        ):
            if needle not in reader:
                print(f"FAIL  [backstop gate: pid_one_cmdline does not "
                      f"{why} (no {needle!r}) — the gate's decision is "
                      f"driven against hand vectors shaped one argument "
                      f"per line, and a read that produces something else "
                      f"makes every one of them describe a shape that "
                      f"never occurs]")
                errors += 1

    # ---- (b5) the RouDi escalation is a LADDER, not an announcement ---
    # Both copies (the EXIT trap and stop_iox_roudi) TERM by identity,
    # wait a bounded grace, then escalate. Three things go wrong quietly
    # if this is not checked: the announcement can be printed on a path
    # that signals nothing (the guard falls back to a bare `kill -0` while
    # the actor refuses an empty identity); `|| true` can swallow rc 2,
    # which `kill_owned_pid`'s own header forbids because it is what keeps
    # the /dev/shm pools from being unlinked under a live daemon; and the
    # `wait` can block for good on a daemon this shell could not signal.
    for fn in ("cleanup", "stop_iox_roudi"):
        try:
            body = _sh_code_only(_extract_bash_function(_RUN_BENCH, fn))
        except RuntimeError as e:
            print(f"FAIL  [roudi escalation: {e}]")
            errors += 1
            continue
        term = 'kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi TERM'
        kill = 'kill_owned_pid "$IOX_ROUDI_PID" "$IOX_ROUDI_ID" iox-roudi KILL'
        at_term, at_kill = body.find(term), body.find(kill)
        if at_term < 0 or at_kill < 0 or at_kill < at_term:
            print(f"FAIL  [roudi escalation: {fn}() does not send RouDi a "
                  f"TERM and then a KILL by identity (TERM at {at_term}, "
                  f"KILL at {at_kill}). A bare escalation signals a pid "
                  f"recorded a whole cell earlier, as root, which the "
                  f"file's own comment says must never be sent blind]")
            errors += 1
            continue
        between = body[at_term:at_kill]
        if not re.search(r'while\s+\[\s*"\$_r_waited"\s+-lt\s+\d+', between):
            print(f"FAIL  [roudi escalation: {fn}() has no BOUNDED grace "
                  f"between RouDi's TERM and its KILL — a signal with no "
                  f"grace after it is decoration, and an unbounded one "
                  f"hangs teardown on a daemon that never exits by "
                  f"itself]")
            errors += 1
        # To the END of the function, not a fixed window: the three-way
        # `case` between the KILL and the gated `wait` is longer than 900
        # characters, and a window that stops short reports the gate
        # missing when it is there — a false FAIL, which is the safe
        # direction but still the wrong message.
        tail = body[at_kill:]
        if re.match(r"[^\n]*\|\|\s*true", tail):
            print(f"FAIL  [roudi escalation: {fn}() swallows the KILL's "
                  f"status with `|| true`. kill_owned_pid's own header "
                  f"forbids exactly that: rc 2 means the daemon IS ours "
                  f"and the signal FAILED, and it is what makes the caller "
                  f"KEEP the /dev/shm pools — reaching "
                  f"`rm -f /dev/shm/iceoryx_*` under a live RouDi is the "
                  f"corruption that arm exists to refuse]")
            errors += 1
        if not re.search(r"2\)\s*roudi_teardown_failed=1", tail):
            print(f"FAIL  [roudi escalation: {fn}() does not route the "
                  f"KILL's rc 2 to `roudi_teardown_failed=1` — a live, "
                  f"un-killable RouDi would then have its ownership "
                  f"cleared and its pools unlinked underneath it]")
            errors += 1
        if not re.search(r'if \[ "\$roudi_teardown_failed" != "1" \]; then\s*'
                         r'\n\s*wait "\$IOX_ROUDI_PID"', tail):
            print(f"FAIL  [roudi escalation: {fn}()'s `wait "
                  f"\"$IOX_ROUDI_PID\"` is not gated on "
                  f"roudi_teardown_failed — RouDi never exits by itself, "
                  f"so waiting on one this shell could not signal blocks "
                  f"teardown for good]")
            errors += 1

    # ---- (b3c) the REJECTED mechanism stays rejected ------------------
    # Keying this gate on `/proc/1/sched`'s parenthesised pid, on the
    # theory that it is reported in the INITIAL namespace, is REJECTED: the
    # kernel prints it through the pid namespace of the READER'S /proc
    # mount, so a container with its own /proc may legitimately print 1 —
    # the gate would then refuse in EVERY container, leaving the backstop
    # inert and a banner on every cell. Nothing in this repository can
    # test which way a given kernel goes, which is why the decision moved
    # to PID 1's argv: a contract this repository owns. A mention of the
    # rejected file in the shell is either that mechanism coming back or a
    # comment describing a gate that no longer works that way.
    if "/proc/1/sched" in _sh_code_only(runner):
        print("FAIL  [backstop gate: run_bench.sh names /proc/1/sched. "
              "That mechanism was rejected: the kernel reports PID 1's pid "
              "through the READER's own /proc mount, so a container with "
              "its own /proc can print 1 and the gate would refuse in "
              "every container — inert, plus a banner per cell — and "
              "nothing here can test which way a kernel goes. The gate "
              "asks whether PID 1's argv names this script instead, which "
              "is a contract this repository owns and pins]")
        errors += 1

    # ---- (b4) the Dockerfile really gives PID 1 this script -----------
    # (b3) proves the gate answers correctly for an argv; this proves the
    # argv the deployment produces is one of them. The two halves are
    # what make the gate non-inert: if the image's CMD stopped naming
    # this script, the gate would refuse in every container — quietly
    # disabling the backstop and printing a banner per cell — and the
    # only thing that could notice is a check on the CMD itself.
    dockerfile = _ROS2_BENCH_SRC.parent / "docker" / "Dockerfile"
    try:
        cmd_line = next(
            (ln for ln in dockerfile.read_text(encoding="utf-8").splitlines()
             if ln.startswith("CMD ")), None)
    except OSError as e:
        print(f"FAIL  [backstop gate: cannot read {dockerfile.name} ({e}) "
              f"— the gate's whole claim is about the argv this image "
              f"gives PID 1]")
        errors += 1
    else:
        if cmd_line is None:
            print("FAIL  [backstop gate: docker/Dockerfile has no CMD — "
                  "PID 1's argv is then whatever the caller passes, and "
                  "the backstop gate cannot be satisfied in any "
                  "container]")
            errors += 1
        elif _RUN_BENCH.name not in cmd_line:
            print(f"FAIL  [backstop gate: the image's CMD does not name "
                  f"{_RUN_BENCH.name} ({cmd_line.strip()[:90]!r}), so PID "
                  f"1's argv never names this script and "
                  f"{_BACKSTOP_GATE} refuses in EVERY container — the "
                  f"blanket backstop is then permanently inert AND every "
                  f"cell prints the refusal banner]")
            errors += 1

    if errors == 0:
        print(f"ok    [backstop gate: the blanket bench-binary sweep is "
              f"spelled once, inside {_BACKSTOP_FUNNEL}(), behind "
              f"{_BACKSTOP_GATE} — which asks whether PID 1's argv names "
              f"THIS script, the way the bench image's CMD runs it, and "
              f"refuses (loudly, naming which of the two reasons applied, "
              f"the manual command and the supported entry point) on the "
              f"host, under --pid host, under a bare `docker run … bash`, "
              f"and on an argv it cannot read]")
    return errors


# ---------------------------------------------------------------------
# The drop counters reach the published CSV row
# ---------------------------------------------------------------------

_DROP_CELL = "jazzy_probe_shm_rclcpp_be1_chrt0"

# The delivery receipt's path is declared TWICE — the writer in
# ros2/run_bench.sh, the reader in compile_csv.py — and every fixture in
# the arm below is written at the READER's own path, so the arm is
# self-consistent and proves nothing about the producer. Rename the subdir
# or the suffix on either side and `delivery_drops_from_log` answers
# "(False, None)" for every rep, every ROS 2 cell publishes the EMPTY
# column, and the meaning published for empty is "this leg writes no
# receipt". A design that refuses an under-count because it would flatter
# the row must not route its likeliest regression into the most flattering
# cell of all.
_RECEIPT_WRITER_NEEDLES = (
    ('LOGS_DIR="$CER_BENCH_RAW_DUMP_DIR/' + compile_csv.DELIVERY_LOG_SUBDIR
     + '"',
     "the subdirectory compile_csv.DELIVERY_LOG_SUBDIR names"),
    ('${CER_BENCH_RAW_NAME}_${size}_delivery.txt',
     "the <cell>_<size>_delivery.txt file name compile_csv builds"),
    ('DELIVERY role=',
     "the receipt line compile_csv greps the counters out of"),
)


def _write_drop_fixture(raw: Path, size: int, receipt: "Optional[str]",
                        samples=(10, 20, 30), dns: bool = False,
                        node_log: bool = False) -> None:
    """One rep's worth of artifacts: a .bin and, when `receipt` is not
    None, the delivery file run_bench.sh greps out of the node log.

    `dns` is the fixed100 exhausted-ladder shape — a `.rate` sidecar
    saying so and NO `.bin`, exactly as the runner leaves it."""
    raw.mkdir(parents=True, exist_ok=True)
    if node_log:
        # Only ros2/run_bench.sh writes this, and it writes one for every
        # cell it runs — so its presence is what makes an ABSENT receipt
        # mean "owed and gone" rather than "this leg writes none".
        (raw / f"{_DROP_CELL}_{size}_node.log").write_text(
            "loaned=0\n", encoding="utf-8")
    if dns:
        (raw / f"{_DROP_CELL}_{size}.rate").write_text(
            compile_csv.DID_NOT_SUSTAIN + "\n", encoding="utf-8")
    else:
        (raw / f"{_DROP_CELL}_{size}.bin").write_bytes(
            _make_blob(list(samples)))
    if receipt is not None:
        logs = raw / compile_csv.DELIVERY_LOG_SUBDIR
        logs.mkdir(parents=True, exist_ok=True)
        (logs / f"{_DROP_CELL}_{size}_delivery.txt").write_text(
            receipt, encoding="utf-8")


def _drop_row(td: str, receipt: "Optional[str]", reps=None,
              dns: bool = False, node_log: bool = False) -> "Tuple[dict, str]":
    """Drive the REAL compile_csv over a one-payload fixture and return
    (the parsed row, the stderr it wrote).

    `reps` drives the MULTI-REP path — a list of (receipt, samples) per
    rep, laid out as `<run>/rep<k>/raw/` the way bench.py's `--reps` does.
    Without it the arm only ever exercised a single rep, and the call site
    that builds the per-rep list from the run's rep dirs was reached by no
    vector at all: restricting that list to the reps that HAVE a receipt
    published a confident under-count and survived every arm.

    `dns` writes the `did_not_sustain` sidecar and NO `.bin`, so the OTHER
    row-writing path is compiled. Its column count was pinned by nothing —
    dropping its trailing commas shipped a 21-cell row under a 23-column
    header, and every arm here plus both pre-existing did_not_sustain arms
    stayed green."""
    root = Path(td)
    size = bench.PAYLOAD_SIZES[0]
    if reps is None:
        _write_drop_fixture(root / "raw", size, receipt, dns=dns,
                            node_log=node_log)
        argv = ["--raw-dir", str(root / "raw")]
    else:
        for k, (rep_receipt, samples) in enumerate(reps):
            _write_drop_fixture(root / f"rep{k}" / "raw", size, rep_receipt,
                                samples=samples, dns=dns, node_log=node_log)
        argv = ["--run-dir", str(root)]
    err = io.StringIO()
    with contextlib.redirect_stderr(err), \
            contextlib.redirect_stdout(io.StringIO()):
        compile_csv.main(argv + ["--out-dir", str(root), "--quiet",
                                 "--allow-partial"])
    text = (root / f"results_{_DROP_CELL}.csv").read_text(encoding="utf-8")
    rows = [ln for ln in text.splitlines() if not ln.startswith("#")]
    # Split by hand rather than through `csv`: no cell here is quoted or
    # holds a comma, and this keeps the module's import list — which one
    # arm reads off this file's own AST — untouched.
    head, first = rows[0].split(","), rows[1].split(",")
    if len(head) != len(first):
        raise RuntimeError(f"the published row has {len(first)} cells "
                           f"against a {len(head)}-column header")
    return (dict(zip(head, first)), err.getvalue())


# The drop-count warn, by its OWN words. A bare `"warn:" in err` is
# satisfied by compile_csv's UNRELATED sample-count warn, which every
# fixture in these arms triggers (3 samples match no schedule) — so a
# generic test is true on every vector and asserts nothing either way.
_DROPS_WARN_MARK = "drop counts cannot be totalled"

_FULL_RECEIPT = ("DELIVERY role=ping published=300\n"
                 "DELIVERY role=pong echoed=300\n"
                 "DELIVERY role=latency received=300 kicks_sent=300 "
                 "unstamped=7 nonpositive_rtt=3\n")


def _check_drop_warn(label: str, want_warn: bool, err: str) -> int:
    """Did the DROP warn fire exactly when the row published `unknown`?

    Shared by both e2e loops so neither can drift, and matched by the
    warn's own words rather than by `"warn:"` — see _DROPS_WARN_MARK."""
    if want_warn and _DROPS_WARN_MARK not in err:
        print(f"FAIL  [drop columns e2e: {label} published "
              f"'{compile_csv.DROPS_UNKNOWN}' with no explanation on "
              f"stderr ({err.strip()[:140]!r}) — a column nobody can "
              f"explain is worse than the gap it replaces]")
        return 1
    if not want_warn and _DROPS_WARN_MARK in err:
        print(f"FAIL  [drop columns e2e: {label} warned about an "
              f"untotalable count it did not have — a warn that fires on "
              f"a healthy cell is a warn nobody reads]")
        return 1
    return 0


def check_delivery_drop_columns() -> int:
    """The stamp-gate drop counts must reach the PUBLISHED row.

    They were counted at the sink and printed on its DELIVERY receipt, and
    stopped there: `compile_csv.py` and the plotters never read the
    receipt, so a cell with one sporadic drop published a row whose
    `max_ns` (or `p99_9_ns`, at small n) could carry the first log-once
    WARN's cost with nothing on the row saying a drop had occurred.

    ADDITIVE: the two columns are appended, so the historical column order
    is unchanged and every reader that resolves columns by NAME is
    unaffected. The schema carries no version field — its contract is the
    header line plus the `#` notes above it, both of which this arm
    reads."""
    errors = 0

    # ---- (a) the reader, over hand-written receipts -------------------
    with tempfile.TemporaryDirectory(prefix="drop_read_") as td:
        raw = Path(td)
        size = 64
        for label, receipt, want in (
                ("a full receipt", _FULL_RECEIPT,
                 (True, {"unstamped": 7, "nonpositive_rtt": 3})),
                ("zero drops — reported, and NOT the same as unreported",
                 "DELIVERY role=latency received=300 kicks_sent=300 "
                 "unstamped=0 nonpositive_rtt=0\n",
                 (True, {"unstamped": 0, "nonpositive_rtt": 0})),
                ("run_bench.sh's own no-receipts note",
                 "no 'DELIVERY role=' lines in x_node.log — nodes killed "
                 "before their exit prints?\n", (True, None)),
                ("a receipt from a pre-item-5 driver (no drop keys)",
                 "DELIVERY role=latency received=300 kicks_sent=300\n",
                 (True, None)),
                ("only ONE of the two keys",
                 "DELIVERY role=latency received=300 unstamped=4\n",
                 (True, None)),
                ("a corrupted value — fails CLOSED, never to a plausible "
                 "number",
                 "DELIVERY role=latency unstamped=7junk nonpositive_rtt=3\n",
                 (True, None)),
                # `str.isdigit()` is TRUE for Unicode digit forms `int()`
                # refuses, so this raised ValueError out of the reader and
                # took the whole compile down with a traceback — a crash,
                # not the fail-closed the reader's contract promises.
                ("a Unicode digit form int() refuses",
                 "DELIVERY role=latency unstamped=\u00b2 nonpositive_rtt=3\n",
                 (True, None)),
                ("a SUFFIX key must not answer for the real one",
                 "DELIVERY role=latency retries_unstamped=7 "
                 "nonpositive_rtt=3\n", (True, None)),
                ("another role's line only",
                 "DELIVERY role=ping published=300 unstamped=7 "
                 "nonpositive_rtt=3\n", (True, None)),
        ):
            _write_drop_fixture(raw, size, receipt)
            try:
                got = compile_csv.delivery_drops_from_log(raw, _DROP_CELL,
                                                          size)
            except Exception as e:                        # noqa: BLE001
                # REPORTED, never allowed to propagate. The reader's
                # contract is that junk fails CLOSED to "could not read",
                # and a reader that RAISES on junk does not merely miss
                # the case — it takes the whole compile down with a
                # traceback, and here it would take every later arm with
                # it and read as a broken tool rather than as the
                # regression it is. Measured: reverting the reader's
                # ASCII check let a Unicode digit form raise ValueError
                # out of this call.
                print(f"FAIL  [drop columns reader: {label} made the "
                      f"reader raise {type(e).__name__} ({e}) instead of "
                      f"returning a verdict. Junk on a receipt must fail "
                      f"CLOSED to (True, None); raising loses the row, "
                      f"every later row, and every later arm]")
                errors += 1
                continue
            if got != want:
                print(f"FAIL  [drop columns reader: {label} read as {got!r}, "
                      f"expected {want!r}. 'the sink reported no drops' and "
                      f"'the sink never reported' are opposite claims and "
                      f"must not collapse onto one another]")
                errors += 1
        # ...and the two ABSENT cases, which are opposite claims and are
        # told apart by whether the receipt-WRITING driver ran here. Only
        # ros2/run_bench.sh writes the per-cell node log, so its presence
        # is the evidence that a receipt was OWED.
        (raw / compile_csv.DELIVERY_LOG_SUBDIR /
         f"{_DROP_CELL}_{size}_delivery.txt").unlink()
        got = compile_csv.delivery_drops_from_log(raw, _DROP_CELL, size)
        if got != (False, None):
            print(f"FAIL  [drop columns reader: a cell with NO delivery "
                  f"receipt and no node log read as {got!r}, expected "
                  f"(False, None) — a native or workspace leg writes "
                  f"neither, and its column must be empty rather than "
                  f"unknown]")
            errors += 1
        (raw / f"{_DROP_CELL}_{size}_node.log").write_text("loaned=0\n",
                                                           encoding="utf-8")
        got = compile_csv.delivery_drops_from_log(raw, _DROP_CELL, size)
        if got != (True, None):
            print(f"FAIL  [drop columns reader: a cell whose ROS 2 node "
                  f"log is present but whose receipt is GONE read as "
                  f"{got!r}, expected (True, None). Blank publishes 'no "
                  f"receipt was owed here', so a cell that LOST its "
                  f"accounting would read as a valid row with the "
                  f"question marked inapplicable — reproduced on review "
                  f"with an executed harness: two contributing ROS 2 reps "
                  f"with samples and no receipts compiled to blank cells "
                  f"and exit 0]")
            errors += 1

    # ---- (b) the cross-rep aggregation, over hand vectors -------------
    F = {"unstamped": 7, "nonpositive_rtt": 3}
    G = {"unstamped": 1, "nonpositive_rtt": 0}
    for label, per_rep, want, want_reason in (
            ("one rep", [("rep0", True, F)], {"unstamped": "7",
                                              "nonpositive_rtt": "3"}, None),
            ("two reps POOL, exactly as iterations does",
             [("rep0", True, F), ("rep1", True, G)],
             {"unstamped": "8", "nonpositive_rtt": "3"}, None),
            ("no receipt anywhere — the column does not apply",
             [("rep0", False, None), ("rep1", False, None)],
             {"unstamped": "", "nonpositive_rtt": ""}, None),
            ("no reps at all", [], {"unstamped": "",
                                    "nonpositive_rtt": ""}, None),
            ("one rep answered and one did not — a sum over the reps that "
             "answered is an UNDER-count wearing a total's clothes",
             [("rep0", True, F), ("rep1", False, None)],
             {"unstamped": "unknown", "nonpositive_rtt": "unknown"},
             "silent"),
            ("one rep's receipt is unparseable",
             [("rep0", True, F), ("rep1", True, None)],
             {"unstamped": "unknown", "nonpositive_rtt": "unknown"},
             "unreadable"),
            ("all-zero across reps stays a REPORTED zero",
             [("rep0", True, {"unstamped": 0, "nonpositive_rtt": 0}),
              ("rep1", True, {"unstamped": 0, "nonpositive_rtt": 0})],
             {"unstamped": "0", "nonpositive_rtt": "0"}, None),
    ):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            got = compile_csv.delivery_drops_across_reps(per_rep, "c", 64)
        wrote = err.getvalue()
        if got != want:
            print(f"FAIL  [drop columns aggregation: {label} gave {got!r}, "
                  f"expected {want!r}]")
            errors += 1
        elif want["unstamped"] != compile_csv.DROPS_UNKNOWN:
            # The must-NOT-warn half. Without it a reporter that warns on
            # EVERY call satisfies every "wrote a warn" verdict above and
            # the arm reads as two checks while being one.
            if wrote.strip():
                print(f"FAIL  [drop columns aggregation: {label} produced "
                      f"a totalled answer AND warned "
                      f"({wrote.strip()[:100]!r}) — a warn that fires on a "
                      f"healthy cell is a warn nobody reads]")
                errors += 1
        else:
            # The REASON, not merely that something was written. The two
            # reasons send an investigator to different places (a rep that
            # produced no receipt at all vs one whose receipt would not
            # parse), and this file already pins per-arm phrases for the
            # sink drop arms "so the two arms' messages cannot be swapped
            # without notice". Measured: swapping the two labels, and
            # gutting the message to `warn: unknown`, both survived a bare
            # `"warn:" in` check.
            want_phrase = ("no receipt in" if want_reason == "silent"
                           else "no parseable count in")
            if "warn:" not in wrote:
                print(f"FAIL  [drop columns aggregation: {label} wrote "
                      f"'{compile_csv.DROPS_UNKNOWN}' onto the row in "
                      f"SILENCE — a column nobody can explain is worse "
                      f"than the gap it replaces]")
                errors += 1
            elif want_phrase not in wrote:
                print(f"FAIL  [drop columns aggregation: {label} wrote a "
                      f"warn that does not say {want_phrase!r} "
                      f"({wrote.strip()[:140]!r}) — the two reasons are "
                      f"investigated differently, so their texts must not "
                      f"be interchangeable]")
                errors += 1
            elif "c" not in wrote or "64" not in wrote:
                print(f"FAIL  [drop columns aggregation: {label}'s warn "
                      f"names neither the cell nor the payload "
                      f"({wrote.strip()[:140]!r}) — an operator cannot act "
                      f"on a warn that does not say which row it is "
                      f"about]")
                errors += 1

    # ---- (c) the published header ------------------------------------
    header = compile_csv.CSV_HEADER.strip().split(",")
    for key in compile_csv.DELIVERY_DROP_KEYS:
        if key not in header:
            print(f"FAIL  [drop columns header: the published CSV header "
                  f"carries no {key!r} column — every arm below resolves "
                  f"the row by NAME, so without it they compare None with "
                  f"None and pass having checked nothing]")
            errors += 1
    if header[:21] != ("payload_bytes,iterations,round_trip_p50_ns,"
                       "round_trip_p99_ns,round_trip_mean_ns,one_way_p50_ns,"
                       "chrt,loaned,floor_ns,p1_ns,p10_ns,p25_ns,p75_ns,"
                       "p90_ns,p95_ns,p99_9_ns,max_ns,rep_count,"
                       "rep_p50_min_ns,rep_p50_max_ns,"
                       "achieved_rate_hz").split(","):
        print(f"FAIL  [drop columns header: the historical 21 columns are "
              f"no longer first ({header[:21]}) — the two counters were "
              f"appended precisely so no existing reader moves, and an "
              f"index-based reader elsewhere in this tree reads "
              f"cells[0..2]]")
        errors += 1
    if errors:
        return errors

    # ---- (d) end to end, through the real compile_csv -----------------
    for label, receipt, want, want_warn in (
            ("a receipt with drops", _FULL_RECEIPT,
             {"unstamped": "7", "nonpositive_rtt": "3"}, False),
            ("no receipt at all (a native / workspace leg)", None,
             {"unstamped": "", "nonpositive_rtt": ""}, False),
            ("a receipt the sink never printed",
             "no 'DELIVERY role=' lines in x_node.log\n",
             {"unstamped": "unknown", "nonpositive_rtt": "unknown"}, True),
    ):
        with tempfile.TemporaryDirectory(prefix="drop_e2e_") as td:
            try:
                row, err = _drop_row(td, receipt)
            except Exception as e:                        # noqa: BLE001
                print(f"FAIL  [drop columns e2e: {label} — compile_csv "
                      f"raised {type(e).__name__} ({e})]")
                errors += 1
                continue
        got = {k: row.get(k) for k in want}
        if got != want:
            print(f"FAIL  [drop columns e2e: {label} published {got!r}, "
                  f"expected {want!r} — the counters are on the sink's "
                  f"receipt and the row is what anybody reads]")
            errors += 1
        errors += _check_drop_warn(label, want_warn, err)

    # ---- (d2) the OTHER row-writing path, and the MULTI-REP one -------
    # Neither is reached by any existing vector, so a variant of either
    # survives every arm in this file. `_drop_row` raises on a header/row width
    # mismatch, so the did_not_sustain case needs only the fixture.
    for label, kwargs, want in (
            ("a did_not_sustain row (no .bin, ladder exhausted)",
             dict(receipt=_FULL_RECEIPT, dns=True),
             {"unstamped": "", "nonpositive_rtt": "",
              "achieved_rate_hz": compile_csv.DID_NOT_SUSTAIN}),
            ("two reps that BOTH report — pooled, like iterations",
             dict(receipt=None,
                  reps=[(_FULL_RECEIPT, (10, 20, 30)),
                        (_FULL_RECEIPT.replace("unstamped=7", "unstamped=5")
                                      .replace("nonpositive_rtt=3",
                                               "nonpositive_rtt=1"),
                         (11, 21, 31))]),
             {"unstamped": "12", "nonpositive_rtt": "4", "rep_count": "2"}),
            ("two reps, one with NO receipt — a sum over the rep that "
             "answered is an under-count, so the row refuses",
             dict(receipt=None,
                  reps=[(_FULL_RECEIPT, (10, 20, 30)),
                        (None, (11, 21, 31))]),
             {"unstamped": "unknown", "nonpositive_rtt": "unknown",
              "rep_count": "2"}),
            # The review reproduction, verbatim: two contributing ROS 2
            # reps with samples and NO receipts at all. It compiled to
            # blank cells and exit 0 — blank meaning "no receipt was owed
            # here", so a cell that lost its whole accounting published a
            # valid-looking row with the question marked inapplicable.
            ("two ROS 2 reps whose receipts are ALL gone",
             dict(receipt=None, node_log=True,
                  reps=[(None, (10, 20, 30)), (None, (11, 21, 31))]),
             {"unstamped": "unknown", "nonpositive_rtt": "unknown",
              "rep_count": "2"}),
            # ...and the control that keeps it from becoming "always
            # unknown": the same shape with NO node log is a native or
            # workspace leg, and its column really does not apply.
            ("two reps with no receipts and no node log (a native leg)",
             dict(receipt=None,
                  reps=[(None, (10, 20, 30)), (None, (11, 21, 31))]),
             {"unstamped": "", "nonpositive_rtt": "", "rep_count": "2"}),
    ):
        with tempfile.TemporaryDirectory(prefix="drop_e2e2_") as td:
            try:
                row, err = _drop_row(td, **kwargs)
            except Exception as e:                        # noqa: BLE001
                print(f"FAIL  [drop columns e2e: {label} — compile_csv "
                      f"raised {type(e).__name__} ({e}). A width mismatch "
                      f"between the header and a row is reported here: "
                      f"the did_not_sustain path writes its own cells and "
                      f"derives its padding from DELIVERY_DROP_KEYS]")
                errors += 1
                continue
        got = {k: row.get(k) for k in want}
        if got != want:
            print(f"FAIL  [drop columns e2e: {label} published {got!r}, "
                  f"expected {want!r}]")
            errors += 1
        # ITS OWN expectation, derived from what this vector's row
        # publishes. It read `want_warn` — a name THIS loop never binds,
        # left over from the loop above — so both halves were evaluated
        # against that loop's LAST vector: the positive half always held
        # (an unrelated warn is always present) and the negative half
        # never ran at all. The second leaked loop variable in this PR;
        # an AST sweep over every arm added here found no others.
        errors += _check_drop_warn(
            label, want.get("unstamped") == compile_csv.DROPS_UNKNOWN, err)

    # ---- (e) the PRODUCER writes where the reader looks ---------------
    try:
        runner = _RUN_BENCH.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [drop columns path: cannot read run_bench.sh ({e}) — "
              f"this arm fails closed rather than assuming the receipt is "
              f"written where compile_csv looks for it]")
        errors += 1
    else:
        for needle, what in _RECEIPT_WRITER_NEEDLES:
            if needle not in runner:
                print(f"FAIL  [drop columns path: run_bench.sh does not "
                      f"spell {needle!r} — it no longer writes {what}. "
                      f"compile_csv then finds no receipt for any rep and "
                      f"publishes the EMPTY column, whose documented "
                      f"meaning is 'this leg writes none' — silently, on "
                      f"every ROS 2 cell, forever. Every fixture in this "
                      f"arm is written at the READER's own path, so "
                      f"nothing else here can see it]")
                errors += 1

    # ---- (f) the CSV notes describe every column ----------------------
    # The published `#` notes are the schema's contract for a human, and
    # they are hand-written prose beside a derived header: a third counter
    # would ship with the header updated and the notes describing two of
    # three columns.
    for key in compile_csv.DELIVERY_DROP_KEYS:
        if key not in compile_csv.CSV_COMMENT:
            print(f"FAIL  [drop columns: the published CSV `#` notes do "
                  f"not mention {key!r}. The header is derived from "
                  f"DELIVERY_DROP_KEYS and the notes are not, so they go "
                  f"stale silently — and the notes are where a reader "
                  f"learns what an empty cell and an "
                  f"'{compile_csv.DROPS_UNKNOWN}' cell mean]")
            errors += 1

    if errors == 0:
        print(f"ok    [drop columns: {', '.join(compile_csv.DELIVERY_DROP_KEYS)} "
              f"are read off the cell's DELIVERY receipt, pooled across "
              f"reps like iterations, and published on the row — appended, "
              f"so the historical 21 columns do not move; empty where no "
              f"receipt exists and '{compile_csv.DROPS_UNKNOWN}' (loudly) "
              f"where one was expected and could not be totalled]")
    return errors


# ---------------------------------------------------------------------
# The shell entry points are documented as unsupported
# ---------------------------------------------------------------------

_UNSUPPORTED_ENTRY_MARK = "NOT A SUPPORTED ENTRY POINT"


def check_shell_entry_points_are_unsupported() -> int:
    """Both runner scripts, and the README, must say that a direct
    invocation is not a supported entry point.

    The alternative was a SECOND copy of the freshness policy in shell,
    and the decision is that there is not one: bench.py owns it, the scripts
    say so, and neither silently substitutes a weaker rule. A docs claim
    is the whole of the fix here, so it is pinned like any other — an
    undocumented entry point is the state this closes."""
    errors = 0
    for path, where in ((_RUN_BENCH, "ros2/run_bench.sh"),
                        (_RUN_WORKSPACE, "workspace/run_workspace.sh")):
        try:
            head = path.read_text(encoding="utf-8").split("\n\n", 1)[0]
        except OSError as e:
            print(f"FAIL  [entry point docs: cannot read {where} ({e})]")
            errors += 1
            continue
        if _UNSUPPORTED_ENTRY_MARK not in head:
            print(f"FAIL  [entry point docs: {where}'s header banner does "
                  f"not say {_UNSUPPORTED_ENTRY_MARK!r}. A reader who runs "
                  f"it directly gets none of bench.py's refusals and is "
                  f"told nothing, which is the state the entry-point rule "
                  f"closes, and the header is the one place such a reader "
                  f"is certain to look]")
            errors += 1
        elif "bench.py" not in head:
            print(f"FAIL  [entry point docs: {where}'s banner says the "
                  f"script is unsupported without naming bench.py as the "
                  f"supported path — a refusal with no alternative sends "
                  f"the reader nowhere]")
            errors += 1
    try:
        doc = _BENCH_README.read_text(encoding="utf-8")
    except OSError as e:
        print(f"FAIL  [entry point docs: cannot read "
              f"{_BENCH_README.name} ({e})]")
        return errors + 1
    # ANCHORED to the block that prints the direct recipes, not searched
    # over the whole README. A file-wide search is satisfied by text that
    # was already there: `one container per` occurs in the pre-existing
    # heading "ROS 2 cells (Docker, one container per cell)", so that
    # needle passed on the unchanged README and deleting the entire new
    # paragraph left this arm green — measured. The recipes are what a
    # reader follows, so the statement has to be where they are.
    _RECIPE_MARK = "bash workspace/run_workspace.sh split"
    at = doc.find(_RECIPE_MARK)
    if at < 0:
        print(f"FAIL  [entry point docs: the README no longer prints the "
              f"direct recipe {_RECIPE_MARK!r} — this arm anchors the "
              f"support statement on that block and cannot place it "
              f"without one]")
        errors += 1
    else:
        # From the heading above the recipes to the recipe itself.
        window = doc[max(0, at - 1600):at].lower()
        for needle, why in (
                ("not a supported entry point",
                 "say so where the direct recipes are given"),
                # Absent from the README before this change, unlike
                # "one container per cell", which is a pre-existing
                # section heading elsewhere in the file.
                ("--pid host",
                 "name the deployment property run_bench.sh's blanket "
                 "backstop rests on"),
                ("bench.py",
                 "name the supported entry point beside the recipes"),
        ):
            if needle.lower() not in window:
                print(f"FAIL  [entry point docs: the README does not {why} "
                      f"(no {needle!r} in the 1600 characters before the "
                      f"direct recipes). The recipes are printed there in "
                      f"full, so a reader who follows them learns from "
                      f"that block or not at all]")
                errors += 1
    # ...and the CERULION override section BELOW the recipes, which is the
    # other route a reader takes to the same fact: it already said a
    # direct invocation "is not covered", and the decision is that it is not
    # supported. Both statements, or a reader who arrives by the second
    # route learns only that a guard does not reach them.
    if at >= 0 and "not a supported entry point" not in doc[at:]:
        print("FAIL  [entry point docs: the README states the support rule at "
              "the recipes but not in the `CERULION` override section "
              "below them, which is the other route a reader takes to the "
              "same question — it tells them the freshness guard does not "
              "reach a direct invocation, and must also say that such an "
              "invocation is not supported]")
        errors += 1
    if errors == 0:
        print("ok    [entry point docs: both runner scripts open by saying "
              "a direct invocation is not a supported entry point and "
              "naming bench.py, and the README says it beside the direct "
              "recipes with the one-container-per-cell property that "
              "confines the blanket backstop]")
    return errors


def check_cerulion_binary_freshness() -> int:
    """A stale `cerulion` must not validate this checkout.

    PR #733's review validated bench graphs with a binary from another
    tree: 14 checks where this engine reports 16, and the difference was
    the bug. The resolver's docstring already said "THIS tree's,
    deliberately" while the code took the first of target/debug,
    target/release, PATH and asked nothing about which source built it.

    Driven several ways, because each catches something the others cannot:
    a hand-written table over the PURE classifier; the REAL resolver over
    a temp tree holding a stale fixture and a fresh control; and the real
    CERULION= refusal, whose call sites are then read out of bench.py's
    own text so the guard cannot ship orphaned."""
    errors = 0
    HOUR = 3_600_000_000_000  # ns
    floor = 1_000_000_000_000_000_000

    # ---- the pure classifier, against a hand-written table ---------------
    IN, OUT, UNK = (bench.PROV_IN_TREE, bench.PROV_FOREIGN,
                    bench.PROV_UNKNOWN)
    table = [
        # (provenance, mtime, floor, verdict, a word the reason must carry)
        (IN,  floor + HOUR, floor, bench.CLI_FRESH, None),
        (IN,  floor,        floor, bench.CLI_FRESH, None),   # equal = fresh
        (IN,  floor - 1,    floor, bench.CLI_STALE, "older"),
        (IN,  floor - HOUR, floor, bench.CLI_STALE, "older"),
        (OUT, floor + HOUR, floor, bench.CLI_FOREIGN, "not this checkout"),
        (OUT, None,         floor, bench.CLI_FOREIGN, "not this checkout"),
        # PROVENANCE is asked FIRST: an unknown-origin binary is undatable
        # however fresh it looks, because "newer than this tree's source"
        # says nothing about WHICH tree built it.
        (UNK, floor + HOUR, floor, bench.CLI_UNVERIFIABLE, "CARGO_TARGET_DIR"),
        (IN,  None,         floor, bench.CLI_UNVERIFIABLE, "modification"),
        (IN,  floor + HOUR, None,  bench.CLI_UNVERIFIABLE, "git"),
    ]
    covered = {row[3] for row in table}
    if covered != set(bench.CLI_VERDICTS):
        print(f"FAIL  [cli freshness classifier: the table covers {covered}, "
              f"but the verdict vocabulary is {set(bench.CLI_VERDICTS)} — a "
              f"verdict nothing drives is a verdict nothing pins]")
        errors += 1
    # The same assertion for the PROVENANCE vocabulary. Without it
    # PROV_ANSWERS was declared and never read — dead by the repo's own
    # rule, and a promise ("so an oracle can assert it covers every
    # answer") that nothing kept.
    provenances = {row[0] for row in table}
    if provenances != set(bench.PROV_ANSWERS):
        print(f"FAIL  [cli freshness classifier: the table drives "
              f"{provenances}, but the provenance vocabulary is "
              f"{set(bench.PROV_ANSWERS)} — an answer nothing drives is an "
              f"answer nothing pins]")
        errors += 1
    for prov, mtime, flr, want, needle in table:
        verdict, reason = bench.classify_cerulion_binary(prov, mtime, flr)
        if verdict != want:
            print(f"FAIL  [cli freshness classifier: provenance={prov} "
                  f"mtime={mtime} floor={flr} => {verdict!r}, want {want!r}]")
            errors += 1
        elif needle is None and reason != "":
            print(f"FAIL  [cli freshness classifier: a fresh binary must "
                  f"carry no complaint, got {reason!r}]")
            errors += 1
        elif needle is not None and needle not in reason:
            print(f"FAIL  [cli freshness classifier: {verdict} reason "
                  f"{reason!r} does not say {needle!r}]")
            errors += 1
    # The stale reason must carry HOW stale, in seconds — an operator
    # cannot act on "stale" alone.
    _v, stale_reason = bench.classify_cerulion_binary(
        bench.PROV_IN_TREE, floor - 2 * HOUR, floor)
    if "7200 s" not in stale_reason:
        print(f"FAIL  [cli freshness classifier: a 2-hour-stale binary must "
              f"report '7200 s', got {stale_reason!r}]")
        errors += 1
    # A SUB-SECOND delta is real staleness (a rebuild lands well inside a
    # second), so the line an operator acts on must not round it away to
    # "0 s older" and read as an artefact.
    _v, sub_second = bench.classify_cerulion_binary(bench.PROV_IN_TREE, floor - 1, floor)
    if "0 s older" in sub_second or "1 s older" not in sub_second:
        print(f"FAIL  [cli freshness classifier: a sub-second-stale binary "
              f"must report '1 s older', never '0 s older', got "
              f"{sub_second!r}]")
        errors += 1

    # ---- the FLOOR itself: partial evidence is not evidence --------------
    # HEAD's commit time alone cannot see a local edit, so a floor built
    # without the tracked-source term would clear a binary built before
    # that edit and call it FRESH — confidently, which is the very failure
    # this check exists to close. Both degraded shapes must answer None,
    # which classify_cerulion_binary then reports as CLI_UNVERIFIABLE.
    saved_sources = bench._tracked_cli_sources
    saved_cache = bench._CLI_SOURCE_FLOOR_CACHE
    try:
        for label, stub in (("git cannot answer", lambda: None),
                            ("git names nothing", lambda: []),
                            ("nothing can be stat'd",
                             lambda: ["no/such/file.rs"])):
            bench._tracked_cli_sources = stub
            bench._CLI_SOURCE_FLOOR_CACHE = None
            got = bench.cerulion_source_floor_ns()
            if got is not None:
                print(f"FAIL  [cli source floor: with {label} the floor must "
                      f"be None (undatable), got {got!r} — a partial floor "
                      f"mints a confident 'fresh']")
                errors += 1
            elif bench.classify_cerulion_binary(bench.PROV_IN_TREE, 1, got)[0] != \
                    bench.CLI_UNVERIFIABLE:
                print(f"FAIL  [cli source floor: with {label} the classifier "
                      f"must answer {bench.CLI_UNVERIFIABLE}]")
                errors += 1
        # The positive control: with a real source term the floor IS the
        # newest of them, so the arms above are not passing on a function
        # that always answers None.
        bench._CLI_SOURCE_FLOOR_CACHE = None
        bench._tracked_cli_sources = saved_sources
        live = bench.cerulion_source_floor_ns()
        try:
            in_work_tree = subprocess.run(
                ["git", "rev-parse", "--is-inside-work-tree"],
                cwd=str(bench.REPO_ROOT), capture_output=True, text=True,
                timeout=30).stdout.strip() == "true"
        except (OSError, subprocess.SubprocessError):
            # No git at all. Every other environment dependency in this
            # file degrades to a loud skip, and a git-less host is exactly
            # the condition the code under test models as UNVERIFIABLE — so
            # dying here would kill the remaining checks in the scenario
            # the feature exists for.
            in_work_tree = False
        if not in_work_tree or live is None:
            # A `git archive` export, or a tarball. Every other environment
            # dependency in this file degrades to a loud skip (no docker, no
            # matplotlib, no compiler, no `cerulion`), and the code under
            # test treats an undatable tree as UNVERIFIABLE rather than as
            # an error — so failing here would be this file holding a
            # stricter policy than the policy it is testing.
            # UNDATABLE is a supported degrade, not a failure: no git, a
            # sparse checkout, or a source file removed with `rm` rather
            # than `git rm` all make the walk partial BY DESIGN. Failing
            # here would hold this file to a stricter policy than the
            # policy it is testing.
            print("skip  [cli source floor positive control: this tree "
                  "cannot date itself (not a git work tree, or the source "
                  "walk is partial) — which is the undatable case, not a "
                  "failure]")
        elif not isinstance(live, int) or live <= 0:
            print(f"FAIL  [cli source floor: a datable tree must produce a "
                  f"positive floor, got {live!r}]")
            errors += 1
    finally:
        bench._tracked_cli_sources = saved_sources
        bench._CLI_SOURCE_FLOOR_CACHE = saved_cache

    # ---- the CACHE: computed once, and an ABSENCE never cached ---------
    # The one-tuple sentinel exists solely to tell "undatable" from "not
    # computed yet", and not caching a None is what stops one transient
    # `git` failure at start-up from leaving a whole process undatable.
    # Neither was observable: every other arm resets the cache before each
    # call, so all three variants (store the bare value, never cache, cache
    # the absence) run green.
    saved_sources2 = bench._tracked_cli_sources
    saved_cache2 = bench._CLI_SOURCE_FLOOR_CACHE
    saved_root_cache = bench.REPO_ROOT
    cache_td = tempfile.TemporaryDirectory(prefix="cache_")
    td_cache = cache_td.name
    try:
        calls = []

        def counting_real():
            calls.append(1)
            return saved_sources2()

        def counting_none():
            calls.append(1)
            return None

        bench._tracked_cli_sources = counting_real
        bench._CLI_SOURCE_FLOOR_CACHE = None
        first = bench.cerulion_source_floor_ns()
        second = bench.cerulion_source_floor_ns()
        if first is None:
            print("skip  [cli source floor cache: this tree cannot date "
                  "itself, so there is no floor to cache]")
        elif second != first or len(calls) != 1:
            print(f"FAIL  [cli source floor cache: a real floor must be "
                  f"computed ONCE and re-served ({first!r} then {second!r}, "
                  f"{len(calls)} walk(s)) — it stats a thousand-odd files]")
            errors += 1
        for label, stub in (("git could not answer", counting_none),
                            ("the floor was clamped as future-dated",
                             None)):
            calls.clear()
            if stub is None:
                # The OTHER way to reach an undatable answer: the walk
                # succeeds and usable_source_floor clamps it. That path
                # runs PAST the early return, so it is the only one that
                # can observe a cache write placed on the wrong side of
                # the None check — the shape a variant reaches.
                fut = Path(td_cache) / "future.rs"
                fut.write_text("// future\n", encoding="utf-8")
                ahead = (time.time_ns() +
                         4 * bench.CLI_SOURCE_FLOOR_FUTURE_GRACE_NS)
                os.utime(fut, ns=(ahead, ahead))
                saved_root2 = bench.REPO_ROOT
                bench.REPO_ROOT = Path(td_cache)

                def counting_future():
                    calls.append(1)
                    return ["future.rs"]

                bench._tracked_cli_sources = counting_future
            else:
                bench._tracked_cli_sources = stub
            bench._CLI_SOURCE_FLOOR_CACHE = None
            got_a = bench.cerulion_source_floor_ns()
            got_b = bench.cerulion_source_floor_ns()
            if stub is None:
                bench.REPO_ROOT = saved_root2
            if got_a is not None or got_b is not None:
                print(f"FAIL  [cli source floor cache: with {label} the "
                      f"answer must be None both times, got {got_a!r} / "
                      f"{got_b!r}]")
                errors += 1
            elif len(calls) != 2:
                print(f"FAIL  [cli source floor cache: an UNDATABLE answer "
                      f"must not be cached ({label}) — one transient "
                      f"failure would then leave every later call in the "
                      f"process undatable ({len(calls)} walk(s) for 2 "
                      f"calls)]")
                errors += 1
    finally:
        bench._tracked_cli_sources = saved_sources2
        bench._CLI_SOURCE_FLOOR_CACHE = saved_cache2
        bench.REPO_ROOT = saved_root_cache
        cache_td.cleanup()

    # A floor in the FUTURE is not "everything is stale" — it is a tree
    # whose clocks are not evidence. Answering stale there refuses every
    # binary anyone could build.
    G = bench.CLI_SOURCE_FLOOR_FUTURE_GRACE_NS
    for label, flr, now, want in (
            ("a floor in the past", 100, 1000, 100),
            ("a floor within the grace", 1000 + 5, 1000, 1005),
            # THE boundary, both sides at one nanosecond. Read from bench
            # so the rows cannot drift from the shipped constant.
            ("a floor exactly at the grace edge", 1000 + G, 1000, 1000 + G),
            ("a floor one ns past the grace edge", 1000 + G + 1, 1000, None),
            ("a floor far in the future", 10**18, 0, None),
            ("no floor at all", None, 1000, None)):
        got = bench.usable_source_floor(flr, now)
        if got != want:
            print(f"FAIL  [cli source floor: {label} => {got!r}, want "
                  f"{want!r}]")
            errors += 1
    if errors == 0:
        print(f"ok    [cli freshness classifier: {len(table)} hand oracles "
              f"agree, and a stale verdict says how far behind]")

    # ---- the SOURCE SET, over a real git fixture -------------------------
    # The exclusion list carries a stated rationale (benches/, demos/ and
    # examples/ are their own cargo workspaces, so including them would
    # move the floor every time someone edits a bench script and refuse a
    # CLI built minutes earlier). Nothing drove it, so an exclusion that
    # stopped excluding — or a walk that stopped seeing .rs — would be
    # invisible.
    if shutil.which("git") is None:
        print("skip  [cli source set: no `git` on PATH]")
    else:
        saved_root = bench.REPO_ROOT
        saved_cache = bench._CLI_SOURCE_FLOOR_CACHE
        saved_ctd_src = os.environ.pop("CARGO_TARGET_DIR", None)
        # This fixture tree has no `cerulion_cli`, so every walk in it
        # falls back to the whole workspace and SPENDS the once-per-process
        # "could not narrow" note. Without this save/restore a genuine
        # fallback later in the same run would be silent — which is the one
        # thing that note exists to prevent.
        saved_note_src = bench._CLI_CLOSURE_NOTE_SHOWN
        try:
            with tempfile.TemporaryDirectory(prefix="src_") as td:
                repo = Path(td) / "repo"
                (repo / "src").mkdir(parents=True)
                (repo / "benches" / "x").mkdir(parents=True)
                (repo / "docs").mkdir(parents=True)
                (repo / "Cargo.toml").write_text("[workspace]\n")
                (repo / "Cargo.lock").write_text("version = 3\n")
                (repo / "src" / "lib.rs").write_text("// cli source\n")
                (repo / "benches" / "x" / "b.rs").write_text("// a bench\n")
                (repo / "docs" / "readme.md").write_text("# docs\n")
                git = ["git", "-c", "user.email=b@e", "-c", "user.name=b",
                       "-c", "commit.gpgsign=false"]

                def run_git(argv):
                    """Fixture construction only. Kept in its own helper so
                    the `except` that turns a build failure into a `skip`
                    can wrap THIS and not the assertions below — a broad
                    except there reported "fixture could not be built"
                    while four arms silently never ran, and the function
                    still printed its closing `ok`."""
                    try:
                        r = subprocess.run(git + argv, cwd=str(repo),
                                           capture_output=True, text=True,
                                           timeout=60)
                    except (OSError, subprocess.SubprocessError) as e:
                        raise RuntimeError(f"git {argv[0]}: {e}")
                    if r.returncode != 0:
                        raise RuntimeError(
                            f"git {argv[0]} rc={r.returncode}: "
                            f"{(r.stdout + r.stderr).strip()[-200:]}")

                for argv in (["init", "-q"], ["add", "-A"],
                             ["commit", "-q", "-m", "fixture"]):
                    run_git(argv)
                bench.REPO_ROOT = repo
                bench._CLI_SOURCE_FLOOR_CACHE = None
                named = set(bench._tracked_cli_sources() or [])
                want = {"Cargo.toml", "Cargo.lock", "src/lib.rs"}
                if named != want:
                    print(f"FAIL  [cli source set: the walk names {named}, "
                          f"want {want} — benches/**.rs is its own cargo "
                          f"workspace and docs are not source]")
                    errors += 1
                else:
                    print("ok    [cli source set: the walk takes this "
                          "workspace's .rs + manifests and skips the "
                          "excluded trees and non-source files]")

                base = bench.cerulion_source_floor_ns()
                # THE arm for the dropped committer-time term. A commit
                # touches no source byte and no mtime, so it must not move
                # the floor. While the floor was max(HEAD %ct, mtimes) this
                # very branch measured false-stale windows of 63-79 minutes
                # — every already-built binary declared stale for the
                # length of an editing session — and no arm saw it.
                run_git(["commit", "-q", "--allow-empty", "-m", "no-op"])
                bench._CLI_SOURCE_FLOOR_CACHE = None
                after_commit = bench.cerulion_source_floor_ns()
                if after_commit != base:
                    print(f"FAIL  [cli source floor: an empty commit moved "
                          f"the floor {base} -> {after_commit} — committing "
                          f"changes no source, so every binary already "
                          f"built would read stale for the rest of the "
                          f"session]")
                    errors += 1
                # WITHIN the future grace, deliberately. The fixture's own
                # HEAD commit is dated ~now, so the floor already sits
                # there and only a LATER mtime can move it — but a mtime an
                # hour ahead of the wall clock is exactly what
                # usable_source_floor refuses as untrustworthy.
                # A 30 s step is above HEAD
                # and inside the grace, which is the only window that
                # tests movement rather than the clamp.
                # Inside the future-mtime grace, DERIVED from it: a
                # step above the grace is refused as untrustworthy by
                # `usable_source_floor`, and this arm would then be
                # measuring the clamp instead of the closure.
                later = base + bench.CLI_SOURCE_FLOOR_FUTURE_GRACE_NS // 2
                # Editing an EXCLUDED tree must not move the floor: that
                # is what stops a bench edit refusing a fresh CLI.
                os.utime(repo / "benches" / "x" / "b.rs",
                         ns=(later, later))
                bench._CLI_SOURCE_FLOOR_CACHE = None
                if bench.cerulion_source_floor_ns() != base:
                    print("FAIL  [cli source floor: touching an EXCLUDED "
                          "tree moved the floor — a bench edit would then "
                          "refuse a CLI built minutes earlier]")
                    errors += 1
                # Editing a real source file must move it.
                os.utime(repo / "src" / "lib.rs", ns=(later, later))
                bench._CLI_SOURCE_FLOOR_CACHE = None
                moved = bench.cerulion_source_floor_ns()
                if moved != later:
                    print(f"FAIL  [cli source floor: touching src/lib.rs "
                          f"must move the floor to {later}, got {moved!r}]")
                    errors += 1
                # An UNTRACKED, un-stat-able entry must NOT turn the
                # whole gate off. The totality rule requires every listed
                # path to stat(), and `--others` is exactly where a
                # dangling `.rs`-named symlink lives (an editor lock file,
                # a path removed with `rm` rather than `git rm`) — one of
                # those anywhere made every candidate UNVERIFIABLE and
                # degraded this whole check to a non-failing skip, for a
                # reason having nothing to do with staleness.
                (repo / "src" / "dangling.rs").symlink_to(repo / "gone.rs")
                bench._CLI_SOURCE_FLOOR_CACHE = None
                if bench.cerulion_source_floor_ns() is None:
                    print("FAIL  [cli source floor: one dangling UNTRACKED "
                          "source must be dropped, not make the entire "
                          "tree undatable — it was never a build input]")
                    errors += 1
                (repo / "src" / "dangling.rs").unlink()
                bench._CLI_SOURCE_FLOOR_CACHE = None

                # The future clamp, through the REAL floor function. The
                # pure table drives usable_source_floor directly and the
                # rest of this fixture deliberately stays inside the grace,
                # so deleting the call in cerulion_source_floor_ns was
                # invisible: a tree whose sources are dated tomorrow would
                # then mark every binary anyone could build as stale.
                far = time.time_ns() + 2 * bench.CLI_SOURCE_FLOOR_FUTURE_GRACE_NS
                os.utime(repo / "src" / "lib.rs", ns=(far, far))
                bench._CLI_SOURCE_FLOOR_CACHE = None
                if bench.cerulion_source_floor_ns() is not None:
                    print("FAIL  [cli source floor: a source dated well "
                          "past the wall clock makes the tree undatable, "
                          "not everything-is-stale — the clamp is not "
                          "wired into the real floor]")
                    errors += 1
                os.utime(repo / "src" / "lib.rs", ns=(later, later))
                bench._CLI_SOURCE_FLOOR_CACHE = None

                # A tracked-but-UNREADABLE file makes the walk PARTIAL,
                # and a partial walk cannot see the newest source — so the
                # answer is "undatable", not a floor built from what could
                # be read. Measured on a synthetic repo before this rule:
                # blinding the single newest source moved the floor back
                # four days and turned a genuinely stale binary into a
                # confident CLI_FRESH, which emits nothing at all because a
                # fresh verdict carries no reason. Answering UNVERIFIABLE
                # on a sparse checkout is the correct claim about a sparse
                # checkout.
                (repo / "src" / "lib.rs").unlink()
                bench._CLI_SOURCE_FLOOR_CACHE = None
                after_delete = bench.cerulion_source_floor_ns()
                if after_delete is not None:
                    print(f"FAIL  [cli source floor: a tracked file that "
                          f"cannot be stat'd makes the walk partial, so the "
                          f"floor must be None (undatable) rather than a "
                          f"confident maximum over what was readable, got "
                          f"{after_delete!r}]")
                    errors += 1
                # Outside any repo there is no evidence at all.
                bench.REPO_ROOT = Path(td) / "not-a-repo"
                bench.REPO_ROOT.mkdir()
                bench._CLI_SOURCE_FLOOR_CACHE = None
                if bench.cerulion_source_floor_ns() is not None:
                    print("FAIL  [cli source floor: outside a git repo the "
                          "floor must be None (undatable)]")
                    errors += 1
        except RuntimeError as e:
            # RuntimeError is raised ONLY by run_git, i.e. only while the
            # fixture is being built. An OSError from the assertion region
            # is deliberately NOT caught: it is a failure, and reporting it
            # as "fixture could not be built" would hide four arms that
            # never ran behind a `skip` and a closing `ok`.
            print(f"skip  [cli source set: fixture could not be built: {e}]")
        finally:
            bench.REPO_ROOT = saved_root
            bench._CLI_SOURCE_FLOOR_CACHE = saved_cache
            bench._CLI_CLOSURE_NOTE_SHOWN = saved_note_src
            if saved_ctd_src is not None:
                os.environ["CARGO_TARGET_DIR"] = saved_ctd_src

    # ---- the REAL resolver, over a stale fixture and a fresh control -----
    saved_root = bench.REPO_ROOT
    saved_floor_fn = bench.cerulion_source_floor_ns
    saved_which = bench.shutil.which
    # An ambient CARGO_TARGET_DIR would send cargo_target_dir() somewhere
    # that is not the fixture tree, classify every fixture binary FOREIGN
    # and invert every positive arm below — the whole gate red on a host
    # whose only sin is sharing a build cache. Popped for the duration and
    # restored; the arm that WANTS it sets it itself.
    saved_ctd_outer = os.environ.pop("CARGO_TARGET_DIR", None)
    try:
        with tempfile.TemporaryDirectory(prefix="cli_") as td:
            tree = Path(td) / "tree"
            outside = Path(td) / "elsewhere" / "cerulion"
            release = tree / "target" / "release" / "cerulion"
            debug = tree / "target" / "debug" / "cerulion"
            bench.REPO_ROOT = tree
            bench.cerulion_source_floor_ns = lambda: floor

            def only_outside(name, *a, **k):
                return str(outside) if name == "cerulion" else saved_which(
                    name, *a, **k)

            # (1) nothing anywhere.
            bench.shutil.which = lambda name, *a, **k: (
                None if name == "cerulion" else saved_which(name, *a, **k))
            got, note = bench.resolve_cerulion_cli()
            if got is not None or REBUILD_CMD not in note:
                print(f"FAIL  [cli resolver: an empty host must resolve to "
                      f"None with a build instruction, got {got!r} / {note!r}]")
                errors += 1

            # (2) FRESH control — the arm every refusal below is measured
            # against. Without it a resolver that refuses everything passes.
            _touch_ns(release, floor + HOUR)
            got, note = bench.resolve_cerulion_cli()
            if got != release or note != "":
                print(f"FAIL  [cli resolver: a fresh in-tree binary must be "
                      f"used with no complaint, got {got!r} / {note!r}]")
                errors += 1

            # (3) STALE fixture — the finding itself.
            _touch_ns(release, floor - HOUR)
            got, note = bench.resolve_cerulion_cli()
            if got is not None or bench.CLI_STALE not in note:
                print(f"FAIL  [cli resolver: a stale in-tree binary must be "
                      f"declined as stale, got {got!r} / {note!r}]")
                errors += 1
            elif REBUILD_CMD not in note:
                print(f"FAIL  [cli resolver: the refusal must name the "
                      f"rebuild, got {note!r}]")
                errors += 1

            # (4) a FRESH sibling rescues a stale one — a stale candidate
            # must not veto the tree's other profile.
            _touch_ns(debug, floor + HOUR)
            got, note = bench.resolve_cerulion_cli()
            if got != debug or note != "":
                print(f"FAIL  [cli resolver: a fresh debug build must be "
                      f"used when release is stale, got {got!r} / {note!r}]")
                errors += 1

            # (5) FRESHEST wins, not list order: release is listed first,
            # debug is newer.
            _touch_ns(release, floor + HOUR)
            _touch_ns(debug, floor + 2 * HOUR)
            got, _note = bench.resolve_cerulion_cli()
            if got != debug:
                print(f"FAIL  [cli resolver: with both profiles fresh the "
                      f"NEWEST must win, got {got!r} (release is listed "
                      f"first, debug is newer)]")
                errors += 1

            # (6) FOREIGN: only a PATH binary, outside the tree, and newer
            # than everything — recency must not buy provenance.
            release.unlink()
            debug.unlink()
            _touch_ns(outside, floor + 9 * HOUR)
            bench.shutil.which = only_outside
            got, note = bench.resolve_cerulion_cli()
            if got is not None or bench.CLI_FOREIGN not in note:
                print(f"FAIL  [cli resolver: a PATH binary from another "
                      f"checkout must be declined as foreign however new it "
                      f"is, got {got!r} / {note!r}]")
                errors += 1

            # (6b) SHAPES that look like a candidate and are not one.
            # `exists()` would have handed the first two to subprocess as
            # binaries, and the third would have vanished with the
            # operator told to build a binary they can see.
            bench.shutil.which = lambda name, *a, **k: (
                None if name == "cerulion" else saved_which(name, *a, **k))
            outside.unlink()
            for label, make, needle in (
                ("a directory named cerulion",
                 lambda: release.mkdir(parents=True), "directory"),
                ("a non-executable file",
                 lambda: (_touch_ns(release, floor + HOUR),
                          os.chmod(release, 0o600)), "not executable"),
                ("a dangling symlink",
                 lambda: (release.parent.mkdir(parents=True, exist_ok=True),
                          release.symlink_to(tree / "gone")),
                 "dangling symlink"),
            ):
                if release.is_symlink() or release.exists():
                    (release.rmdir() if release.is_dir()
                     else release.unlink())
                make()
                got, note = bench.resolve_cerulion_cli()
                if got is not None:
                    print(f"FAIL  [cli resolver: {label} must not be used as "
                          f"a binary, got {got!r}]")
                    errors += 1
                elif needle not in note:
                    print(f"FAIL  [cli resolver: {label} must be NAMED in "
                          f"the refusal (an operator told to 'build one' "
                          f"is sent to the wrong problem), got {note!r}]")
                    errors += 1
            if release.is_symlink() or release.exists():
                (release.rmdir() if release.is_dir() else release.unlink())

            # (6c) SYMLINK semantics, both directions — the resolver
            # documents them and nothing drove them, so `.resolve()` could
            # be dropped silently.
            _touch_ns(debug, floor + HOUR)
            release.parent.mkdir(parents=True, exist_ok=True)
            release.symlink_to(debug)          # in-tree -> in-tree
            got, _n = bench.resolve_cerulion_cli()
            if got is None:
                print("FAIL  [cli resolver: an in-tree symlink to an in-tree "
                      "binary must be usable]")
                errors += 1
            release.unlink()
            outside.parent.mkdir(parents=True, exist_ok=True)
            _touch_ns(outside, floor + HOUR)
            release.symlink_to(outside)        # in-tree -> OUTSIDE
            got, note = bench.resolve_cerulion_cli()
            if got == release or (got is None and
                                  bench.CLI_FOREIGN not in note):
                print(f"FAIL  [cli resolver: a target/ path that is a "
                      f"symlink OUT of the tree must be declined foreign — "
                      f"recency and location must both be resolved, got "
                      f"{got!r} / {note!r}]")
                errors += 1
            release.unlink()

            # (6c-bis) A symlinked DEFAULT binary must not lend its
            # exemption to every other alias of the same target. If
            # `target/release/cerulion` is itself a link to a foreign
            # build, `candidate.resolve() == default_cli.resolve()` asks
            # only "do these end at the same file" — which any other link
            # to that foreign binary also satisfies, exempting it from
            # every provenance check.
            alias_foreign = Path(td) / "alias-to-foreign"
            if alias_foreign.exists() or alias_foreign.is_symlink():
                alias_foreign.unlink()
            _touch_ns(outside, floor + HOUR)
            if release.exists() or release.is_symlink():
                release.unlink()
            release.symlink_to(outside)
            alias_foreign.symlink_to(outside)
            saved_env_alias = os.environ.get(bench.CERULION_BIN_ENV)
            os.environ[bench.CERULION_BIN_ENV] = str(alias_foreign)
            try:
                bench.refuse_foreign_cerulion_override("workspace")
                print("FAIL  [CERULION override: with the default path "
                      "itself a symlink to a foreign binary, a SEPARATE "
                      "alias of that same target must NOT inherit the "
                      "default-path exemption — it was accepted]")
                errors += 1
            except SystemExit:
                pass
            finally:
                if saved_env_alias is None:
                    os.environ.pop(bench.CERULION_BIN_ENV, None)
                else:
                    os.environ[bench.CERULION_BIN_ENV] = saved_env_alias
            alias_foreign.unlink()
            release.unlink()

            # (6d) EQUAL mtimes: the tie-break must be deterministic, and
            # it is candidate order (release is listed first).
            _touch_ns(release, floor + HOUR)
            _touch_ns(debug, floor + HOUR)
            first = bench.resolve_cerulion_cli()[0]
            second = bench.resolve_cerulion_cli()[0]
            if first != release or second != release:
                print(f"FAIL  [cli resolver: with both profiles at the SAME "
                      f"mtime the tie must break deterministically to the "
                      f"first candidate (release), got {first!r} then "
                      f"{second!r}]")
                errors += 1

            # (6e) An UNDATABLE binary is DECLINED by the resolver. The
            # `CERULION=` override refusal takes the opposite view of the
            # same condition (arm below), which is the whole reason the two
            # policies are written separately rather than shared.
            bench.cerulion_source_floor_ns = lambda: None
            _touch_ns(release, floor + HOUR)
            got, note = bench.resolve_cerulion_cli()
            if got is not None:
                print(f"FAIL  [cli resolver: an undatable binary must be "
                      f"DECLINED — accepting one restores exactly the "
                      f"behaviour this check exists to end — got {got!r}]")
                errors += 1
            elif bench.CLI_UNVERIFIABLE not in note or \
                    bench.CLI_REBUILD_CMD not in note:
                print(f"FAIL  [cli resolver: the refusal must name the "
                      f"verdict and the rebuild, got {note!r}]")
                errors += 1
            bench.cerulion_source_floor_ns = lambda: floor

            # (6f) CARGO_TARGET_DIR. An ordinary developer setting, and
            # hard-coding REPO_ROOT/target against it is worse than merely
            # missing the binary: a freshly built in-tree CLI is then
            # invisible (or, on PATH, `foreign`) and the refusal tells the
            # operator to run `cargo build`, which puts the output right
            # back where nothing is looking.
            # CARGO_TARGET_DIR is an ordinary developer setting AND a
            # SHARED one — `export CARGO_TARGET_DIR=~/.cargo-target` in a
            # shell rc is the standard way to share a build cache across
            # checkouts, and then `$CARGO_TARGET_DIR/release/cerulion` is
            # whatever tree built LAST. So the path cannot answer the
            # provenance question there; the binary's cargo depfile (which
            # lists absolute source paths) must. Getting this wrong in
            # either direction is a real defect: hard-coding REPO_ROOT
            # makes an in-tree build invisible, and trusting the path makes
            # a FOREIGN build read `fresh` with an empty note.
            saved_ctd = os.environ.get("CARGO_TARGET_DIR")
            try:
                elsewhere_target = Path(td) / "ctd"
                moved_cli = elsewhere_target / "release" / "cerulion"
                _touch_ns(moved_cli, floor + HOUR)
                os.environ["CARGO_TARGET_DIR"] = str(elsewhere_target)

                # (i) no depfile => nobody can tell => declined, not used.
                got, note = bench.resolve_cerulion_cli()
                if got is not None or bench.CLI_UNVERIFIABLE not in note:
                    print(f"FAIL  [cli resolver: under a shared "
                          f"CARGO_TARGET_DIR a binary with no depfile "
                          f"cannot be attributed to any checkout and must "
                          f"be declined, got {got!r} / {note!r}]")
                    errors += 1

                # (ii) a depfile naming THIS tree => this tree's build.
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {tree}/crates/cerulion_cli/src/main.rs\n",
                    encoding="utf-8")
                got, note = bench.resolve_cerulion_cli()
                if got != moved_cli or note != "":
                    print(f"FAIL  [cli resolver: under CARGO_TARGET_DIR a "
                          f"binary whose depfile names THIS checkout is "
                          f"this checkout's build and must be used, got "
                          f"{got!r} / {note!r}]")
                    errors += 1
                elif bench.cli_provenance(moved_cli) != bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: its provenance must read "
                          "in_tree]")
                    errors += 1

                # (ii-b) THE bypass: a custom target dir NESTED inside
                # REPO_ROOT/target. `CARGO_TARGET_DIR=$PWD/target/shared`
                # is an ordinary thing to write, and the "the checkout's
                # own target/ is proof" shortcut readmitted it — measured
                # returning in_tree for a binary with NO depfile at all,
                # which is the provenance guarantee bypassed by the very
                # path check meant to establish it.
                nested_target = tree / "target" / "shared"
                nested_cli = nested_target / "release" / "cerulion"
                _touch_ns(nested_cli, floor + HOUR)
                os.environ["CARGO_TARGET_DIR"] = str(nested_target)
                if bench.cli_provenance(nested_cli) == bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: a CARGO_TARGET_DIR nested "
                          "inside REPO_ROOT/target must NOT be trusted on "
                          "its path — a shared target dir is whatever tree "
                          "built last, wherever it sits]")
                    errors += 1
                nested_cli.with_suffix(".d").write_text(
                    f"{nested_cli}: {Path(td)}/other/src/main.rs\n",
                    encoding="utf-8")
                if bench.cli_provenance(nested_cli) != bench.PROV_FOREIGN:
                    print("FAIL  [cli resolver: a nested-CARGO_TARGET_DIR "
                          "binary whose depfile names another checkout is "
                          "FOREIGN]")
                    errors += 1
                nested_cli.with_suffix(".d").write_text(
                    f"{nested_cli}: {tree}/crates/cerulion_cli/src/main.rs\n",
                    encoding="utf-8")
                if bench.cli_provenance(nested_cli) != bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: a nested-CARGO_TARGET_DIR "
                          "binary whose depfile names THIS checkout is this "
                          "checkout's build]")
                    errors += 1
                os.environ["CARGO_TARGET_DIR"] = str(elsewhere_target)

                # ...and with CARGO_TARGET_DIR UNSET, the default target IS
                # proof by construction — the shortcut must not be lost
                # altogether, or every ordinary desk needs a depfile.
                os.environ.pop("CARGO_TARGET_DIR", None)
                default_cli = tree / "target" / "release" / "cerulion"
                _touch_ns(default_cli, floor + HOUR)
                if bench.cli_provenance(default_cli) != bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: with CARGO_TARGET_DIR unset, "
                          "a binary under this checkout's own target/ is its "
                          "own build — requiring a depfile there would make "
                          "every ordinary desk undatable]")
                    errors += 1
                os.environ["CARGO_TARGET_DIR"] = str(elsewhere_target)

                # (ii-c) A depfile listing OUT-OF-TREE sources FIRST.
                # cargo writes its dep set sorted and lists everything that
                # contributed — generated `$OUT_DIR/*.rs` included, which
                # under a shared target dir sits outside the checkout by
                # construction. Deciding on the first `.rs` token answered
                # FOREIGN for a binary this checkout really built, exactly
                # where the depfile is the only evidence there is.
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {elsewhere_target}/build/x/out/gen.rs "
                    f"{Path(td)}/aaa-other/src/lib.rs "
                    f"{tree}/crates/cerulion_cli/src/main.rs\n",
                    encoding="utf-8")
                if bench.cli_provenance(moved_cli) != bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: a depfile that names THIS "
                          "checkout anywhere in its source list is this "
                          "checkout's build — the first token is an "
                          "accident of sort order, not evidence]")
                    errors += 1
                # ...and a depfile with NO in-tree source anywhere is
                # foreign, which is what keeps the scan from degrading into
                # "any depfile will do".
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {Path(td)}/aaa-other/src/lib.rs "
                    f"{Path(td)}/zzz-other/src/main.rs\n",
                    encoding="utf-8")
                if bench.cli_provenance(moved_cli) != bench.PROV_FOREIGN:
                    print("FAIL  [cli resolver: a depfile naming only other "
                          "trees is FOREIGN]")
                    errors += 1
                # A depfile with no `.rs` token at all is not an answer.
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}:\n", encoding="utf-8")
                if bench.cli_provenance(moved_cli) != bench.PROV_UNKNOWN:
                    print("FAIL  [cli resolver: a depfile listing no sources "
                          "says nothing, which is UNKNOWN not FOREIGN]")
                    errors += 1

                # (ii-d) A RELATIVE CARGO_TARGET_DIR is resolved by CARGO,
                # from the repo root that run_workspace.sh invokes it in —
                # not from wherever this process happens to be running.
                saved_cwd_env = os.environ["CARGO_TARGET_DIR"]
                os.environ["CARGO_TARGET_DIR"] = "target-cache"
                if bench.cargo_target_dir() != tree / "target-cache":
                    print(f"FAIL  [cli resolver: a relative "
                          f"CARGO_TARGET_DIR must anchor at the repo root "
                          f"(cargo's cwd), got "
                          f"{bench.cargo_target_dir()!r}]")
                    errors += 1
                os.environ["CARGO_TARGET_DIR"] = saved_cwd_env

                # (ii-e) The depfile's own SPELLING and LOCATION.
                #
                # Three separate ways a binary this checkout really built
                # was declined as foreign, all of them reachable only once
                # CARGO_TARGET_DIR is set (the path check short-circuits
                # otherwise), and all of them hard: the override refusal
                # calls SystemExit on FOREIGN.
                #  - an UNRESOLVED prefix. REPO_ROOT is built from
                #    `Path(__file__).resolve()`, so a "raw or resolved"
                #    comparison against it was a one-element set, and
                #    cargo records the path IT saw — which on a host
                #    reached through a symlinked prefix (`/var` ->
                #    `/private/var` here) is the unresolved one.
                #  - an ESCAPED SPACE. Cargo escapes a literal space as
                #    `\ `; a bare whitespace split tore such a path in two.
                #  - a SYMLINKED candidate, whose depfile sits beside the
                #    real file, not beside the link.
                # REPO_ROOT is the RAW spelling here, so a token written
                # in the CANONICAL one exercises the realpath comparison
                # and nothing else — a raw-prefix test alone cannot match
                # it. (The reverse pairing is what a real host produces:
                # REPO_ROOT canonical, cargo recording what it saw.)
                if str(tree.resolve()) != str(tree):
                    moved_cli.with_suffix(".d").write_text(
                        f"{moved_cli}: {tree.resolve()}/src/main.rs\n",
                        encoding="utf-8")
                    if bench.cli_provenance(moved_cli) != bench.PROV_IN_TREE:
                        print("FAIL  [cli resolver: a depfile whose source "
                              "paths are spelled with the OTHER form of "
                              "this checkout's prefix must still read "
                              "in_tree — cargo records the path it saw, "
                              "and REPO_ROOT is only ever one spelling]")
                        errors += 1
                else:
                    print("skip  [cli resolver: this fixture's prefix is "
                          "not a symlink, so the two spellings coincide]")
                spaced = Path(td) / "with space"
                spaced.mkdir(exist_ok=True)
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {tree}/a\\ b/src/main.rs\n",
                    encoding="utf-8")
                if bench.cli_provenance(moved_cli) != bench.PROV_IN_TREE:
                    print("FAIL  [cli resolver: a depfile path with a "
                          "cargo-escaped space belongs to whichever tree "
                          "its unescaped form names — splitting on the "
                          "escape declines this checkout's own build]")
                    errors += 1
                escaped = r"/a/my\ tree/x.rs /b/y.rs"
                got_tokens = bench._depfile_tokens(escaped)
                if got_tokens != ["/a/my tree/x.rs", "/b/y.rs"]:
                    print("FAIL  [cli resolver: the depfile tokenizer must "
                          "unescape a cargo-escaped space, got "
                          + repr(got_tokens) + "]")
                    errors += 1
                # A SYMLINKED candidate: the depfile is beside the real
                # file. Read beside the link, it is not found at all, and
                # PROV_UNKNOWN downgrades the override refusal from a
                # SystemExit to a warning — a foreign symlink slipping
                # through is the worse direction of this one.
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {Path(td)}/zzz-other/src/main.rs\n",
                    encoding="utf-8")
                linked = elsewhere_target / "release" / "cerulion_link"
                if linked.exists() or linked.is_symlink():
                    linked.unlink()
                linked.symlink_to(moved_cli)
                if bench.cli_provenance(linked) != bench.PROV_FOREIGN:
                    print(f"FAIL  [cli resolver: a SYMLINKED candidate's "
                          f"depfile sits beside the real file — read beside "
                          f"the link it is not found, and UNKNOWN lets a "
                          f"foreign binary through the override refusal "
                          f"with a warning instead of a refusal, got "
                          f"{bench.cli_provenance(linked)}]")
                    errors += 1
                linked.unlink()

                # (iii) a depfile naming ANOTHER tree => foreign, however
                # new.
                moved_cli.with_suffix(".d").write_text(
                    f"{moved_cli}: {Path(td)}/other-checkout/src/main.rs\n",
                    encoding="utf-8")
                got, note = bench.resolve_cerulion_cli()
                if got is not None or bench.CLI_FOREIGN not in note:
                    print(f"FAIL  [cli resolver: under CARGO_TARGET_DIR a "
                          f"binary whose depfile names ANOTHER checkout is "
                          f"foreign however fresh it looks, got {got!r} / "
                          f"{note!r}]")
                    errors += 1
                moved_cli.with_suffix(".d").unlink()
            finally:
                if saved_ctd is None:
                    os.environ.pop("CARGO_TARGET_DIR", None)
                else:
                    os.environ["CARGO_TARGET_DIR"] = saved_ctd

            # (7) An unusable SIBLING must be named even on a good day:
            # a fresh debug build beside a broken release path resolves
            # fine, and saying nothing about the broken one leaves an
            # operator to discover it the next time it is the only
            # candidate.
            _touch_ns(debug, floor + HOUR)
            if release.exists():
                release.unlink()
            release.mkdir(parents=True)
            got, note = bench.resolve_cerulion_cli()
            if got != debug:
                print(f"FAIL  [cli resolver: a fresh debug build must be "
                      f"used when release is unusable, got {got!r}]")
                errors += 1
            elif "directory" not in note:
                print(f"FAIL  [cli resolver: a successful resolve must "
                      f"still name an unusable sibling, got {note!r}]")
                errors += 1
            elif str(release) not in note:
                # `.capitalize()` uppercases the first character and
                # LOWERCASES every other one, and this note carries
                # filesystem paths — an operator was shown a path that does
                # not exist on a case-sensitive filesystem.
                print(f"FAIL  [cli resolver: the note must name the "
                      f"unusable path VERBATIM — case-mangling it points "
                      f"an operator at a path that does not exist, got "
                      f"{note!r}]")
                errors += 1
            release.rmdir()

            # A REPO_ROOT that cannot be resolved at all: a symlink loop
            # inside it. `Path.resolve()` raises there on CPython <= 3.12
            # (as RuntimeError, not OSError — an OSError-only handler let
            # it escape as an unhandled crash from a guard whose whole job
            # is to refuse cleanly) and simply returns the path on 3.13+.
            # So the OUTCOME is asserted, not the mechanism: the exact
            # default path is still exempted, and nothing raises anything
            # other than the SystemExit this function is allowed to raise.
            # A REPO_ROOT that cannot be resolved at all: a symlink loop
            # inside it. `Path.resolve()` raises there on CPython <= 3.12
            # (as RuntimeError, NOT OSError — an OSError-only handler let
            # it escape as an unhandled crash from a guard whose whole job
            # is to refuse cleanly) and simply returns the path on 3.13+.
            # So the OUTCOME is asserted, not the mechanism: never a crash,
            # and the exact default still exempted.
            loop_root = Path(td) / "looproot"
            loop_root.mkdir()
            (loop_root / "loop").symlink_to(loop_root / "loop")
            saved_root_loop = bench.REPO_ROOT
            saved_env_loop = os.environ.get(bench.CERULION_BIN_ENV)
            bench.REPO_ROOT = loop_root / "loop"
            try:
                for what, rel, want_exempt in (
                    # The exact default: only the RAW comparison can answer
                    # it here, because resolution cannot run at all.
                    ("the exact default", ("target", "release", "cerulion"),
                     True),
                    # A NON-default path is the only way to REACH the
                    # resolution (the exact default returns above it), so
                    # this is the row where the RuntimeError is reachable.
                    ("a non-default path", ("target", "debug", "cerulion"),
                     False),
                ):
                    os.environ[bench.CERULION_BIN_ENV] = str(
                        bench.REPO_ROOT.joinpath(*rel))
                    outcome = "accepted"
                    try:
                        bench.refuse_foreign_cerulion_override("workspace")
                    except SystemExit:
                        outcome = "refused"
                    except Exception as e:    # noqa: BLE001 - that IS the bug
                        outcome = f"CRASH {type(e).__name__}: {e}"
                    if outcome.startswith("CRASH"):
                        print(f"FAIL  [CERULION override: an unresolvable "
                              f"REPO_ROOT must not escape the guard "
                              f"({what}) — got {outcome}. `Path.resolve()` "
                              f"raises RuntimeError, not OSError, on a "
                              f"symlink loop before CPython 3.13]")
                        errors += 1
                    elif want_exempt and outcome != "accepted":
                        print(f"FAIL  [CERULION override: with an "
                              f"unresolvable REPO_ROOT the EXACT default "
                              f"path must still be exempted — the raw "
                              f"comparison is what covers the case "
                              f"resolution cannot — got {outcome}]")
                        errors += 1
                    elif not want_exempt and outcome != "refused":
                        print(f"FAIL  [CERULION override: {what} under an "
                              f"unresolvable root must be REFUSED, got "
                              f"{outcome}]")
                        errors += 1
            finally:
                bench.REPO_ROOT = saved_root_loop
                if saved_env_loop is None:
                    os.environ.pop(bench.CERULION_BIN_ENV, None)
                else:
                    os.environ[bench.CERULION_BIN_ENV] = saved_env_loop

            # The default-path exemption only holds while the runner
            # BUILDS the path it runs. Under a custom CARGO_TARGET_DIR it
            # does not: `cargo build -p cerulion_cli --release` writes to
            # the custom dir while run_workspace.sh's
            # `${CERULION:-$REPO_ROOT/target/release/cerulion}` default
            # still names the repo one, so the binary it executes is
            # whatever was left there, rebuilt by nothing. Exempting it
            # then measures a campaign through stale code.
            saved_ctd_ex = os.environ.get("CARGO_TARGET_DIR")
            saved_env_ex = os.environ.get(bench.CERULION_BIN_ENV)
            try:
                _touch_ns(release, floor - HOUR)          # STALE
                release.with_suffix(".d").write_text(
                    f"{release}: {tree}/crates/cerulion_cli/src/main.rs\n",
                    encoding="utf-8")
                os.environ[bench.CERULION_BIN_ENV] = str(release)
                # Every spelling that still has cargo write the file the
                # runner runs must keep the exemption. Asking whether the
                # VARIABLE is set answers the shared-cache case and gets
                # the ordinary ones wrong: `CARGO_TARGET_DIR=target`, and
                # an absolute spelling of this checkout's own target dir,
                # both build exactly `<repo>/target/release/cerulion`.
                for label, ctd, want_refused in (
                        ("unset — the runner builds it", None, False),
                        ("a RELATIVE spelling of this checkout's own "
                         "target dir", "target", False),
                        ("an ABSOLUTE spelling of the same dir",
                         str(tree / "target"), False),
                        ("the runner builds elsewhere",
                         str(Path(td) / "shared-cache"), True)):
                    os.environ.pop("CARGO_TARGET_DIR", None)
                    if ctd is not None:
                        os.environ["CARGO_TARGET_DIR"] = ctd
                    try:
                        bench.refuse_foreign_cerulion_override("workspace")
                        refused = False
                    except SystemExit:
                        refused = True
                    if refused != want_refused:
                        print(f"FAIL  [CERULION override: a STALE override "
                              f"naming the runner's default path, when "
                              f"{label} — expected "
                              f"{'refused' if want_refused else 'accepted'}, "
                              f"got {'refused' if refused else 'accepted'}. "
                              f"The exemption's premise is that the runner "
                              f"rebuilds exactly that file, which a custom "
                              f"CARGO_TARGET_DIR makes false]")
                        errors += 1
                release.with_suffix(".d").unlink()

                # ABSENT, not merely stale: the fresh-checkout shape a CI
                # wrapper hits. The runner BUILDS this path, so refusing
                # it is a refusal of nothing — and it is what the
                # variable-is-set test produced.
                if release.exists() or release.is_symlink():
                    release.unlink()
                for label, ctd, want_refused in (
                        ("unset", None, False),
                        ("a relative spelling of this checkout's target",
                         "target", False),
                        ("a different target dir",
                         str(Path(td) / "shared-cache"), True)):
                    os.environ.pop("CARGO_TARGET_DIR", None)
                    if ctd is not None:
                        os.environ["CARGO_TARGET_DIR"] = ctd
                    try:
                        bench.refuse_foreign_cerulion_override("workspace")
                        refused = False
                    except SystemExit:
                        refused = True
                    if refused != want_refused:
                        print(f"FAIL  [CERULION override: an ABSENT "
                              f"override naming the runner's default path, "
                              f"with CARGO_TARGET_DIR {label} — expected "
                              f"{'refused' if want_refused else 'accepted'},"
                              f" got "
                              f"{'refused' if refused else 'accepted'}. The "
                              f"runner builds that path before running it, "
                              f"so refusing it refuses nothing]")
                        errors += 1
                _touch_ns(release, floor + HOUR)
            finally:
                os.environ.pop("CARGO_TARGET_DIR", None)
                if saved_ctd_ex is not None:
                    os.environ["CARGO_TARGET_DIR"] = saved_ctd_ex
                if saved_env_ex is None:
                    os.environ.pop(bench.CERULION_BIN_ENV, None)
                else:
                    os.environ[bench.CERULION_BIN_ENV] = saved_env_ex

            # `rm -rf <entry>` does not follow symlinks, so the deletion
            # check must be LEXICAL. A resolved check gets both directions
            # wrong, and both were MEASURED against the real command:
            # a `cerulion` symlink inside a real `target/debug` has its
            # ENTRY deleted (its target survives), while a `target/debug`
            # that is itself a symlink loses only the link, so everything
            # in its target survives.
            doom_root = Path(td) / "doom"
            real_debug = doom_root / "real" / "debug"
            real_debug.mkdir(parents=True)
            far = doom_root / "far"
            far.mkdir()
            (far / "cerulion").write_text("x", encoding="utf-8")
            (real_debug / "cerulion").symlink_to(far / "cerulion")
            (doom_root / "filedebug").write_text("x", encoding="utf-8")
            (doom_root / "ext").symlink_to(doom_root / "filedebug")
            linked_debug = doom_root / "linked" / "debug"
            linked_debug.parent.mkdir(parents=True)
            linked_debug.symlink_to(far)
            for what, cand, doomed, want in (
                ("a symlink ENTRY inside a real target/debug",
                 real_debug / "cerulion", real_debug, True),
                ("a binary whose directory target/debug merely LINKS to",
                 far / "cerulion", linked_debug, False),
                ("an unrelated binary", doom_root / "other", real_debug,
                 False),
                # `rm -rf <entry>` removes a regular FILE as readily as it
                # empties a directory, so an external symlink to a
                # file-valued target/debug dangles before the run.
                ("an external symlink to a FILE-valued target/debug",
                 doom_root / "ext", doom_root / "filedebug", True),
            ):
                got = bench._deleted_by_the_runner(cand, doomed)
                if got != want:
                    print(f"FAIL  [CERULION override: `rm -rf` of the "
                          f"runner's target/debug — {what} should be "
                          f"{'removed' if want else 'untouched'}, the check "
                          f"says {'removed' if got else 'untouched'}]")
                    errors += 1

            # ---- the CERULION= override refusal ---------------------------
            # A symlinked alias of the whole checkout, so one row can name
            # the default CLI through a prefix that is not lexically equal
            # to REPO_ROOT.
            alias_tree = Path(td) / "alias-checkout"
            if not alias_tree.exists():
                alias_tree.symlink_to(tree)
            alias_release = alias_tree / "target" / "release" / "cerulion"
            noexec = tree / "target" / "release" / "cerulion_noexec"
            _touch_ns(noexec, floor + HOUR)
            os.chmod(noexec, 0o600)
            # The REAL workspace dir, because that is what the guard reads;
            # the file is created and removed by this arm alone.
            doomed_cli = bench.WORKSPACE_DIR / "target" / "debug" / "cerulion"
            _touch_ns(doomed_cli, floor + HOUR)
            # Its own target, deliberately: the row must exercise the
            # DELETION check, and pointing at a fixture file that other
            # arms unlink or turn into a directory makes the link dangle,
            # so the guard answers "not a file" and the row stops testing
            # what it names.
            doom_target = tree / "doom_link_target"
            _touch_ns(doom_target, floor + HOUR)
            doomed_link = doomed_cli.parent / "cerulion_link"
            if doomed_link.exists() or doomed_link.is_symlink():
                doomed_link.unlink()
            doomed_link.symlink_to(doom_target)
            saved_env = os.environ.get(bench.CERULION_BIN_ENV)
            try:
                # `release_ns` per row, because the exemption row is the
                # only one that needs the DEFAULT path to be STALE — and
                # while every row got a fresh release, that row passed with
                # the exemption deleted. A vacuous
                # row is worse than a missing one: it reads as coverage.
                for label, value, want_raise, needle, release_ns in (
                    ("unset", None, False, None, floor + HOUR),
                    # EMPTY is unset, because run_workspace.sh reads
                    # `${CERULION:-<default>}` and `:-` substitutes for an
                    # empty value too. Refusing it would be a FALSE refusal
                    # on a run the runner performs correctly.
                    ("empty", "", False, None, floor + HOUR),
                    # NOT unset: `${CERULION:-...}` substitutes only for a
                    # NULL value, so the shell passes the spaces through and
                    # the runner aborts on `[ ! -x "   " ]` — after its
                    # rebuild and after the SHM sweep. Verified against the
                    # real shell. This row asserted the opposite and
                    # approved the guard's unsafe path.
                    ("whitespace-only", "   ", True, "only whitespace",
                     floor + HOUR),
                    # This row uses the DEFAULT release path, so it exits
                    # at the exemption and never reaches the classifier —
                    # kept because that IS the exemption's happy path.
                    ("the default release path, fresh",
                     str(release), False, None, floor + HOUR),
                    # ...and this one does NOT, so it is the only row that
                    # reaches the CLI_FRESH accept branch. Without it the
                    # narrowing `verdict in (CLI_FRESH,
                    # CLI_UNVERIFIABLE)` to `== CLI_UNVERIFIABLE` runs green:
                    # the documented, supported case (an in-tree, provably
                    # fresh override) is silently refused.
                    ("fresh in-tree, NOT the default path",
                     str(debug), False, None, floor + HOUR),
                    # The path run_workspace.sh REBUILDS before it runs
                    # it: naming the default explicitly is the same ask as
                    # not setting it, so it must not be refused for
                    # staleness the next thirty seconds cure.
                    ("the default release path, stale",
                     str(release), False, None, floor - HOUR),
                    # THE case the exemption exists for, and the one it
                    # could not reach while it sat below `is_file()`: a
                    # fresh checkout, the binary not built yet, CERULION
                    # exported to the runner's own default by a CI wrapper
                    # or a shell rc. run_workspace.sh BUILDS that exact
                    # path; the identical run with CERULION unset succeeds,
                    # so refusing this one is a refusal of nothing.
                    ("the default release path, NOT BUILT YET",
                     str(release), False, None, None),
                    # The SAME unbuilt default, named through a symlinked
                    # checkout prefix. A lexical comparison misses it, and
                    # the `is_file()` check below then refuses a default
                    # the runner would have built — the exact false
                    # refusal hoisting the exemption was meant to end,
                    # reached one symlink further out. (`/tmp` is a
                    # symlink to `/private/tmp` on macOS, so this is not
                    # an exotic spelling.)
                    ("the default release path via a SYMLINKED prefix",
                     str(alias_release), False, None, None),
                    ("stale in-tree", str(debug), True, "older", floor + HOUR),
                    ("foreign", str(outside), True, "not this checkout",
                     floor + HOUR),
                    ("not a file", str(tree / "nope"), True, "not a file", floor + HOUR),
                    # run_workspace.sh DOES check `[ ! -x "$CERULION" ]`,
                    # but only after its rebuild and after the SHM sweep —
                    # the exact ordering the wiring arm requires this
                    # refusal to come before.
                    ("not executable", str(noexec), True, "not executable",
                     floor + HOUR),
                    # The runner `rm -rf`s the bench workspace's own
                    # target/debug (its freshest-wins guard) before running
                    # $CERULION, so an override living there is removed
                    # between this check and the run.
                    ("under the directory the runner deletes",
                     str(doomed_cli), True, "DELETES", floor + HOUR),
                    # A SYMLINK entry in that directory, pointing outside
                    # it. This is the row that discriminates at the CALL
                    # SITE: a resolved containment check follows the link
                    # out of the doomed tree and ACCEPTS it, while `rm -rf`
                    # deletes the entry and the override is gone before the
                    # run. The row above cannot see that — it is a real
                    # file, which both checks agree is doomed.
                    ("a symlink entry in the directory the runner deletes",
                     str(doomed_link), True, "DELETES", floor + HOUR),
                    # RELATIVE: this process and the runner do not share a
                    # working directory, so a relative override could name a
                    # different file in each — the check would then have
                    # vouched for the wrong binary.
                    ("relative", "target/release/cerulion", True,
                     "relative path", floor + HOUR),
                    # pathlib normalises a trailing separator away; the
                    # shell does not (ENOTDIR), so accepting it would
                    # vouch for a path the runner cannot exec.
                    ("trailing separator", str(release) + "/", True,
                     "path separator", floor + HOUR),
                ):
                    # release_ns None = the binary does not exist at all.
                    if release_ns is None:
                        if release.exists():
                            release.unlink()
                    else:
                        _touch_ns(release, release_ns)
                    _touch_ns(debug, floor + HOUR
                              if "fresh in-tree" in label else floor - HOUR)
                    if value is None:
                        os.environ.pop(bench.CERULION_BIN_ENV, None)
                    else:
                        os.environ[bench.CERULION_BIN_ENV] = value
                    try:
                        bench.refuse_foreign_cerulion_override("workspace")
                        raised = ""
                    except SystemExit as e:
                        raised = str(e)
                    if want_raise and not raised:
                        print(f"FAIL  [CERULION override: the {label} "
                              f"case must be refused, it was accepted]")
                        errors += 1
                    elif not want_raise and raised:
                        print(f"FAIL  [CERULION override: the {label} "
                              f"case must be accepted, got {raised!r}]")
                        errors += 1
                    elif want_raise and needle not in raised:
                        print(f"FAIL  [CERULION override: the {label} "
                              f"refusal must say {needle!r}, got {raised!r}]")
                        errors += 1
                    elif want_raise and "unset CERULION" not in raised:
                        print(f"FAIL  [CERULION override: the {label} "
                              f"refusal must name the remedy, got "
                              f"{raised!r}]")
                        errors += 1
                # The one arm that does NOT raise but proceeds anyway:
                # an in-tree override this tree cannot date. That stderr
                # warning is the entire safety margin of the accept
                # decision — delete it and "used a binary nobody can date,
                # loudly" becomes "used it silently" — so it is the one
                # line here that must not be deletable.
                saved_floor_fn2 = bench.cerulion_source_floor_ns
                try:
                    bench.cerulion_source_floor_ns = lambda: None
                    _touch_ns(debug, floor + HOUR)
                    os.environ[bench.CERULION_BIN_ENV] = str(debug)
                    err = io.StringIO()
                    raised = ""
                    try:
                        with contextlib.redirect_stderr(err):
                            bench.refuse_foreign_cerulion_override("workspace")
                    except SystemExit as e:
                        raised = str(e)
                    if raised:
                        print(f"FAIL  [CERULION override: an UNDATABLE "
                              f"in-tree override must proceed (refusing "
                              f"blocks a whole campaign because git "
                              f"hiccuped), got {raised!r}]")
                        errors += 1
                    elif "UNVERIFIED" not in err.getvalue() or \
                            str(debug) not in err.getvalue():
                        print(f"FAIL  [CERULION override: proceeding on an "
                              f"undatable override must WARN, naming the "
                              f"path — that warning is the whole margin of "
                              f"the accept decision, got "
                              f"{err.getvalue()!r}]")
                        errors += 1
                finally:
                    bench.cerulion_source_floor_ns = saved_floor_fn2
            finally:
                if doomed_link.exists() or doomed_link.is_symlink():
                    doomed_link.unlink()
                if doomed_cli.exists():
                    doomed_cli.unlink()
                if saved_env is None:
                    os.environ.pop(bench.CERULION_BIN_ENV, None)
                else:
                    os.environ[bench.CERULION_BIN_ENV] = saved_env
    finally:
        bench.REPO_ROOT = saved_root
        bench.cerulion_source_floor_ns = saved_floor_fn
        bench.shutil.which = saved_which
        if saved_ctd_outer is not None:
            os.environ["CARGO_TARGET_DIR"] = saved_ctd_outer

    # ---- a declined binary must SKIP LOUDLY, naming the reason -----------
    # Two claims, both tested where they matter. (a) The wrapper asks
    # STRICTLY, so an undatable
    # binary is declined here rather than validating this checkout — the
    # policy split the resolver's docstring describes. (b) The two REAL
    # arms render the reason on their own `skip` line: printing it on a
    # separate `note` line while the actionable line still said "no
    # `cerulion` binary" told an operator to build one they already had.
    saved_resolve = bench.resolve_cerulion_cli
    saved_finder = globals()["_find_cerulion_cli"]
    try:
        # A ZERO-ARGUMENT spy, deliberately: accepting a
        # keyword the wrapper no longer passes would tolerate a real
        # TypeError at the wrapper's only two production call sites going
        # unseen here — the suite would catch it elsewhere, this arm
        # would not.
        def spy():
            return (None, f"declined by the spy — {REBUILD_CMD}")

        bench.resolve_cerulion_cli = spy
        got = _find_cerulion_cli()
        if got != (None, f"declined by the spy — {REBUILD_CMD}"):
            print(f"FAIL  [cli skip loudness: the wrapper must hand the "
                  f"reason back to its caller, got {got!r}]")
            errors += 1

        marker = "DECLINED-MARKER"
        globals()["_find_cerulion_cli"] = lambda: (None, marker)
        for fn in (check_bench_graphs_validate,
                   check_pod_schema_hash_matches_the_generated_type):
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = fn()
            printed = buf.getvalue()
            if rc != 0:
                print(f"FAIL  [cli skip loudness: {fn.__name__} must SKIP "
                      f"(not fail) without a usable binary, got rc={rc}]")
                errors += 1
            elif "skip" not in printed or marker not in printed:
                print(f"FAIL  [cli skip loudness: {fn.__name__}'s skip line "
                      f"must carry the resolver's reason — 'no binary, "
                      f"build one' and 'a binary was found and declined' "
                      f"have different remedies — got {printed!r}]")
                errors += 1
    finally:
        bench.resolve_cerulion_cli = saved_resolve
        globals()["_find_cerulion_cli"] = saved_finder

    # ---- the guard must be WIRED, not merely defined ---------------------
    # Everything above drives the functions in isolation, so deleting every
    # call site would leave it all green while the sweep happily inherited a
    # foreign CERULION=. Read bench.py's own text for the call sites, and
    # for the ORDER in the one function where order matters.
    # REACH: this proves a call EXISTS, not that it is
    # unconditional — it would be satisfied by one nested under `if
    # False:` or inside a swallowing `except`. That is the standard limit
    # of an AST-presence check; what it does buy is that deleting the call
    # sites cannot leave the suite green, which is the regression it is
    # here for.
    for fn in ("cmd_workspace", "cmd_full", "cmd_smoke", "run_workspace_leg"):
        target = getattr(bench, fn, None)
        if target is None:
            print(f"FAIL  [CERULION override wiring: bench.py has no {fn} — "
                  f"renamed or removed, so this arm is no longer asking "
                  f"about the code that runs]")
            errors += 1
            continue
        if not _calls_in(target, "refuse_foreign_cerulion_override"):
            print(f"FAIL  [CERULION override wiring: {fn} never asks — an "
                  f"exported override reaches run_workspace.sh unchallenged]")
            errors += 1
    asks = _calls_in(bench.run_workspace_leg,
                     "refuse_foreign_cerulion_override")
    sweeps = _calls_in(bench.run_workspace_leg, "cleanup_iceoryx")
    if not asks or not sweeps:
        print(f"FAIL  [CERULION override wiring: run_workspace_leg does not "
              f"both ask (found={bool(asks)}) and sweep SHM "
              f"(found={bool(sweeps)})]")
        errors += 1
    elif asks[0] > sweeps[0]:
        print("FAIL  [CERULION override wiring: run_workspace_leg refuses "
              "AFTER cleanup_iceoryx() — the refusal would first destroy "
              "SHM state shared with every other tenant on the machine]")
        errors += 1

    if errors == 0:
        print("ok    [cli freshness: the resolver declines a stale or "
              "foreign binary and uses the freshest in-tree one, an "
              "undatable binary is used but flagged, the CERULION= "
              "override refusal holds, and all four call sites are wired "
              "(the leg's ahead of the SHM sweep)]")
    return errors


def main() -> int:
    # A typo'd flag must not read as "no flag": `--nolock` would otherwise
    # be silently ignored, the gate would refuse, and the operator would be
    # told to pass the option they thought they had passed.
    # The gate above runs under `__name__ == "__main__"`, so importing
    # this module and calling `main()` would run every check — including
    # the ones that write into the real bench workspace — with no lock and
    # no note. Children import the module but never call `main()`.
    if not _RUN_LOCK_HELD and not _OVERRIDE:
        print(f"parity checker: main() was called without the run lock "
              f"(imported rather than run?). Run the script, or pass "
              f"{NO_LOCK_FLAG} to say the runs are serialized some other "
              f"way.", file=sys.stderr)
        return _EX_USAGE
    unknown = [a for a in sys.argv[1:] if a != NO_LOCK_FLAG]
    if unknown:
        print(f"parity checker: unrecognized argument(s) "
              f"{' '.join(unknown)}. The only option is {NO_LOCK_FLAG} "
              f"(run without serializing against another instance).",
              file=sys.stderr)
        return _EX_USAGE
    errors = 0
    # PREFLIGHT the one environment value that makes every cargo question
    # unanswerable. bench.py's readers REFUSE it (they match cargo, which
    # exits 101), and a refusal raised from inside an arm would take this
    # whole process down partway through — reading as a crash rather than as
    # the configuration problem it is, with every later arm unrun. Asked
    # once, here, so the checker REPORTS it and stops with its own verdict.
    try:
        bench.cargo_target_dir_setting()
    except SystemExit as e:
        print(f"FAIL  [parity checker preflight: {e}]")
        return 1

    with tempfile.TemporaryDirectory(prefix="parity_") as td:
        for i, (desc, values) in enumerate(FIXTURES):
            bin_path = Path(td) / f"fixture_{i}.bin"
            bin_path.write_bytes(_make_blob(values))

            bench_p50 = bench.read_p50_ns(bin_path)
            csv_p50 = compile_csv.percentile(sorted(values), 0.5)
            summarize_p50 = compile_csv.summarize(list(values))["p50"]

            if bench_p50 != csv_p50 or bench_p50 != summarize_p50:
                print(f"FAIL  [{desc}]")
                print(f"      values={values}")
                print(f"      bench.py::read_p50_ns          => {bench_p50}")
                print(f"      compile_csv.py::percentile     => {csv_p50}")
                print(f"      compile_csv.py::summarize p50  => {summarize_p50}")
                errors += 1
            else:
                print(f"ok    [{desc}]  p50={bench_p50}")

    print()
    errors += check_run_lock_serializes_one_checkout()
    print()
    errors += check_rep_aggregation()
    print()
    errors += check_torn_dump_refused()
    print()
    errors += check_smoke_sample_contract()
    print()
    errors += check_plot_variant_gate()
    print()
    errors += check_usage_wiring()
    print()
    errors += check_one_line_encoder()
    print()
    errors += check_did_not_sustain_exclusion()
    print()
    errors += check_compile_csv_raw_dir_out_default()
    print()
    errors += check_workspace_size_contract()
    print()
    errors += check_ros2_sizes_decimal()
    print()
    errors += check_ros2_image_axis_gate()
    print()
    errors += check_type_class_visuals()
    print()
    errors += check_type_class_footnote()
    print()
    errors += check_type_class_legend()
    print()
    errors += check_raw_name_class_agreement()
    print()
    errors += check_smoke_gates_refuse_an_unreadable_log()
    print()
    errors += check_class_carriers_refuse_at_the_mint()
    print()
    errors += check_bench_graph_schemas_resolve()
    print()
    errors += check_pod_schema_is_never_rewritten()
    print()
    errors += check_pod_sweep_leaves_the_checkout_clean()
    print()
    errors += check_bench_graphs_validate()
    print()
    errors += check_pod_registration_reconstruction()
    print()
    errors += check_pod_schema_hash_matches_the_generated_type()
    print()
    errors += check_cxx_code_only()
    print()
    errors += check_image_stamp_codec()
    print()
    errors += check_cerulion_binary_freshness()
    print()
    errors += check_sample_gate_accounting()
    print()
    errors += check_oracles_are_cmake_targets()
    print()
    errors += check_sink_set_is_accounted()
    print()
    errors += check_backstop_is_namespace_gated()
    print()
    errors += check_delivery_drop_columns()
    print()
    errors += check_shell_entry_points_are_unsupported()
    print()
    errors += check_empty_cargo_target_dir_is_refused()
    print()
    errors += check_cli_source_closure()
    errors += check_fixture_trees_do_not_spend_the_fallback_note()
    print()
    errors += check_cerulion_override_is_documented()
    print()
    errors += check_loaned_column_is_first_match()
    print()
    errors += check_loan_lane_detector()
    print()
    errors += check_custom_overlay_braces()
    print()
    errors += check_plot_arms_skip_without_matplotlib()
    print()
    errors += check_smoke_type_class_coverage()
    print()
    errors += check_smoke_exit_contract()
    print()
    errors += check_smoke_contract_restores_on_a_throw()
    errors += check_preflight_order()
    errors += check_pids_tokenizer()

    print()
    if errors:
        # No hand-written contract list on the FAIL path: it had already
        # gone stale (did_not_sustain was missing) and every check above
        # prints its own attributable FAIL line naming what broke. The PASS
        # line below does still enumerate — it carries no failure to
        # attribute, so a name there is the only signal that a check ran.
        print(f"FAIL: {errors} divergence(s) — see the FAIL line(s) above; "
              f"the bench.py / compile_csv.py / plot.py / usage_sampler.py "
              f"/ run_workspace.sh / ros2/run_bench.sh contracts are out of "
              f"sync.")
        return 1
    print(f"PASS: one checker at a time per checkout (the lock is taken "
          f"before the sibling imports, keyed by this directory, and a "
          f"second process is refused it); the smoke scenario hands its "
          f"`bench` globals back on a throw in either of its two phases; "
          f"the CLI freshness floor is `cerulion_cli`'s own manifest-derived "
          f"member closure, so editing a member it does not link leaves a "
          f"good binary FRESH; all {len(FIXTURES)} percentile fixtures agree "
          f"between "
          f"bench.py::read_p50_ns and compile_csv.py::percentile/summarize, "
          f"the A1 cross-rep aggregation matches its hand oracles, both "
          f"readers refuse torn dumps, the smoke dispatcher's sample counts "
          f"agree across spellings, plot.py's variant gate holds, the "
          f"CER_BENCH_USAGE native wiring (per-size invocations, sidecar "
          f"naming + stale guard, exact-set gate) drives clean both on and "
          f"off, the one_line sidecar encoder, the did-not-sustain "
          f"exclusion, the compile-csv raw-dir output default and the smoke "
          f"no-baseline exit contract hold, and "
          f"the type-class axis holds (run_bench.sh refuses every "
          f"unenumerated image axis, image twins get their own visuals, the "
          f"matched-quantity footnote needs both classes, a paired "
          f"figure names BOTH classes in the legend so a --release render "
          f"still identifies them, neither runner accepts a raw prefix "
          f"carrying the other class's pinned grammar over every shape "
          f"the enumeration mints, the per-size smoke gates refuse an "
          f"unreadable or marker-less log on a run that succeeded while "
          f"leaving a failed run its own verdict, the loaned column takes "
          f"the FIRST match, the force-rate knob refuses its own bad "
          f"input, every class-carrying identity refuses at the mint, "
          f"the loan-lane "
          f"detector sees every loan cell incl. zc, custom overlays draw "
          f"rate braces only under an explicit --variant, the runner "
          f"refuses misaligned pod sizes before building, both runners read "
          f"sizes as decimal only and take exactly the pinned sweep set, "
          f"neither runs a zero-size sweep, the plot arms skip loudly "
          f"without matplotlib, smoke covers both classes per stack, the "
          f"image class's stamp codec is the codec the shipping MsgAdapter "
          f"calls and the class dispatch refuses an unmeasurable host "
          f"before running one, "
          + (f"its compiled hand oracles agree, "
             if _STAMP_ORACLE_RAN else
             f"its compiled oracle SKIPPED for want of a C++ compiler, ")
          + f"and no stale or foreign `cerulion` can "
          f"validate this checkout, an empty CARGO_TARGET_DIR is "
          f"refused the way cargo refuses it by every reader that asks, "
          f"and every latency sink COUNTS and REPORTS the echoes its "
          f"stamp gate declines"
          + ("" if _SAMPLE_GATE_ORACLE_RAN else
             " — though that gate's compiled oracle SKIPPED for want of a "
             "C++ compiler")
          + f"; those counts now reach the published CSV row, pooled like "
          f"iterations and never summed over the reps that happened to "
          f"answer; colcon COMPILES both hand oracles, so a failed "
          f"static_assert breaks the CONTAINER build rather than waiting "
          f"for a desk run of this checker; every C++ source in the bench "
          f"package is accounted for as a judged sink or a declared "
          f"non-sink, so a fourth sink cannot escape by renaming its "
          f"locals; every recorded pid — node AND daemon — is signalled "
          f"and polled by identity; the blanket bench-binary sweep is "
          f"spelled once, behind an assertion that this shell holds a PID "
          f"namespace of its own; and both runner scripts and the README "
          f"say a direct invocation is not a supported entry point"
          + f").")
    return 0


if __name__ == "__main__":
    sys.exit(main())
