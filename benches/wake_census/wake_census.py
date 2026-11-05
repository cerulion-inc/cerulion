#!/usr/bin/env python3
"""Wake census: how often each process of an idle graph wakes up.

Measurement only. It reports; it gates nothing.

Usage:

    # every process under a running `cerulion graph run` supervisor
    python3 benches/wake_census/wake_census.py --parent-pid <supervisor pid> --window 10

    # explicit processes
    python3 benches/wake_census/wake_census.py --pid 1234 --pid 1235 --window 10

    # machine-readable
    python3 benches/wake_census/wake_census.py --parent-pid <pid> --json

Start the graph, let it go idle (no data flowing), then run the census. It
reads each process's counters, sleeps --window seconds, reads them again and
prints the per-second rate of each counter, per process and in total. A
process that exits during the window is reported as gone, never as zero.

Counters, Linux, per thread in /proc/<pid>/task/<tid>/ and summed per
process. A thread that starts during the window counts from zero; a thread
that exits during it loses what it counted after the first read, and the
report says how many did.

    vol_csw          voluntary_ctxt_switches (status): the thread blocked
                     and was later woken, whatever woke it (data, a futex, a
                     timer expiry).
    nonvol_csw       nonvoluntary_ctxt_switches (status): the thread was
                     preempted.
    runs             the third field of schedstat: how many times the thread
                     was switched onto a CPU, for any reason (about vol_csw +
                     nonvol_csw).
    wakeups          nr_wakeups from sched (spelled se.statistics.nr_wakeups
                     before Linux 5.16): every wakeup of the thread, any
                     cause. Present only when the kernel keeps scheduler
                     statistics (sysctl kernel.sched_schedstats=1).
    irq_ctx_wakes    with --perf: sched:sched_waking events for the process's
                     threads issued while the CPU ran the idle task (pid 0,
                     `swapper`) or `ksoftirqd`, from `perf record -a` over the
                     window. sched_waking fires in the waker's context, so a
                     wakeup sent by another task carries that task's name;
                     what is left is wakeups raised from interrupt context on
                     an idle CPU, chiefly timer expiries. A timer that expires
                     while another task is on that CPU is attributed to that
                     task and is not counted. Needs perf and permission to
                     trace system wide (root, or kernel.perf_event_paranoid
                     <= -1). Divided by the window perf recorded.

Counters, macOS (proc_pid_rusage RUSAGE_INFO_V0 and proc_pidinfo
PROC_PIDTASKINFO):

    idle_wakeups       ri_pkg_idle_wkups: wakeups that took the package out
                       of idle.
    interrupt_wakeups  ri_interrupt_wkups: wakeups from an interrupt (timers
                       included).
    csw                pti_csw: context switches (a 32-bit counter; the delta
                       is taken modulo 2^32).

A counter this host does not provide reads `unavailable`, one the caller may
not read (another user's process without the privilege) reads `unreadable`;
neither is ever shown as zero.
"""

import argparse
import ctypes
import ctypes.util
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

UNREADABLE = "unreadable"
UNAVAILABLE = "unavailable"

# ---------------------------------------------------------------------------
# Process selection.
# ---------------------------------------------------------------------------


def children_of(pid):
    """Direct children of pid: the union over its threads' /proc children
    lists (a child belongs to the thread that forked it), else `pgrep -P`."""
    task_dir = f"/proc/{pid}/task"
    if os.path.isdir(task_dir):
        kids = set()
        try:
            tids = os.listdir(task_dir)
        except OSError:
            return []
        for tid in tids:
            try:
                with open(f"{task_dir}/{tid}/children") as f:
                    kids.update(int(x) for x in f.read().split())
            except OSError:
                pass
        return sorted(kids)
    try:
        out = subprocess.run(
            ["pgrep", "-P", str(pid)], capture_output=True, text=True, check=False
        ).stdout
    except FileNotFoundError:
        return []
    return [int(x) for x in out.split()]


def descendants(root):
    """root and every process below it."""
    seen, stack = [], [root]
    while stack:
        p = stack.pop()
        if p in seen:
            continue
        seen.append(p)
        stack.extend(children_of(p))
    return seen


def comm_of(pid):
    try:
        with open(f"/proc/{pid}/comm") as f:
            return f.read().strip()
    except OSError:
        pass
    try:
        out = subprocess.run(
            ["ps", "-o", "comm=", "-p", str(pid)],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip()
        return os.path.basename(out) or "?"
    except FileNotFoundError:
        return "?"


# ---------------------------------------------------------------------------
# Linux counters.
# ---------------------------------------------------------------------------

LINUX_KEYS = ("vol_csw", "nonvol_csw", "runs", "wakeups")


def parse_status_csw(text):
    """(voluntary, nonvoluntary) context switches from /proc/.../status text."""
    vol = re.search(r"^voluntary_ctxt_switches:\s+(\d+)", text, re.M)
    non = re.search(r"^nonvoluntary_ctxt_switches:\s+(\d+)", text, re.M)
    return (int(vol.group(1)) if vol else None, int(non.group(1)) if non else None)


def parse_schedstat_runs(text):
    """The run count (third field) of a /proc/.../schedstat line."""
    fields = text.split()
    return int(fields[2]) if len(fields) >= 3 else None


def parse_sched_wakeups(text):
    """nr_wakeups from /proc/.../sched text (either spelling), or None."""
    m = re.search(r"^(?:se\.statistics\.)?nr_wakeups\s*:\s*(\d+)", text, re.M)
    return int(m.group(1)) if m else None


def read_text(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def linux_thread_sample(base):
    """One thread's counters: an int, or UNREADABLE / UNAVAILABLE."""
    out = {}
    status = read_text(f"{base}/status")
    vol, non = parse_status_csw(status) if status is not None else (None, None)
    out["vol_csw"] = vol if vol is not None else UNREADABLE
    out["nonvol_csw"] = non if non is not None else UNREADABLE
    stat = read_text(f"{base}/schedstat")
    runs = parse_schedstat_runs(stat) if stat is not None else None
    out["runs"] = runs if runs is not None else (UNAVAILABLE if stat is not None else UNREADABLE)
    sched = read_text(f"{base}/sched")
    if sched is None:
        out["wakeups"] = UNREADABLE
    else:
        w = parse_sched_wakeups(sched)
        out["wakeups"] = w if w is not None and schedstats_enabled() else UNAVAILABLE
    return out


def linux_sample(pid):
    """{tid: counters} for pid's threads, or None when the process is gone."""
    task_dir = f"/proc/{pid}/task"
    try:
        tids = os.listdir(task_dir)
    except OSError:
        return None
    return {int(t): linux_thread_sample(f"{task_dir}/{t}") for t in tids}


_SCHEDSTATS = None


def schedstats_enabled():
    global _SCHEDSTATS
    if _SCHEDSTATS is None:
        _SCHEDSTATS = (read_text("/proc/sys/kernel/sched_schedstats") or "").strip() == "1"
    return _SCHEDSTATS


def linux_rates(before, after, seconds):
    """Per-second per-process rates from two {tid: counters} samples, and the
    number of threads that exited in between."""
    out = {}
    for k in LINUX_KEYS:
        total, reason = 0, None
        for tid, a in after.items():
            v = a[k]
            b = before.get(tid, {}).get(k, 0)
            if isinstance(v, str) or isinstance(b, str):
                reason = v if isinstance(v, str) else b
                break
            total += v - b
        out[k] = reason if reason is not None else total / seconds
    exited = sum(1 for tid in before if tid not in after)
    return out, exited


def thread_ids(pid):
    try:
        return [int(t) for t in os.listdir(f"/proc/{pid}/task")]
    except OSError:
        return []


# A line of `perf script`: "<comm> <tid> [<cpu>] <ts>: sched:sched_waking: comm=X pid=N ..."
_WAKING_RE = re.compile(
    r"^\s*(?P<waker>\S.*?)\s+(?P<wpid>-?\d+)\s+\[\d+\]\s+(?P<ts>[\d.]+):.*sched:sched_waking:.*\bpid=(?P<pid>\d+)"
)


def parse_perf_wakings(text, tid_to_pid):
    """(counts per process, recorded seconds) from `perf script` text: the
    sched_waking events whose waking context (the line's leading comm and pid)
    is the idle task (pid 0, `swapper`) or `ksoftirqd/N`. `tid_to_pid` maps
    each thread id to its process; the seconds span the first to last event."""
    counts, first, last = {}, None, None
    for line in text.splitlines():
        m = _WAKING_RE.match(line)
        if not m:
            continue
        ts = float(m.group("ts"))
        first = ts if first is None else first
        last = ts
        pid = tid_to_pid.get(int(m.group("pid")))
        if pid is None:
            continue
        waker = m.group("waker")
        if int(m.group("wpid")) == 0 or waker.startswith(("swapper", "ksoftirqd")):
            counts[pid] = counts.get(pid, 0) + 1
    span = (last - first) if first is not None and last > first else None
    return counts, span


def perf_irq_wakes(pids, window):
    """(counts per pid, seconds recorded) over `window` seconds via perf, or
    an error string."""
    perf = shutil.which("perf")
    if not perf:
        return "perf not installed"
    with tempfile.TemporaryDirectory() as d:
        data = os.path.join(d, "perf.data")
        rec = subprocess.run(
            [perf, "record", "-q", "-a", "-e", "sched:sched_waking", "-o", data, "--", "sleep", str(window)],
            capture_output=True,
            text=True,
            check=False,
        )
        if rec.returncode != 0:
            return f"perf record failed: {rec.stderr.strip()[:200]}"
        # Threads started during the window are mapped too.
        tid_to_pid = {t: p for p in pids for t in thread_ids(p)}
        out = subprocess.run([perf, "script", "-i", data], capture_output=True, text=True, check=False)
        if out.returncode != 0:
            return f"perf script failed: {out.stderr.strip()[:200]}"
    counts, span = parse_perf_wakings(out.stdout, tid_to_pid)
    return counts, (span if span else window)


# ---------------------------------------------------------------------------
# macOS counters.
# ---------------------------------------------------------------------------


class RusageInfoV0(ctypes.Structure):
    _fields_ = [
        ("ri_uuid", ctypes.c_uint8 * 16),
        ("ri_user_time", ctypes.c_uint64),
        ("ri_system_time", ctypes.c_uint64),
        ("ri_pkg_idle_wkups", ctypes.c_uint64),
        ("ri_interrupt_wkups", ctypes.c_uint64),
        ("ri_pageins", ctypes.c_uint64),
        ("ri_wired_size", ctypes.c_uint64),
        ("ri_resident_size", ctypes.c_uint64),
        ("ri_phys_footprint", ctypes.c_uint64),
        ("ri_proc_start_abstime", ctypes.c_uint64),
        ("ri_proc_exit_abstime", ctypes.c_uint64),
    ]


class ProcTaskInfo(ctypes.Structure):
    _fields_ = [
        ("pti_virtual_size", ctypes.c_uint64),
        ("pti_resident_size", ctypes.c_uint64),
        ("pti_total_user", ctypes.c_uint64),
        ("pti_total_system", ctypes.c_uint64),
        ("pti_threads_user", ctypes.c_uint64),
        ("pti_threads_system", ctypes.c_uint64),
        ("pti_policy", ctypes.c_int32),
        ("pti_faults", ctypes.c_int32),
        ("pti_pageins", ctypes.c_int32),
        ("pti_cow_faults", ctypes.c_int32),
        ("pti_messages_sent", ctypes.c_int32),
        ("pti_messages_received", ctypes.c_int32),
        ("pti_syscalls_mach", ctypes.c_int32),
        ("pti_syscalls_unix", ctypes.c_int32),
        ("pti_csw", ctypes.c_int32),
        ("pti_threadnum", ctypes.c_int32),
        ("pti_numrunning", ctypes.c_int32),
        ("pti_priority", ctypes.c_int32),
    ]


RUSAGE_INFO_V0 = 0
PROC_PIDTASKINFO = 4
ESRCH = 3
MACOS_KEYS = ("idle_wakeups", "interrupt_wakeups", "csw")
_libproc = None


def libproc():
    global _libproc
    if _libproc is None:
        _libproc = ctypes.CDLL(ctypes.util.find_library("proc") or "/usr/lib/libproc.dylib", use_errno=True)
    return _libproc


def macos_sample(pid):
    """Counters for pid, or None when the process is gone."""
    lib = libproc()
    out = {k: UNREADABLE for k in MACOS_KEYS}
    ru = RusageInfoV0()
    if lib.proc_pid_rusage(ctypes.c_int(pid), ctypes.c_int(RUSAGE_INFO_V0), ctypes.byref(ru)) == 0:
        out["idle_wakeups"] = ru.ri_pkg_idle_wkups
        out["interrupt_wakeups"] = ru.ri_interrupt_wkups
    elif ctypes.get_errno() == ESRCH:
        return None
    ti = ProcTaskInfo()
    n = lib.proc_pidinfo(ctypes.c_int(pid), PROC_PIDTASKINFO, ctypes.c_uint64(0), ctypes.byref(ti), ctypes.sizeof(ti))
    if n == ctypes.sizeof(ti):
        out["csw"] = ti.pti_csw & 0xFFFFFFFF
    return out


def rates(before, after, seconds, wrap=None):
    """Per-second deltas of every counter; a non-numeric reading is passed
    through as its reason. `wrap` names counters taken modulo 2^32."""
    out = {}
    for k, a in before.items():
        b = after.get(k)
        if isinstance(a, str) or isinstance(b, str) or b is None:
            out[k] = a if isinstance(a, str) else (b if isinstance(b, str) else UNREADABLE)
            continue
        d = b - a
        if wrap and k in wrap:
            d %= 1 << 32
        out[k] = d / seconds
    return out


# ---------------------------------------------------------------------------
# The census.
# ---------------------------------------------------------------------------


def census(pids, window, use_perf):
    linux = sys.platform.startswith("linux")
    sample = linux_sample if linux else macos_sample
    names = {p: comm_of(p) for p in pids}
    t0 = time.monotonic()
    before = {p: sample(p) for p in pids}
    perf = None
    if use_perf and linux:
        perf = perf_irq_wakes(pids, window)
    else:
        time.sleep(window)
    seconds = time.monotonic() - t0
    after = {p: sample(p) for p in pids}
    rows, notes = [], []
    for p in pids:
        row = {"pid": p, "comm": names[p]}
        if before[p] is None or after[p] is None:
            row["gone"] = True
        elif linux:
            r, exited = linux_rates(before[p], after[p], seconds)
            row.update(r)
            if exited:
                row["threads_exited"] = exited
            if isinstance(perf, tuple):
                counts, span = perf
                row["irq_ctx_wakes"] = counts.get(p, 0) / span
            elif use_perf:
                row["irq_ctx_wakes"] = UNAVAILABLE
        else:
            row.update(rates(before[p], after[p], seconds, wrap={"csw"}))
        rows.append(row)
    if linux and not schedstats_enabled():
        notes.append("wakeups unavailable: kernel.sched_schedstats=0")
    if isinstance(perf, str):
        notes.append(f"irq_ctx_wakes unavailable: {perf}")
    if any(r.get("threads_exited") for r in rows):
        notes.append("threads_exited: threads that exited during the window; what they counted after the first read is lost")
    return {"platform": sys.platform, "window_s": seconds, "rows": rows, "notes": notes}


def render(result):
    rows = result["rows"]
    keys = []
    for r in rows:
        for k in r:
            if k not in ("pid", "comm", "gone") and k not in keys:
                keys.append(k)
    lines = [f"wake census over {result['window_s']:.1f} s on {result['platform']} (per second)"]
    lines.append(f"{'pid':>8} {'comm':<20}" + "".join(f" {k:>18}" for k in keys))
    totals = {k: 0.0 for k in keys}
    for r in rows:
        if r.get("gone"):
            lines.append(f"{r['pid']:>8} {r['comm'][:20]:<20} gone during the window")
            continue
        cells = []
        for k in keys:
            v = r.get(k, "-")
            if isinstance(v, str):
                cells.append(f" {v:>18}")
            else:
                totals[k] += v
                cells.append(f" {v:>18.1f}")
        lines.append(f"{r['pid']:>8} {r['comm'][:20]:<20}" + "".join(cells))
    lines.append(f"{'total':>8} {'':<20}" + "".join(f" {totals[k]:>18.1f}" for k in keys))
    lines.extend(f"note: {n}" for n in result["notes"])
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Count an idle graph's wakeups per process.")
    ap.add_argument("--pid", type=int, action="append", default=[], help="a process to count (repeatable)")
    ap.add_argument("--parent-pid", type=int, help="count this process and every process below it")
    ap.add_argument("--window", type=float, default=10.0, help="seconds to count over (default 10)")
    ap.add_argument("--perf", action="store_true", help="Linux: also count wakeups raised from interrupt context with perf")
    ap.add_argument("--json", action="store_true", help="print JSON instead of a table")
    args = ap.parse_args(argv)
    pids = list(args.pid)
    if args.parent_pid is not None:
        pids.extend(p for p in descendants(args.parent_pid) if p not in pids)
    if not pids:
        ap.error("name at least one --pid or a --parent-pid")
    if args.window <= 0:
        ap.error("--window must be positive")
    result = census(pids, args.window, args.perf)
    print(json.dumps(result, indent=2) if args.json else render(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
