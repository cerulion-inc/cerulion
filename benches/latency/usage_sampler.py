#!/usr/bin/env python3
"""Per-cell CPU + memory usage sampler (the usage sidecars).

Given a target process set — an explicit PID list, a parent PID to
DESCEND from (rescanned every tick, so children spawned after start are
picked up), or a docker container name resolved from the HOST — this
script samples, on the host's /proc:

    CPU     /proc/<pid>/stat utime+stime deltas → cpu_pct (% of ONE
            core; a multithreaded process can exceed 100), at
            --stat-hz (default 5 Hz)
    RSS     /proc/<pid>/status VmRSS (kB)         at --stat-hz
    PSS     /proc/<pid>/smaps_rollup Pss (kB)     at --pss-hz (default
            1 Hz — smaps_rollup walks the VMA list and is pricier)

and appends rows to ONE `.usage.csv` sidecar (`--out`), written beside
the cell's `.bin` by the callers (bench.py / run_workspace.sh):

    ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb

    - ts_ns    CLOCK_MONOTONIC_RAW-ish (time.monotonic_ns) — a per-run
               relative axis, NOT wall time; only deltas are meaningful
    - cpu_pct  empty on a pid's FIRST sighting (no delta yet)
    - pss_kb   empty on non-PSS ticks, and empty when smaps_rollup is
               unreadable (reading another user's smaps_rollup needs
               ptrace access — e.g. a root-owned chrt/sudo bench tree
               sampled by an unprivileged sampler). Denials are counted
               in the footer, never silently zeroed.

WHY PSS AND RSS BOTH: for SHM-heavy processes (iceoryx2 pools mapped
into every participant) RSS double-counts every shared mapping in every
process, so a summed-RSS "memory use" is affirmatively wrong; PSS
divides each shared page by its mapper count and SUMS correctly across a
cell's processes. Both are recorded, labeled, so the double-count is
visible instead of silently quoted.

Header lines (leading '#') record scope/cadence/host facts; a footer
line records the SAMPLER'S OWN cpu cost (os.times() over the run) —
the observer-cost number METHODOLOGY.md cites. Two channels sit
OUTSIDE that self-accounting, both bounded and documented rather than
silently uncounted: (a) child-process CPU — the `docker inspect` polls
run as subprocesses whose ticks land in os.times() children fields,
which the footer does not sum (negligible after container start: one
poll per docker-wait tick, none once the init PID resolves); (b)
target-side perturbation — reading smaps_rollup walks the TARGET's VMA
list under its mmap_lock, so a page-faulting target can be briefly
delayed by the observer; that cost lands in the target's numbers, not
the sampler's footer (another reason usage runs are companion runs,
never headline latency rows). No number here is ever fabricated:
unreadable = empty + counted, never zero (Principle #13).

Docker cells (`--docker-name`): the sampler polls
`docker inspect -f '{{.State.Pid}}'` (bounded by --docker-wait-s) for
the container's init PID as seen from the HOST pid namespace, then
descends from it exactly like --parent-pid (scope stays `procs` for
CPU/RSS — host /proc shows container processes). MEMORY caveat: ros2
containers run as the image default user (root), so an unprivileged
host sampler is smaps_rollup-DENIED for the whole tree (PSS needs
ptrace access). When a PSS tick in docker mode is denied for EVERY
process (and read nothing), the sampler falls back to the container's
cgroup memory.stat and writes ONE `scope=cgroup` row for that tick
(pid=0, comm=`cgroup`, pss_kb = anon+file kB — the container TOTAL,
labeled by a `# cgroup_fallback:` marker line; v1 fallback rss+cache).
A cgroup total is NOT comparable to per-process PSS — plot_usage.py
renders it as its own labeled series, never summed with per-proc rows.
Exits when the container/parent dies, on --duration-s, or on
SIGTERM/SIGINT (the callers stop it explicitly).

Linux-only (needs /proc): any other OS is a loud exit 2, never a
fabricated sidecar.

Self-check: `python3 usage_sampler.py --self-test` spawns a CPU-spinning,
memory-growing python child and asserts cpu_pct sanity + RSS growth.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

STOP = False


def _on_signal(signum, frame):  # noqa: ARG001 (signature fixed by signal)
    global STOP
    STOP = True


# ---------------------------------------------------------------------------
# /proc readers
# ---------------------------------------------------------------------------

def read_stat_cpu_ticks(pid: int) -> Optional[Tuple[str, int, int]]:
    """(comm, utime+stime ticks, starttime) from /proc/<pid>/stat, or
    None if gone.

    The comm field is parenthesized and may itself contain spaces or
    ')' — split on the LAST ')' (the documented parse). starttime
    (field 22, ticks since boot) identifies the process INSTANCE: a
    recycled pid gets a different starttime, so the delta baseline
    keyed on (pid, starttime) can never inherit a dead process's tick
    count (which would mint one garbage — possibly negative — cpu_pct
    row)."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
    except (OSError, ValueError):
        return None
    try:
        head, _, tail = raw.rpartition(")")
        comm = head.partition("(")[2]
        fields = tail.split()
        # tail starts at field 3 (state); utime/stime are fields 14/15
        # (1-indexed) → tail indices 11/12; starttime is field 22 → 19.
        return comm, int(fields[11]) + int(fields[12]), int(fields[19])
    except (IndexError, ValueError):
        return None


def read_rss_kb(pid: int) -> Optional[int]:
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except (OSError, ValueError):
        pass
    return None


def read_pss_kb(pid: int) -> Tuple[Optional[int], bool]:
    """(pss_kb, denied). denied=True on a permission failure — counted
    by the caller, never silently zeroed."""
    try:
        with open(f"/proc/{pid}/smaps_rollup") as f:
            for line in f:
                if line.startswith("Pss:"):
                    return int(line.split()[1]), False
    except PermissionError:
        return None, True
    except (OSError, ValueError):
        pass
    return None, False


# ---------------------------------------------------------------------------
# Target discovery
# ---------------------------------------------------------------------------

def _children_via_proc_children(pid: int) -> Optional[List[int]]:
    """Kernel CONFIG_PROC_CHILDREN fast path; None when unavailable."""
    kids: List[int] = []
    task_dir = Path(f"/proc/{pid}/task")
    try:
        tasks = list(task_dir.iterdir())
    except OSError:
        return []   # pid died mid-walk: no children, NOT a kernel gap
    saw_file = False
    for t in tasks:
        try:
            txt = (t / "children").read_text()
            saw_file = True
        except OSError:
            continue
        kids.extend(int(c) for c in txt.split())
    return kids if saw_file else None


def _ppid_map() -> Dict[int, int]:
    """Full /proc scan fallback: {pid: ppid}."""
    out: Dict[int, int] = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            tail = Path(f"/proc/{pid}/stat").read_text().rpartition(")")[2]
            out[pid] = int(tail.split()[1])
        except (OSError, ValueError, IndexError):
            continue
    return out


class Discovery:
    """Descendant discovery with the fast children-file path when the
    kernel has it, else a full-/proc ppid scan. Records which was used
    (the sidecar header names it — the scan path is the pricier one)."""

    def __init__(self) -> None:
        self.mode: Optional[str] = None

    def descendants(self, root: int) -> Set[int]:
        found: Set[int] = set()
        frontier = [root]
        use_children = self.mode in (None, "children")
        while frontier:
            pid = frontier.pop()
            if pid in found:
                continue
            found.add(pid)
            kids: Optional[List[int]] = None
            if use_children:
                kids = _children_via_proc_children(pid)
                if kids is None and self.mode is None:
                    use_children = False
                elif self.mode is None:
                    self.mode = "children"
            if kids is None:
                if self.mode != "scan":
                    self.mode = "scan"
                ppids = _ppid_map()
                # one scan resolves the whole tree
                remaining = set(ppids) - found
                changed = True
                while changed:
                    changed = False
                    for pid2 in list(remaining):
                        if ppids.get(pid2) in found:
                            found.add(pid2)
                            remaining.discard(pid2)
                            changed = True
                return found
            frontier.extend(kids)
        return found


def docker_container_pid(name: str) -> Optional[int]:
    try:
        r = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Pid}}", name],
            capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if r.returncode != 0:
        return None
    try:
        pid = int(r.stdout.strip())
    except ValueError:
        return None
    return pid if pid > 0 else None


def cgroup_mem_dir(pid: int) -> Optional[Path]:
    """The memory-accounting cgroup dir of `pid`, from /proc/<pid>/cgroup.

    cgroup v2 (unified): the `0::<path>` line → /sys/fs/cgroup<path>.
    cgroup v1 (legacy/hybrid, e.g. 5.15-tegra defaults): the line whose
    controller list contains `memory` → /sys/fs/cgroup/memory<path>."""
    try:
        lines = Path(f"/proc/{pid}/cgroup").read_text().splitlines()
    except OSError:
        return None
    v1_mem: Optional[Path] = None
    for line in lines:
        parts = line.split(":", 2)
        if len(parts) != 3:
            continue
        _, controllers, path = parts
        if controllers == "":               # v2 unified line
            d = Path("/sys/fs/cgroup" + path)
            if (d / "memory.stat").exists():
                return d
        elif "memory" in controllers.split(","):
            v1_mem = Path("/sys/fs/cgroup/memory" + path)
    if v1_mem is not None and (v1_mem / "memory.stat").exists():
        return v1_mem
    return None


def read_cgroup_mem_kb(cg_dir: Path) -> Optional[int]:
    """anon+file kB from the cgroup's memory.stat (v2 keys `anon`/`file`;
    v1 fallback `rss`/`cache`) — the container-TOTAL memory number used
    when per-process PSS is permission-denied for the whole tree. None
    when unreadable (absent, never zero)."""
    try:
        stat = (cg_dir / "memory.stat").read_text()
    except OSError:
        return None
    vals: Dict[str, int] = {}
    for line in stat.splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                vals[parts[0]] = int(parts[1])
            except ValueError:
                continue
    for a, b in (("anon", "file"), ("rss", "cache")):
        if a in vals and b in vals:
            return (vals[a] + vals[b]) // 1024
    return None


# ---------------------------------------------------------------------------
# Main sampling loop
# ---------------------------------------------------------------------------

def one_line(text: str) -> str:
    """Fold text onto ONE line so it cannot forge a sidecar record.

    The sidecar is parsed a line at a time: a `#` prefix makes it a
    comment, anything else is a data row, and `plot_usage.py` accepts a
    row on a 6-field comma split. So a line break inside a free-text
    field does not merely truncate provenance — it emits a SECOND line
    that is not a comment, and one carrying five commas whose numeric
    fields happen to parse is read as a measurement nobody sampled
    (Principle #13: never fabricate data).
    Four fields carry text this module does not choose: `--label` and
    `--docker-name` come from the caller, `comm` is whatever a process
    called itself, and the cgroup dir comes from /proc.

    The fold is keyed on `str.splitlines()` — the SAME function the
    reader splits on — rather than on a hand-listed set of characters,
    so "what counts as a line break here" cannot drift from "what counts
    as a line break there". (That set is wider than CR/LF: it also
    includes \v, \f, \x1c-\x1e, \x85, U+2028 and U+2029.) Backslashes are
    doubled first so an escape is never confused with literal text.
    Callers writing a DATA field additionally replace commas, the field
    separator; a comment line has no separator, so its commas are kept.

    NOT injective, deliberately: `splitlines()` DROPS a trailing
    separator, so "a\n" and "a" both record as `a`. What this function
    owes the reader is that nothing it writes can become a record;
    round-tripping a provenance string is not part of the contract, and
    buying it would mean re-deriving the break set by hand — the exact
    drift the shared key avoids.
    """
    return "\\n".join(text.replace("\\", "\\\\").splitlines())


def parse_pids_spec(spec: str) -> List[int]:
    """The ONE tokenizer for `--pids`. Raises ValueError with a message fit
    for `ap.error`.

    Three sites used to split this string: the validator, the emptiness
    check, and `run_sampler`. Two of them stripped each token and the third
    filtered on the RAW token, so a whitespace-only entry survived every
    guard and reached `int(" ")`:

        --pids "1, ,2"   validator: passes (" ".strip() is falsy, skipped)
                         emptiness: passes (same strip)
                         consumer : ValueError: invalid literal for int()
                                    with base 10: ' '

    -- a bare traceback deep inside run_sampler, which is the exact failure
    the validator's own comment says it exists to prevent. A validator that
    disagrees with its consumer about what a token IS cannot guard it, so
    the tokenizer is now one function and the disagreement has nowhere to
    live.

    Stripping is the behaviour that was kept, deliberately: `int()` strips
    too, so `--pids "1, 2"` has always worked and rejecting it now would
    break a working invocation to fix a crashing one.
    """
    out: List[int] = []
    for raw_tok in spec.split(","):
        tok = raw_tok.strip()
        if not tok:
            # Empty AND whitespace-only slots are both dropped -- "1,,2" and
            # "1, ,2" now mean the same thing to every caller.
            continue
        if not (tok.isascii() and tok.isdigit()):
            raise ValueError(f"--pids must be a comma-separated list of "
                             f"non-negative integers, got {tok!r}")
        out.append(int(tok))
    return out


def run_sampler(args: argparse.Namespace) -> int:
    clk_tck = os.sysconf("SC_CLK_TCK")
    ncpus = os.cpu_count() or 1
    disc = Discovery()

    # Resolve the sampling root.
    root_pid: Optional[int] = None
    static_pids: Optional[List[int]] = None
    if args.pids:
        static_pids = parse_pids_spec(args.pids)
        target_desc = f"pids={static_pids}"
    elif args.docker_name:
        deadline = time.monotonic() + args.docker_wait_s
        while time.monotonic() < deadline and not STOP:
            root_pid = docker_container_pid(args.docker_name)
            if root_pid is not None:
                break
            time.sleep(0.5)
        if root_pid is None:
            print(f"usage_sampler: container '{args.docker_name}' never "
                  f"appeared within {args.docker_wait_s}s — no sidecar "
                  f"written (nothing to sample is not a zero)",
                  file=sys.stderr)
            return 3
        target_desc = f"docker={args.docker_name} init_pid={root_pid}"
    else:
        root_pid = args.parent_pid
        target_desc = (f"parent_pid={root_pid} descend={bool(args.descend)}")

    self_pid = os.getpid()
    stat_period = 1.0 / args.stat_hz
    pss_every = max(1, round(args.stat_hz / args.pss_hz))

    # pid -> (starttime, cpu_ticks, ts_ns). Keyed on the process
    # INSTANCE: a recycled pid carries a new starttime, so its first
    # sighting starts a fresh baseline instead of inheriting the dead
    # process's ticks (finding: one garbage/negative cpu_pct row).
    prev: Dict[int, Tuple[int, int, int]] = {}
    pss_denied = 0
    cgroup_rows = 0
    cgroup_dir: Optional[Path] = None
    cgroup_marker_written = False
    rows = 0
    t_self0 = os.times()
    wall0 = time.monotonic()
    started_ns = time.monotonic_ns()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        f.write("# usage_sampler v1 scope=procs\n")
        f.write(f"# target: {one_line(target_desc)}\n")
        if args.label:
            f.write(f"# label: {one_line(args.label)}\n")
        f.write(f"# stat_hz={args.stat_hz} pss_hz={args.pss_hz} "
                f"clk_tck={clk_tck} ncpus={ncpus} "
                f"started_monotonic_ns={started_ns} "
                f"uname={os.uname().sysname}-{os.uname().machine}\n")
        f.write("# cpu_pct is % of ONE core (multithreaded procs exceed "
                "100); pss_kb sampled at pss_hz, empty elsewhere; "
                "empty != 0 (unreadable is counted, never fabricated)\n")
        f.write("ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb\n")

        tick = 0
        deadline = (time.monotonic() + args.duration_s
                    if args.duration_s else None)
        # Descendant discovery is CACHED and refreshed at --rescan-hz —
        # NOT per stat tick: without CONFIG_PROC_CHILDREN a rescan is a
        # full /proc walk, and doing it at 5 Hz measured 14.5% of a core
        # on a Jetson Orin (the observer must stay cheap enough not to
        # perturb what it observes). New children are picked up within
        # one rescan period; a cell's workers all spawn at bring-up, so
        # nothing long-lived is missed.
        rescan_period = 1.0 / args.rescan_hz if args.rescan_hz > 0 else None
        cached_pids: Set[int] = set()
        next_rescan = 0.0
        while not STOP:
            if deadline is not None and time.monotonic() >= deadline:
                break
            loop_start = time.monotonic()

            if static_pids is not None:
                pids = set(static_pids)
            elif args.descend or args.docker_name:
                if rescan_period is None or loop_start >= next_rescan:
                    cached_pids = disc.descendants(root_pid)
                    if rescan_period is not None:
                        next_rescan = loop_start + rescan_period
                pids = cached_pids
            else:
                pids = {root_pid}
            pids = set(pids)
            pids.discard(self_pid)

            # Root gone (and, for descend, nothing left) → the cell is
            # over; exit rather than sample an empty set forever.
            root_alive = (static_pids is not None
                          or Path(f"/proc/{root_pid}").exists())
            live_rows = 0
            do_pss = (tick % pss_every) == 0
            tick_pss_ok = 0
            tick_pss_denied = 0
            ts = time.monotonic_ns()
            for pid in sorted(pids):
                st = read_stat_cpu_ticks(pid)
                if st is None:
                    prev.pop(pid, None)
                    continue
                comm, ticks_now, starttime = st
                rss = read_rss_kb(pid)
                cpu_str = ""
                entry = prev.get(pid)
                if entry is not None and entry[0] == starttime:
                    _, ticks0, ts0 = entry
                    dt_s = (ts - ts0) / 1e9
                    if dt_s > 0:
                        # max(0, ·): belt-and-braces beside the starttime
                        # key — a clock/tick oddity must never mint a
                        # negative percentage a summing reader would eat.
                        cpu = max(0.0, (ticks_now - ticks0)
                                  / clk_tck / dt_s * 100.0)
                        cpu_str = f"{cpu:.1f}"
                # A recycled pid (starttime mismatch) drops through with
                # cpu_str empty — a first sighting of the NEW instance.
                prev[pid] = (starttime, ticks_now, ts)
                pss_str = ""
                if do_pss:
                    pss, denied = read_pss_kb(pid)
                    if denied:
                        pss_denied += 1
                        tick_pss_denied += 1
                    elif pss is not None:
                        pss_str = str(pss)
                        tick_pss_ok += 1
                # A data field: the comma is the separator (already
                # replaced), and one_line keeps the row on one line — a
                # process free to name itself is free to name itself with
                # a newline in it.
                comm_safe = one_line(comm).replace(",", "_")
                f.write(f"{ts},{pid},{comm_safe},{cpu_str},"
                        f"{'' if rss is None else rss},{pss_str}\n")
                rows += 1
                live_rows += 1
            # Docker whole-tree PSS denial → cgroup-total fallback: on a
            # PSS tick where EVERY read was denied (and none succeeded —
            # exactly the root-owned-container shape), record the
            # container's cgroup anon+file as ONE labeled scope=cgroup
            # row (pid=0, comm=cgroup, value in pss_kb). A container
            # TOTAL, not per-process PSS — plot_usage renders it as its
            # own series and never sums it with per-proc rows.
            if (args.docker_name and do_pss and tick_pss_denied > 0
                    and tick_pss_ok == 0):
                if cgroup_dir is None:
                    cgroup_dir = cgroup_mem_dir(root_pid)
                if cgroup_dir is not None:
                    cg_kb = read_cgroup_mem_kb(cgroup_dir)
                    if cg_kb is not None:
                        if not cgroup_marker_written:
                            cgroup_marker_written = True
                            f.write(f"# cgroup_fallback: scope=cgroup "
                                    f"dir={one_line(str(cgroup_dir))} "
                                    f"(whole-tree "
                                    f"smaps_rollup denial — container-"
                                    f"total anon+file, NOT per-process "
                                    f"PSS)\n")
                        f.write(f"{ts},0,cgroup,,,{cg_kb}\n")
                        cgroup_rows += 1
            f.flush()
            tick += 1
            if not root_alive and live_rows == 0:
                break
            if args.docker_name and not root_alive:
                break
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, stat_period - elapsed))

        # Footer: the sampler's own cost — the observer-cost record.
        t_self1 = os.times()
        wall = time.monotonic() - wall0
        self_cpu_s = ((t_self1.user - t_self0.user)
                      + (t_self1.system - t_self0.system))
        self_pct = (self_cpu_s / wall * 100.0) if wall > 0 else 0.0
        f.write(f"# sampler_self: cpu_s={self_cpu_s:.3f} wall_s={wall:.1f} "
                f"cpu_pct_of_one_core={self_pct:.2f} rows={rows} "
                f"ticks={tick} pss_denied={pss_denied} "
                f"cgroup_rows={cgroup_rows} "
                f"discovery={disc.mode or 'static'}\n")
    return 0


# ---------------------------------------------------------------------------
# Self-test (run on a Linux host; asserts against a spawned stress child)
# ---------------------------------------------------------------------------

_STRESS_CHILD = r"""
import time
blocks = []
end = time.monotonic() + 4.0
i = 0
while time.monotonic() < end:
    i += 1                      # spin: ~100% of one core
    if i % 2_000_000 == 0:
        blocks.append(bytearray(4_000_000))   # grow RSS ~4 MB steps
"""


def self_test() -> int:
    import tempfile
    child = subprocess.Popen([sys.executable, "-c", _STRESS_CHILD])
    out = Path(tempfile.mkstemp(suffix=".usage.csv")[1])
    try:
        rc = run_sampler(argparse.Namespace(
            out=str(out), parent_pid=child.pid, descend=False, pids=None,
            docker_name=None, docker_wait_s=0, stat_hz=5, pss_hz=1,
            rescan_hz=1.0, duration_s=3.5, label="self-test"))
        assert rc == 0, f"sampler rc={rc}"
    finally:
        child.wait(timeout=10)
    lines = [l for l in out.read_text().splitlines()
             if l and not l.startswith("#") and not l.startswith("ts_ns")]
    assert len(lines) >= 10, f"too few samples: {len(lines)}"
    cpus = [float(p[3]) for p in (l.split(",") for l in lines) if p[3]]
    rsss = [int(p[4]) for p in (l.split(",") for l in lines) if p[4]]
    psss = [int(p[5]) for p in (l.split(",") for l in lines) if p[5]]
    mean_cpu = sum(cpus) / len(cpus)
    ncpus = os.cpu_count() or 1
    # A spin loop should read near 100% of one core; a loaded box can
    # starve it, so the floor is deliberately modest — the pin is "the
    # sampler sees real CPU", not "the machine was idle".
    assert 25.0 < mean_cpu <= ncpus * 100 + 10, f"cpu mean {mean_cpu:.1f}"
    # RSS monotone sanity: the child only ever allocates; allow small
    # allocator jitter (5%) but require net growth.
    assert rsss, "no rss samples"
    assert min(rsss) >= rsss[0] * 0.95, "rss dipped >5% on a grow-only child"
    assert max(rsss) > rsss[0] + 2_000, \
        f"rss never grew: first={rsss[0]} max={max(rsss)}"
    assert psss, "no pss samples (same-user child should be readable)"
    footer = [l for l in out.read_text().splitlines()
              if l.startswith("# sampler_self:")]
    assert footer, "missing sampler_self footer"
    print(f"self-test PASS: {len(lines)} rows, cpu mean {mean_cpu:.1f}% "
          f"(1 core = 100), rss {rsss[0]}→{max(rsss)} kB, "
          f"pss samples {len(psss)}")
    print(footer[0])
    out.unlink(missing_ok=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    tgt = ap.add_mutually_exclusive_group()
    tgt.add_argument("--parent-pid", type=int,
                     help="root PID; with --descend, sample its whole tree "
                          "(the descendant set is cached and refreshed at "
                          "--rescan-hz)")
    tgt.add_argument("--pids", help="comma-separated explicit PID list")
    tgt.add_argument("--docker-name",
                     help="sample a docker container's processes from the "
                          "host (polls `docker inspect` for the init PID)")
    ap.add_argument("--descend", action="store_true",
                    help="with --parent-pid: sample all descendants")
    ap.add_argument("--out", help=".usage.csv sidecar path (required "
                                  "except under --self-test)")
    ap.add_argument("--stat-hz", type=float, default=5.0,
                    help="per-process /proc/stat sampling rate (> 0)")
    ap.add_argument("--pss-hz", type=float, default=1.0,
                    help="PSS sampling rate (> 0; smaps_rollup is much "
                         "costlier than stat, hence the separate rate)")
    ap.add_argument("--rescan-hz", type=float, default=1.0,
                    help="descendant-set refresh rate (>= 0). 0 disables "
                         "CACHING, i.e. rescans on EVERY stat tick — the "
                         "expensive path measured below, not a cheap one. "
                         "Kernels without "
                         "CONFIG_PROC_CHILDREN pay a full /proc scan per "
                         "rescan — measured on a Jetson Orin 5.15-tegra: "
                         "14.5%% of one core with per-tick rescans at 5 Hz, "
                         "4.9-5.5%% at the 1 Hz default)")
    ap.add_argument("--duration-s", type=float, default=None,
                    help="stop after this many seconds (> 0); omit to "
                         "sample until SIGTERM/SIGINT")
    ap.add_argument("--docker-wait-s", type=float, default=120.0,
                    help="how long to wait for --docker-name's container "
                         "to appear (>= 0; 0 polls zero times and reports "
                         "the container as absent)")
    ap.add_argument("--label", default=None,
                    help="free-text provenance recorded in the header")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    # Rate and duration validation, BEFORE any of it reaches arithmetic.
    # `--stat-hz 0` divided straight into `1.0 / args.stat_hz` and killed
    # the sampler with a ZeroDivisionError traceback. A negative rate did
    # NOT crash: the loop's `time.sleep(max(0.0, ...))` clamps the negative
    # period to zero, so the sampler BUSY-LOOPED on /proc at full speed —
    # an observer perturbing exactly what it observes, which is worse than
    # the traceback. NaN passed both. A traceback is not a refusal: it exits non-zero with no sidecar and no statement of what
    # was wrong, and the runner that spawned this sampler reports a missing
    # sidecar rather than a bad flag. argparse's own error path exits 2 and
    # names the option. Each bound is the one the arithmetic below actually
    # needs (a NaN fails every comparison, so `not (x > 0)` catches it):
    #   --stat-hz     1.0 / stat_hz            -> strictly positive
    #   --pss-hz      round(stat_hz / pss_hz)  -> strictly positive
    #   --rescan-hz   1.0 / rescan_hz, already guarded at 0 (which means
    #                 rescan EVERY tick, not never) -> non-negative
    #   --duration-s  a deadline; 0 or less is not a window
    #   --docker-wait-s  a bounded wait; 0 polls ZERO times and reports
    #                 the container as never having appeared
    # `math.isfinite` is not belt-and-braces: `inf > 0` is True, so +inf
    # passed every bound below, and each flag then failed differently —
    # `--stat-hz` raised OverflowError out of `round(inf / pss_hz)` during
    # setup, before the sidecar was even opened; `--pss-hz` and
    # `--rescan-hz` degraded SILENTLY to per-tick sampling; and
    # `--docker-wait-s` waited forever. One finite check, four failure
    # modes, not one of them a refusal. (NaN needs no extra clause: `nan >
    # 0` and `nan >= 0` are both False, so the comparison rejects it.)
    for name, value, allow_zero in (
            ("--stat-hz", args.stat_hz, False),
            ("--pss-hz", args.pss_hz, False),
            ("--rescan-hz", args.rescan_hz, True),
            ("--docker-wait-s", args.docker_wait_s, True)):
        if not math.isfinite(value) or not (
                value >= 0 if allow_zero else value > 0):
            ap.error(f"{name} must be a finite number "
                     f"{'>= 0' if allow_zero else '> 0'}, got {value!r}")
    # --pids is the other caller-supplied value the sampling root is built
    # from; `int(p)` on it raised a bare ValueError traceback deep in
    # run_sampler — no sidecar, nothing naming the flag. `isascii()` as
    # well as `isdigit()`: the latter is True for characters `int()`
    # refuses (superscripts, other digit-class code points), so the ASCII
    # clause is what makes this guard agree with the consumer.
    if args.pids is not None:
        try:
            parse_pids_spec(args.pids)
        except ValueError as e:
            ap.error(str(e))
    # Finiteness of each value is not finiteness of the quotient the loop
    # forms from them: `--pss-hz 1e-320` is a finite positive denormal that
    # clears every bound above, and `round(stat_hz / 1e-320)` overflows to
    # the same OverflowError. Check the derived value, not just the inputs.
    if not math.isfinite(args.stat_hz / args.pss_hz):
        ap.error(f"--stat-hz / --pss-hz overflows (got "
                 f"{args.stat_hz!r} / {args.pss_hz!r}) — the PSS tick "
                 f"divisor is derived from the pair, so the ratio has to be "
                 f"representable, not just each rate on its own")
    # An empty pid list is not "sample nothing": with no root pid the
    # liveness break can never fire, so the sampler loops writing zero rows
    # until the runner kills it.
    # Same tokenizer as the validator above and the consumer in run_sampler:
    # "is this spec empty" must mean the same thing to all three, or a spec
    # one of them treats as non-empty is not dropped by another.
    if args.pids is not None and not parse_pids_spec(args.pids):
        ap.error(f"--pids is empty (got {args.pids!r}) — there is nothing "
                 f"to sample, and an empty root set never terminates")
    if args.duration_s is not None and not math.isfinite(args.duration_s):
        ap.error(f"--duration-s must be a finite number, got "
                 f"{args.duration_s!r} (omit it to sample until signalled)")
    if args.duration_s is not None and not args.duration_s > 0:
        ap.error(f"--duration-s must be > 0 when given, got "
                 f"{args.duration_s!r} (omit it to sample until signalled; "
                 f"0 would otherwise read as 'no deadline' and a negative "
                 f"value would end the run before its first sample)")

    if not sys.platform.startswith("linux"):
        print("usage_sampler: /proc sampling is Linux-only — refusing "
              "(no sidecar is written; absent data is never fabricated)",
              file=sys.stderr)
        return 2
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    if args.self_test:
        return self_test()
    if not args.out:
        ap.error("--out is required")
    if not (args.parent_pid or args.pids or args.docker_name):
        ap.error("one of --parent-pid / --pids / --docker-name is required")
    return run_sampler(args)


if __name__ == "__main__":
    sys.exit(main())
