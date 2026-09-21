#!/usr/bin/env python3
"""Cerulion public latency-benchmark orchestrator (benches/latency/).

One self-contained suite, one tree. The old sibling-directory dispatch
(cerulion_round_trip vs cerulion_round_trip_quiescent) is replaced by a
single `--variant {quiescent, fixed100, backtoback}` knob that maps to the
`CER_BENCH_PACING` environment variable consumed by every bench process
(native binaries, the workspace runner, and the ROS 2 in-container driver).

Pacing variants (METHODOLOGY §1 + § "The rate axis"):

    quiescent   sensor-rate realism (per-size rates, 1 kHz → 10 Hz) —
                the default, and the fallback FLOOR of the fixed100 ladder
    fixed100    ONE uniform target rate, 100 Hz, at EVERY payload size
                (the field convention for a
                payload sweep — Part 2 of the shielding
                findings, archived internally).
                2100 total / 100 warmup per size (measured 2000 —
                tail-resolved). A cell that cannot SUSTAIN 100 Hz at a size
                steps DOWN the fallback ladder 100→50→20→<sensor rate,
                when below 20> and re-runs that size; the ACHIEVED rate
                is recorded in a `.rate` sidecar beside the `.bin`
                (compile-csv folds it into the achieved_rate_hz column;
                plot.py annotates any non-target point '@NHz' — no
                mixed-rate line is ever silent). A cell that exhausts the
                ladder renders 'did not sustain' (empty CSV row + loud
                note), never a fake latency.
    backtoback  saturation (uncapped) — the labeled secondary

Subcommands
-----------
    native       Build + run native host binaries (no docker)
    workspace    Run the real `cerulion graph run` workspace legs (split/mono)
    ros2         Build + run dockerized ROS 2 comparison cells
    compile-csv  Convert raw .bin samples to per-cell results_<prefix>.csv
    plots        Regenerate plot PNGs from CSVs (delegates to plot.py)
    smoke        Low-n smoke gate vs expected-ranges.yaml (machine-hash keyed)
    full         native + workspace + ros2 + compile-csv + plots
    list-cells   Print the full cell inventory (incl. '! skip' lines), run nothing

Line inventory (pinned; plot.py derives its expected
CSV set from the enumerations in THIS file, so the inventory lives in exactly
one place):

    Native host lines:
        iox2_chrt0                raw_iceoryx2_round_trip    (FLOOR, spin-bound)
        zenoh_shm_chrt{0,1}       zenoh_shm_round_trip(+_pong) (COMPARISON)
    (cerulion_user was RETIRED: manually-ticked
     nodes are not a usage pattern the public suite may showcase. The
     user-facing Cerulion lines are the workspace legs below.)

    Workspace legs (real `cerulion` CLI, driven via workspace/run_workspace.sh):
        cerulion_workspace_split_chrt{0,1}  graph run rtt_bench_split (declared
            2-group process_groups multi-process; THE headline row — it replaces
            the flagless `default` leg, which
            measured ~25.9 µs p50 in the historical retirement investigation
            because of the park-wake bug; this suite keeps the declared split
            as its headline until that inventory is deliberately revised)
        cerulion_workspace_mono_chrt{0,1}   graph run rtt_bench --single-process

    ROS 2 cells: {distro}_{rmw}_{shmmode}_{recv}_{qos}_chrt{N}
        distro ∈ {humble, jazzy, lyrical}
        rmw    ∈ {cyclonedds, fastdds, zenoh}
        shmmode: shm everywhere; zc (FastDDS DataSharing true zero-copy —
                 fastdds_zc.xml + RMW_FASTRTPS_USE_QOS_FROM_XML=1, the
                 rmw_fastrtps README recipe) is fastdds-ONLY with the same
                 distro coverage as shm; no_shm ONLY on jazzy
        recv   ∈ {rclcpp, loan}; loan is SHM-family-only (shm + zc);
                 zenoh×loan is a structural skip (NO-OP take-loan stub —
                 rmw_take_loaned_message returns RMW_RET_UNSUPPORTED;
                 rc=77)
        qos    ∈ {be1, rel10}; rel10 is SHM-family-only (shm + zc, never
                 no_shm), on jazzy × {cyclonedds, fastdds} × rclcpp plus
                 the fastdds zc lane (the original rmw pin was an SHM
                 head-to-head — no no_shm rel10 cells)
    (rmw_cerulion, ROS 2 over the Cerulion transport, is the separate
     `cerulion` rmw value: shm only. CERULION_RMWS below says why it sits
     beside the stock vendors and not among them.)

    ROS 2 usage-pattern lanes (ros2/memo.md —
    two pseudo-rmw lanes, never crossed with the matrix axes; both run
    qos=stock = rmw_qos_profile_default and recv=rclcpp by definition):
        {distro}_stock_rclcpp_chrt{N}               the ZERO-CONFIG default
            (memo §3: rmw_fastrtps_cpp, no XML, no transport env,
            DataSharing off, plain publish, 3 processes — what `ros2 run`
            gives you; the slowest-common lane). No verify_shm gate — the
            lane claims no transport label; a per-cell provenance note is
            written instead.
        {distro}_composed_ipc{on,off}_rclcpp_chrt{N}  the FASTEST-COMMON
            pattern (memo §2: composition — Nav2-default since Humble —
            with rclcpp intra-process comms as the opt-in shade): ONE
            process, three manually-composed nodes on one single-threaded
            executor (composed_rtt_node), CER_BENCH_IPC={on,off}.
        Distro coverage: jazzy is first-class (swept + verified); humble/
        lyrical are ENUMERATED but loudly unverified until a sweep runs
        them (cmd_ros2 prints the warning per cell).

Env-var contract (shared with native/src/lib.rs, workspace/run_workspace.sh
and ros2/run_bench.sh):

    CER_BENCH_RAW_DUMP_DIR / CER_BENCH_RAW_NAME   set per cell by this script
    CER_BENCH_PACING ∈ {quiescent, fixed100,     set from --variant
                        backtoback}
    CER_BENCH_SMOKE_N                             smoke-only (never export it)
    CER_BENCH_TARGET_SAMPLES / CER_BENCH_WARMUP   MEASURED samples + warmup
                                                  (G1 contract — see
                                                  samples_for; backtoback
                                                  defaults 10000/1000)
    CER_BENCH_TARGET_RATE_HZ                      ROS2-side per-payload rate
    CER_BENCH_QOS ∈ {be1, rel10}                  ROS2 side only
    CER_BENCH_PAYLOAD_SIZE(S)                     per-cell payload selection;
                                                  an ambient
                                                  CER_BENCH_PAYLOAD_SIZES
                                                  export RESTRICTS the
                                                  native/workspace/ros2
                                                  sweeps (partial re-runs —
                                                  only the restricted sizes'
                                                  .bins are cleared/gated,
                                                  so a restricted run merges
                                                  over a full sweep in place)
    CERULION_CPU_DMA_LOCK=1                       DMA latency lock (injected
                                                  only under the default
                                                  tuned posture)
    CER_BENCH_DMA_LOCK ∈ {1 (default), 0}         C-state POSTURE, honored
                                                  uniformly: 0 = STOCK (no
                                                  cap var, native bins skip
                                                  the lock, ros2 containers
                                                  get no --device) — see
                                                  dma_lock_enabled()
    CER_BENCH_WALL_STAMP=1                        workspace legs (wall stamps)
    CER_BENCH_USAGE ∈ {0 (default), 1}            per-cell CPU+memory usage
                                                  sidecars (<cell>_<size>
                                                  .usage.csv via
                                                  usage_sampler.py; Linux-
                                                  only host-side /proc
                                                  sampling — PSS + RSS both
                                                  recorded; native sweeps
                                                  split per-size under it;
                                                  plot with plot_usage.py)
    CER_BENCH_NATIVE_TIMEOUT_S                    native watchdog override
                                                  (default 1800 quiescent /
                                                  900 backtoback / 300 smoke)

Results land in results/<machine-hash>-<date>-<variant>/rep<k>/ (raw/ for
.bins + _logs/). The pacing variant is IN the run-dir key so cross-variant
same-day runs can never overwrite each other's raws, and the rep index k
(`--rep k` on native/workspace/ros2, default 1; `full --reps N` runs the
WHOLE matrix round-robin per rep) keys repeated sweeps so reps ACCUMULATE
instead of overwriting — the cross-rep summary CSVs (compile-csv) and
plots/ land at the variant-dir top, beside a merge-written run.json
provenance manifest (git sha + dirty flag, machine hash, uname, cpu
governor + turbo/boost state, per-cell start/end timestamps, the skip
inventory, docker image ids + their embedded repo-sha labels). compile-csv
and plots stay backward-readable for legacy rep-less run dirs (raw/
directly under the run dir; pass --run-dir explicitly for those). The
machine hash comes from
scripts/benchmarks/lib/machine_hash.sh::compute_live_machine_hash — the
repo-level single source of truth; never reimplemented here.

No number is ever fabricated by this script (Principle #13: no fake data):
results/ ships empty, expected-ranges.yaml ships `hosts: {}`, and baselines
are written only by `smoke --capture-baseline` from real measured p50s.
"""

import argparse
import json
import os
import platform
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# ============================================================================
# Platform detection
# ============================================================================

IS_LINUX = sys.platform.startswith("linux")
IS_MACOS = sys.platform == "darwin"
IS_WINDOWS = sys.platform == "win32"
PLATFORM_LABEL = ("linux" if IS_LINUX else "macos" if IS_MACOS
                  else "windows" if IS_WINDOWS else sys.platform)

# ============================================================================
# Paths (single tree — no sibling-dir variant dispatch)
# ============================================================================

LATENCY_DIR = Path(__file__).resolve().parent          # benches/latency
BENCHES_DIR = LATENCY_DIR.parent                       # benches
REPO_ROOT = BENCHES_DIR.parent                         # repo root
NATIVE_DIR = LATENCY_DIR / "native"
WORKSPACE_DIR = LATENCY_DIR / "workspace"
ROS2_DIR = LATENCY_DIR / "ros2"
RESULTS_ROOT = LATENCY_DIR / "results"
RANGES_PATH = LATENCY_DIR / "expected-ranges.yaml"
NATIVE_BIN_DIR = NATIVE_DIR / "target" / "release"
MACHINE_HASH_LIB = REPO_ROOT / "tools" / "scripts" / "benchmarks" / "lib" / "machine_hash.sh"

IMAGE_PREFIX = "latency_bench"     # docker image: latency_bench:{distro}

def image_tag(distro: str) -> str:
    return f"{IMAGE_PREFIX}:{distro}"

# ============================================================================
# Pacing variants (--variant → CER_BENCH_PACING; same tree, two modes)
# ============================================================================

PACING_VARIANTS = ("quiescent", "fixed100", "backtoback")
PACING_ENV = "CER_BENCH_PACING"

# The ONE host-tuning toggle (machine posture), honored
# UNIFORMLY across all three stacks — it mirrors and extends the workspace
# runner's existing knob of the same name:
#   1 (default)  TUNED-side C-state posture: workspace legs get
#                CERULION_CPU_DMA_LOCK=1 injected (hard C0 pin), native
#                bins acquire the C-state lock (lib.rs::acquire_dma_lock),
#                ros2 containers get /dev/cpu_dma_latency passed through
#                (--device) so the in-container bins can hold the cap.
#   0            STOCK posture — no bench-side machine tuning: workspace
#                legs run the product's flagless default (the var is
#                omitted entirely, so `cerulion graph run` resolves its
#                own Auto posture), native bins SKIP the lock (they read
#                this same inherited var), and ros2 containers run
#                WITHOUT the device (the in-container bins print their
#                loud no-device note and proceed uncapped).
DMA_LOCK_ENV = "CER_BENCH_DMA_LOCK"

# Backtoback (saturation) counts, mirrored by the native
# binaries' own env defaults in native/src/lib.rs. Per the G1 contract,
# TARGET_SAMPLES is the MEASURED count (warmup rides on top: 11000 total).
BACKTOBACK_TARGET_SAMPLES = 10000
BACKTOBACK_WARMUP = 1000

# fixed100 (uniform-rate) constants. ONE
# target rate at EVERY payload size; counts uniform (measured 2000 —
# tail-resolved, and rate-INDEPENDENT so the exact-count CSV gate holds at
# every ladder rung). Lockstep in four places: native/src/lib.rs::
# FIXED100_{RATE_HZ,TOTAL,WARMUP}, ros2/run_bench.sh's fixed100 arm,
# workspace/run_workspace.sh::FIXED100_*, and here.
FIXED100_RATE_HZ = 100
FIXED100_TOTAL = 2100
FIXED100_WARMUP = 100
# Fallback-rung retry budget (the target rung keeps the normal 3-attempt
# transient-flake budget; lower rungs get fewer — a genuinely
# unsustainable rate fails every attempt identically, so extra retries
# only burn watchdog windows).
FIXED100_FALLBACK_ATTEMPTS = 2

def check_ambient_pacing(variant: str) -> None:
    """Refuse a run whose ambient CER_BENCH_PACING contradicts --variant.

    bench.py owns the pacing env for every process it spawns; a conflicting
    export means the operator believes a different mode is active than the
    one the spawned processes would be told to run. Loud > silent."""
    ambient = os.environ.get(PACING_ENV)
    if ambient is not None and ambient != variant:
        raise SystemExit(
            f"{PACING_ENV}={ambient!r} is exported but --variant {variant} was "
            f"requested — refusing to run with a contradictory pacing env. "
            f"Either `unset {PACING_ENV}` or pass --variant {ambient}.")

# The posture recorded when TUNED was requested on a host that cannot
# deliver it (see require_dma_posture_deliverable). Deliberately NOT
# 'tuned' and NOT 'stock': the run asked for the cap and did not get it,
# which is neither, and plot.py's posture gate refuses any value that is
# not the one a hero figure claims — so such a run dir cannot render a
# posture-titled figure at all.
UNCAPPED_POSTURE = "tuned-uncapped"

# Mirrors compile_csv.py::REP_DIR_RE. The refusal note counts what THAT
# reader would find, so the two must agree on what a rep directory is.
REFUSAL_REP_DIR_RE = re.compile(r"^rep(\d+)$")

# The parsed args of the current invocation, parked by main() so the posture
# refusal can name what an explicitly-targeted --run-dir already holds. A
# module slot rather than a fifth parameter on a gate that is called once
# per invocation from five places; empty in unit drives, which is why every
# read goes through getattr with a default.
_REFUSAL_ARGS = argparse.Namespace()

def dma_lock_enabled() -> bool:
    """Resolve CER_BENCH_DMA_LOCK (see DMA_LOCK_ENV above). Loud on a bad
    value or a contradictory CERULION_CPU_DMA_LOCK export — a silently
    misread posture is a mislabeled-figure hazard, the same class as a
    mixed pacing variant."""
    v = os.environ.get(DMA_LOCK_ENV, "1")
    if v == "1":
        return True
    if v == "0":
        if os.environ.get("CERULION_CPU_DMA_LOCK") is not None:
            raise SystemExit(
                f"{DMA_LOCK_ENV}=0 (stock posture) but CERULION_CPU_DMA_LOCK "
                f"is also exported — contradictory C-state posture; unset "
                f"one. A stock run must not inherit the cap var.")
        return False
    raise SystemExit(f"{DMA_LOCK_ENV} must be 0 or 1 (got {v!r})")

DMA_DEVICE = Path("/dev/cpu_dma_latency")

# Which principal has to be able to take the lock for THIS invocation.
# "host": at least one leg measures on the host as this user (native,
# workspace, and therefore full and smoke). "container": every measured
# cell runs inside a docker container, which this script starts with no
# --user, so it runs as the image's default (root) and can open a device
# this process cannot. Set by each data-producing subcommand beside its
# posture gate.
#
# `None` is UNDECLARED, and it is NOT the same as "host": `dma_basis()`
# reads an undeclared slot as the stricter "host", so a subcommand that
# forgets to declare cannot get the weaker check — but a DECLARED host
# basis additionally LOCKS OUT a later "container", which an undeclared
# default could not do.
#
# That lock is the whole point. `cmd_full` declares "host" and then
# DISPATCHES INTO cmd_ros2, which declared "container" and overwrote it —
# after the native and workspace legs had already measured uncapped on a
# host that cannot take the lock. The ROS 2 cells then got `--device` and
# really were capped, so ONE run directory carried capped comparison rows
# beside uncapped Cerulion rows under a single label describing neither
# half: the exact mix the per-principal basis was introduced to prevent,
# reintroduced by the dispatch. The basis belongs to the INVOCATION, not
# to whichever subcommand set it last, and the only safe direction of
# travel is toward the stricter principal.
_DMA_BASIS: Optional[str] = None

def set_dma_basis(basis: str) -> None:
    assert basis in ("host", "container"), basis
    global _DMA_BASIS
    if _DMA_BASIS == "host" and basis == "container":
        # An enclosing sweep already declared that some leg measures on
        # the host. A nested lane cannot relax that for the whole run.
        #
        # LOUD, not a bare return: this is an inference boundary, and a
        # lane measuring under a basis it did not declare is exactly the
        # confusion the ratchet exists to end. Silent, the override leaves
        # no trace anywhere — not stderr, not run.json — and on a host
        # where the device IS writable it changes nothing observable, so
        # the only evidence it happened would be the refusal path, by
        # accident.
        print(f"! C-state basis: this lane declares 'container', but an "
              f"enclosing sweep already declared 'host' for this "
              f"invocation — keeping 'host', the stricter principal. ONE "
              f"basis per run directory: its host-side legs cannot take "
              f"the lock, so its ROS 2 cells must not either.",
              file=sys.stderr)
        return
    _DMA_BASIS = basis

def dma_basis() -> str:
    """The declared basis, defaulting to the stricter principal."""
    return _DMA_BASIS or "host"

# Resolved ONCE per INVOCATION — main() clears it alongside _DMA_BASIS,
# since a probe memoised under one basis must not outlive it. Within an
# invocation every consumer — the refusal, the manifest label, and the
# per-cell --device flag — must read the same answer, or the flag and the
# label can disagree across time with nobody told. The reason is kept with
# the verdict so the refusal reports what actually failed instead of
# re-deriving a guess.
_DMA_PROBE: Optional[Tuple[bool, str]] = None

def _probe_dma_device() -> Tuple[bool, str]:
    if not IS_LINUX:
        return (False, "not linux")
    try:
        fd = os.open(str(DMA_DEVICE), os.O_WRONLY)
    except FileNotFoundError:
        return (False, "absent")
    except PermissionError:
        return (False, "unwritable")
    except OSError as e:
        # NOT a posture fact. EMFILE/ENFILE under a long sweep, ENOMEM, a
        # device busy — treating any of these as "unwritable" would demote
        # a genuinely tuned run's label (plot.py then refuses its hero
        # figures) and would send the operator to `chmod` for an
        # fd-exhaustion problem. Refuse rather than label on a guess.
        raise SystemExit(
            f"cannot determine the C-state posture: opening {DMA_DEVICE} "
            f"failed with {e}. That is not a permission fact, so the run "
            f"would be labelled on a guess — fix the condition and re-run.")
    os.close(fd)
    return (True, "ok")

def dma_probe() -> Tuple[bool, str]:
    global _DMA_PROBE
    if _DMA_PROBE is None:
        _DMA_PROBE = _probe_dma_device()
    return _DMA_PROBE

def dma_device_present() -> bool:
    """Is the C-state device there at all? Only meaningful on Linux.

    PRESENCE is not the posture — see `dma_device_acquirable`. It is kept
    apart because the two failures have DIFFERENT remedies (get a host
    with the device vs fix its permissions), and a message that cannot
    tell them apart sends the operator to the wrong one."""
    return IS_LINUX and DMA_DEVICE.exists()

def dma_device_acquirable() -> bool:
    """Can the C-state lock actually BE TAKEN by the principal that measures?

    Existence was the wrong question. `cerulion_core::dma_lock::cpu_dma_lock`
    OPENS the device for writing; a present-but-unwritable
    /dev/cpu_dma_latency (the common non-root shape — mode 0600 root:root)
    makes that open fail, `acquire_dma_lock` then returns None under
    CER_BENCH_ALLOW_NO_DMA_LOCK=1, and the samples are uncapped while a
    posture derived from existence alone still said 'tuned'. Principle #2:
    the label must follow what the run DID, not what the filesystem hints.

    Asked the way the lock asks it — the same open(2) — because a
    permission-bit inspection is a different question: `os.access` uses the
    REAL uid and reports success for root on files root cannot actually
    write, and it cannot see a SELinux/AppArmor denial or an exclusive
    holder. Opening WITHOUT writing is side-effect-free: the kernel's pm_qos
    device registers a request at PM_QOS_DEFAULT_VALUE (no constraint) on
    open and drops it on close; the cap is only asserted by the write the
    real lock does and this probe deliberately does not."""
    if not IS_LINUX:
        return False
    if dma_basis() == "container":
        # The cells run as root inside a container we hand the device to,
        # so THIS process's inability to open it says nothing about
        # theirs. Presence is the whole precondition on that basis.
        return DMA_DEVICE.exists()
    return dma_probe()[0]

def dma_lock_uncapped_allowed() -> bool:
    """`CER_BENCH_ALLOW_NO_DMA_LOCK=1` — the EXISTING escape hatch for
    'the lock was asked for and could not be had'. native/src/lib.rs has
    honoured it for a failed acquisition since the knob landed; a missing
    device is the same condition one step earlier, so it is the same
    hatch rather than a new one."""
    return os.environ.get("CER_BENCH_ALLOW_NO_DMA_LOCK") == "1"

def require_dma_posture_deliverable(scope: str) -> None:
    """Refuse a TUNED run this host cannot deliver.

    The native bins already refuse: `acquire_dma_lock` panics when
    `cpu_dma_lock()` fails unless the hatch is set. The ROS 2 lane did
    not — bench.py simply omitted `--device`, every container ran
    uncapped, and the manifest still recorded `dma_lock_posture: tuned`.
    Those rows are then indistinguishable from capped ones to plot.py's
    posture gate, to the smoke baselines, and to anyone reading the CSV:
    uncapped measurements consumable as capped tuned data, which is the
    exact mislabel the knob exists to prevent.

    So: refuse, naming the hatch. With the hatch set the run proceeds and
    the posture LABEL becomes accurate (see `resolved_dma_posture`) rather
    than the intent."""
    if not IS_LINUX:
        # No cell on this platform consumes the device; the posture is
        # decided per-lane by the runners themselves.
        return
    if not dma_lock_enabled() or dma_device_acquirable():
        return
    # Two distinct conditions with two distinct remedies. Reported apart
    # because "no such device" and "there but you cannot open it" send the
    # operator to completely different fixes, and the second is the common
    # one (mode 0600 root:root on a stock install).
    # The reason comes from the probe that actually failed, so the message
    # cannot describe a condition other than the one observed.
    reason = "absent" if dma_basis() == "container" else dma_probe()[1]
    if reason == "unwritable":
        why = ("/dev/cpu_dma_latency EXISTS but cannot be opened for "
               "writing by this user, so the lock cannot be taken")
        fix = ("make it writable — `sudo chmod 666 /dev/cpu_dma_latency` "
               "(until reboot) or a udev rule `KERNEL==\"cpu_dma_latency\", "
               "MODE=\"0666\"`; check `ls -l /dev/cpu_dma_latency`")
    else:
        why = "this host has no /dev/cpu_dma_latency at all"
        fix = ("run on a host with the device — it comes from the kernel's "
               "pm_qos support, there is no module to load")
    if dma_lock_uncapped_allowed():
        print(f"! {DMA_LOCK_ENV}=1 (tuned) but {why} — {scope} run "
              f"UNCAPPED. CER_BENCH_ALLOW_NO_DMA_LOCK=1 is set, so the run "
              f"proceeds and is labelled '{UNCAPPED_POSTURE}': p99/p99.9 "
              f"tails are NOT citable, and posture-titled figures will "
              f"refuse this run dir.", file=sys.stderr)
        return
    raise SystemExit(
        f"{DMA_LOCK_ENV}=1 requests the TUNED posture but {why}: the native "
        f"bins would refuse and the ROS 2 containers would run uncapped "
        f"while the manifest recorded 'tuned'. Refusing to produce "
        f"measurements that would be consumable as capped data."
        f"{_refusal_leftovers_note()}\n"
        f"fix: {fix}; or sweep the deliverable posture with {DMA_LOCK_ENV}=0 "
        f"(stock); or set CER_BENCH_ALLOW_NO_DMA_LOCK=1 to record anyway — "
        f"labelled '{UNCAPPED_POSTURE}', tails not citable.")

def _refusal_leftovers_note() -> str:
    """Name what an EXPLICITLY TARGETED run dir already holds, so a refusal
    cannot be read as "this invocation produced what is in there".

    The refusal fires before any directory is resolved, created or written,
    so this invocation contributes nothing — but when `--run-dir` names an
    existing directory those rows are an EARLIER invocation's, and a later
    compile-csv globs the directory rather than asking who wrote it.

    Deliberately NOT a deletion. Clearing on refusal would destroy a
    completed campaign because a knob was mistyped — strictly worse than
    the mislabel it prevents, and the operator can always re-sweep or
    delete. Only an EXPLICIT --run-dir is inspected: the default path mints
    a per-(machine, date, variant) directory, and reporting its contents
    would just describe today's own earlier legs.

    BOUNDED to the layout this harness writes, never a recursive walk.
    `--run-dir` is an arbitrary operator-supplied path with no constraint
    that it be a results directory, so a typo — `--run-dir /tmp`, or a home
    directory — would have made a PREFLIGHT REFUSAL traverse an arbitrarily
    large tree before printing its message. The layout is known and shallow
    (`raw_dir_of(rep_dir_of(...))`: `<run>/rep<k>/raw/`, plus the legacy
    rep-less `<run>/raw/`), so one `iterdir` of the run directory plus one
    `listdir` per raw directory answers the question, and a wrong
    `--run-dir` costs a single readdir instead of a filesystem crawl.

    Best-effort by construction: an unreadable directory yields no note
    rather than a second failure on top of the refusal being reported."""
    raw = getattr(_REFUSAL_ARGS, "run_dir", None)
    if not raw:
        return ""
    try:
        d = Path(raw)
        if not d.is_dir():
            return ""
        # The layout is `<run>/rep<k>/raw/*.bin`, with a legacy rep-less
        # `<run>/raw/*.bin` — THREE levels and two levels, matching
        # compile_csv's own `discover_rep_raw_dirs`, which is the reader
        # this note exists to warn about. A shallower guess would have
        # counted zero on every real run directory and stayed silent
        # exactly when the warning is due.
        #
        # Each directory is listed INDIVIDUALLY rather than globbed as one
        # pattern: glob never raises on a permission error, it just yields
        # nothing, which is indistinguishable from "empty" — so an
        # unreadable raw dir has to be caught where it is opened.
        blind = False
        def _count_bins(p: Path) -> int:
            nonlocal blind
            try:
                return sum(1 for f in os.listdir(p) if f.endswith(".bin"))
            except OSError:
                blind = True
                return 0
        bins = 0
        raw_dirs = []
        try:
            for child in sorted(d.iterdir()):
                if child.is_dir() and REFUSAL_REP_DIR_RE.match(child.name) \
                        and (child / "raw").is_dir():
                    raw_dirs.append(child / "raw")
        except OSError:
            blind = True
        # `Path.is_dir()` does NOT swallow every OSError — EACCES
        # propagates — so probing inside an unreadable run directory
        # raises here, and the outer handler below would return "" with
        # `blind` discarded. That is how the UNKNOWN branch stayed
        # unreachable: the one input it exists for never got there.
        legacy = d / "raw"
        try:
            if legacy.is_dir():
                raw_dirs.append(legacy)
        except OSError:
            blind = True
        for r in raw_dirs:
            bins += _count_bins(r)
    except OSError:
        # Only `Path(raw)` and the first `is_dir()` remain under this: if
        # the path cannot even be established as a directory, there is
        # nothing to report about its contents.
        return ""
    if blind and not bins:
        # Counted nothing AND could not read something. "at least 0" would
        # assert that leftovers exist on no evidence at all — the mirror of
        # the confident-wrong-count this note was rewritten to avoid. Say
        # what is true: unknown.
        return (f"\nNOTE: this invocation produced NOTHING, and "
                f"{_short(d)} could not be listed — whether it already "
                f"holds .bin files from an EARLIER invocation is UNKNOWN. "
                f"Check it before compiling anything as this run's.")
    if not bins:
        return ""
    count = f"at least {bins}" if blind else str(bins)
    return (f"\nNOTE: this invocation produced NOTHING — but {_short(d)} "
            f"already holds {count} .bin file(s) from an EARLIER "
            f"invocation (its own run.json records THAT run's posture). "
            f"They were left untouched; do not compile them as this run's.")

def resolved_dma_posture() -> str:
    """The posture that was actually DELIVERED, for every artifact that
    records one. `dma_lock_enabled()` is the REQUEST; on a host with no
    device an honoured request is impossible, so a run that proceeded
    under the hatch is labelled `UNCAPPED_POSTURE` and never 'tuned'.
    plot.py's hero gate compares this string for equality, so the third
    value makes posture-titled figures REFUSE such a run dir — which is
    the correct outcome for data whose tails are not citable."""
    if not dma_lock_enabled():
        return "stock"
    if IS_LINUX and not dma_device_acquirable():
        return UNCAPPED_POSTURE
    return "tuned"

def announce_posture(scope: str) -> None:
    """Once per data-producing invocation: loud stock-posture banner.
    (Tuned is the default and stays quiet — the manifest records the
    posture either way.)"""
    if not dma_lock_enabled():
        print(f"! STOCK POSTURE ({DMA_LOCK_ENV}=0) — {scope}: no bench-side "
              f"C-state tuning is applied (workspace legs run the product "
              f"flagless default; native bins skip the C-state lock; ros2 "
              f"containers get NO /dev/cpu_dma_latency device). Tails are "
              f"NOT comparable to capped runs.", file=sys.stderr)

# ============================================================================
# Per-cell CPU + memory usage sidecars (CER_BENCH_USAGE=1 — default OFF)
# ============================================================================

# CER_BENCH_USAGE=1 records a `<cell>_<size>.usage.csv` sidecar beside every
# `.bin` (ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb — usage_sampler.py's format;
# PSS is the accurate cross-process number for SHM-heavy cells, RSS rides
# along labeled). Default OFF: an unset/0 value runs the suite byte-
# identically to a pre-usage tree (zero sampler processes, zero overhead).
# Sampling is host-side /proc (Linux-only): native + workspace cells are
# sampled by descending the spawned process tree; ros2 cells are sampled
# from the HOST via the container's init PID (docker inspect). Under
# usage mode the NATIVE sweep is split into one invocation per payload
# size (the binaries otherwise sweep sizes internally, which would make
# per-size attribution impossible from outside); the .bin contract and
# gates are unchanged. Plot with plot_usage.py.
USAGE_ENV = "CER_BENCH_USAGE"
USAGE_SAMPLER = LATENCY_DIR / "usage_sampler.py"
_WARNED_USAGE_NON_LINUX = False

def usage_enabled() -> bool:
    """Strict CER_BENCH_USAGE parse: unset/'0' = off, '1' = on, anything
    else is a loud refusal (a silently-misread knob is the mislabeled-
    figure class)."""
    v = os.environ.get(USAGE_ENV)
    if v is None or v == "0":
        return False
    if v == "1":
        if not IS_LINUX:
            global _WARNED_USAGE_NON_LINUX
            if not _WARNED_USAGE_NON_LINUX:
                _WARNED_USAGE_NON_LINUX = True
                print(f"! {USAGE_ENV}=1 but usage sampling is /proc-based "
                      f"(Linux-only) — NO .usage.csv sidecars will be "
                      f"written on {PLATFORM_LABEL} (absent, never "
                      f"fabricated)", file=sys.stderr)
            return False
        return True
    raise SystemExit(f"{USAGE_ENV} must be 0 or 1 (got {v!r})")

def usage_sidecar_path(raw_dir: Path, raw_name: str, payload: int) -> Path:
    return raw_dir / f"{raw_name}_{payload}.usage.csv"

def start_usage_sampler(out_path: Path, log_dir: Path, *,
                        parent_pid: Optional[int] = None,
                        docker_name: Optional[str] = None
                        ) -> Optional[subprocess.Popen]:
    """Spawn usage_sampler.py for one cell invocation (already gated on
    usage_enabled() by the callers). Sampler stderr goes to a per-cell
    _logs/<sidecar>.sampler.log so a sampler failure is auditable without
    polluting the cell's own log. Returns None only if the spawn itself
    fails (loud — the cell still runs; a missing sidecar is visible)."""
    out_path.unlink(missing_ok=True)   # stale-sidecar guard (same rule as .rate)
    log_dir.mkdir(parents=True, exist_ok=True)
    slog = log_dir / f"{out_path.name}.sampler.log"
    cmd = [sys.executable, str(USAGE_SAMPLER), "--out", str(out_path)]
    if docker_name is not None:
        cmd += ["--docker-name", docker_name]
    else:
        cmd += ["--parent-pid", str(parent_pid), "--descend"]
    try:
        return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=slog.open("wb"))
    except OSError as e:
        print(f"    ! usage sampler failed to spawn ({e}) — cell runs "
              f"UNSAMPLED (no sidecar)", file=sys.stderr)
        return None

def stop_usage_sampler(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    if proc.poll() is None:
        proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)

# ============================================================================
# Payload sweep + schedules
# ============================================================================

# Pinned 10-size sweep (settles the May 9-vs-10 discrepancy).
PAYLOAD_SIZES = (64, 256, 1024, 4096, 16384, 65536, 262144, 1048576,
                 4194304, 16777216)

def ambient_payload_restriction() -> Optional[Tuple[int, ...]]:
    """The ambient CER_BENCH_PAYLOAD_SIZES restriction, validated, or None.

    The suite-wide partial-re-sweep mechanism (METHODOLOGY §11): exporting
    CER_BENCH_PAYLOAD_SIZES="4194304 16777216" restricts every subcommand's
    sweep to those sizes — the native binaries honor it themselves
    (native/src/lib.rs::sweep_payload_sizes), the workspace runner reads it
    directly, and cmd_ros2 enumerates only the restricted payloads. The
    per-prefix stale-mix guards clear ONLY the restricted sizes, so a
    restricted run MERGES over a prior full sweep in place. Values must be
    a non-empty subset of the pinned sweep (the CSV/plot row contracts key
    on PAYLOAD_SIZES) — anything else refuses loudly."""
    raw = os.environ.get("CER_BENCH_PAYLOAD_SIZES")
    if raw is None:
        return None
    parts = [p for p in re.split(r"[,\s]+", raw.strip()) if p]
    try:
        sizes = tuple(int(p) for p in parts)
    except ValueError:
        raise SystemExit(
            f"CER_BENCH_PAYLOAD_SIZES={raw!r} must be whitespace/comma-"
            f"separated byte counts") from None
    bad = [s for s in sizes if s not in PAYLOAD_SIZES]
    if bad or not sizes:
        raise SystemExit(
            f"CER_BENCH_PAYLOAD_SIZES={raw!r}: "
            f"{'empty restriction' if not sizes else 'unknown size(s) ' + ', '.join(map(str, bad))}"
            f" — values must be a non-empty subset of the pinned sweep: "
            f"{', '.join(map(str, PAYLOAD_SIZES))}")
    return sizes

def quiescent_schedule(payload: int) -> Tuple[int, int, int]:
    """Return (rate_hz, total_iterations, warmup) for a payload size.

    The tuple is (rate, TOTAL, warmup) — total INCLUDES the warmup, so the
    measured-sample count is total − warmup (derived by samples_for; the G1
    contract is that CER_BENCH_TARGET_SAMPLES always means MEASURED samples).

    MUST stay in lockstep in FOUR places (G2):
      - native/src/lib.rs::quiescent_schedule (Rust)
      - ros2/run_bench.sh's schedule case statement (bash)
      - bench.py::quiescent_schedule (this function)
      - workspace/run_workspace.sh::schedule_for (bash)
    All four must produce identical per-payload counts so cross-stack CSV
    rows are apples-to-apples comparable.

    Smoke-gate override: when CER_BENCH_SMOKE_N is set, (total, warmup) is
    replaced with (N, max(N // 10, 1)) while the rate stays on the schedule.
    THREE of the four mirrors implement it — this one, native/src/lib.rs's
    smoke_override(), and run_workspace.sh's SMOKE_N block. ros2/run_bench.sh
    deliberately does NOT: its cells always receive explicit measured/warmup
    counts as `docker -e` flags, derived by samples_for() from THIS function,
    so the ROS 2 containers inherit the override without reading the knob.
    (A direct `bash run_bench.sh` with CER_BENCH_SMOKE_N exported therefore
    runs the FULL schedule — the variable is a bench.py-orchestrated knob.)
    """
    # Tail-resolved tops: 4 MiB and 16 MiB run
    # longer windows (~70 s / ~205 s) so measured n >= 2000 and a
    # single-rep p99 is citable at EVERY size (>= 20 tail exceedances).
    if payload in (64, 256, 1024):       sched = (1000, 60000, 5000)
    elif payload in (4096, 16384):       sched = (500, 30000, 2500)
    elif payload == 65536:               sched = (200, 12000, 1000)
    elif payload == 262144:              sched = (100, 6000, 500)
    elif payload == 1048576:             sched = (60, 3600, 300)
    elif payload == 4194304:             sched = (30, 2100, 100)
    elif payload == 16777216:            sched = (10, 2050, 50)
    else:                                sched = (10, 600, 50)
    smoke_n = os.environ.get("CER_BENCH_SMOKE_N")
    if smoke_n is not None:
        try:
            n = int(smoke_n)
        except ValueError:
            raise SystemExit(
                f"CER_BENCH_SMOKE_N must be an integer > 2, got {smoke_n!r}") from None
        if n <= 2:
            raise SystemExit(
                "CER_BENCH_SMOKE_N must be > 2 (n=2 leaves only 1 measurement "
                "sample after warmup; read_p50_ns requires >= 2 samples — "
                "minimum safe value is 3)")
        return (sched[0], n, max(n // 10, 1))
    return sched

def _smoke_total_warmup() -> Optional[Tuple[int, int]]:
    """The CER_BENCH_SMOKE_N (total, warmup) override, validated, or None
    — shared by the quiescent and fixed100 schedules (same rule as the
    Rust smoke_override): (N, max(N // 10, 1))."""
    smoke_n = os.environ.get("CER_BENCH_SMOKE_N")
    if smoke_n is None:
        return None
    try:
        n = int(smoke_n)
    except ValueError:
        raise SystemExit(
            f"CER_BENCH_SMOKE_N must be an integer > 2, got {smoke_n!r}") from None
    if n <= 2:
        raise SystemExit(
            "CER_BENCH_SMOKE_N must be > 2 (n=2 leaves only 1 measurement "
            "sample after warmup; read_p50_ns requires >= 2 samples — "
            "minimum safe value is 3)")
    return n, max(n // 10, 1)

def fixed100_schedule(payload: int) -> Tuple[int, int, int]:
    """(rate_hz, total, warmup) for the fixed100 variant: the UNIFORM
    (100, 2100, 100) tuple at every payload size (measured 2000,
    tail-resolved; see METHODOLOGY § "The rate axis"). Same four-place lockstep + CER_BENCH_SMOKE_N override rule
    as quiescent_schedule. The payload argument exists for signature
    parity (the tuple is deliberately payload-independent — that IS the
    variant)."""
    _ = payload
    total, warmup = _smoke_total_warmup() or (FIXED100_TOTAL, FIXED100_WARMUP)
    return (FIXED100_RATE_HZ, total, warmup)

def fixed100_ladder(payload: int) -> Tuple[int, ...]:
    """The fixed100 fallback ladder for one payload size: the 100 Hz
    target, then 50, then 20, then the size's SENSOR rate (its quiescent
    schedule rate) when that sits BELOW the 20 Hz rung — strictly
    descending (today only 16 MiB @ 10 Hz gains a 4th rung). A cell that
    cannot sustain a rung is RE-RUN at the next one; counts stay the
    uniform fixed100 tuple at every rung. Lockstep with
    native/src/lib.rs::fixed100_ladder +
    workspace/run_workspace.sh::ladder_for."""
    rungs = [FIXED100_RATE_HZ, 50, 20]
    sensor_rate = quiescent_schedule(payload)[0]
    if sensor_rate < rungs[-1]:
        rungs.append(sensor_rate)
    return tuple(rungs)

# ----------------------------------------------------------------------------
# fixed100 achieved-rate sidecars: <raw_name>_<size>.rate beside the .bin.
# First line = the achieved integer Hz, or the literal `did_not_sustain`
# (then NO .bin exists — the ladder was exhausted and no latency was
# minted); later `#` lines are provenance. Written by every runner ONLY
# under fixed100 (the native binaries + workspace runner write their own;
# bench.py writes the ROS 2 cells' here, since it drives the per-rung
# container invocations). compile_csv.py folds the first line into the
# achieved_rate_hz CSV column; plot.py annotates non-target points.
# ----------------------------------------------------------------------------

DID_NOT_SUSTAIN = "did_not_sustain"

def rate_sidecar_path(raw_dir: Path, raw_name: str, payload: int) -> Path:
    return raw_dir / f"{raw_name}_{payload}.rate"

def read_rate_sidecar(path: Path) -> Optional[str]:
    """First line of a .rate sidecar (stripped), or None when absent /
    unreadable — readers consume the first line only."""
    try:
        with path.open() as f:
            return f.readline().strip()
    except OSError:
        return None

def write_rate_sidecar(path: Path, first_line: str, provenance: str) -> None:
    path.write_text(f"{first_line}\n# {provenance}\n")

def payload_accounted(raw_dir: Path, raw_name: str, payload: int) -> bool:
    """A (cell, payload) is ACCOUNTED when its .bin exists OR a fixed100
    did_not_sustain sidecar records the ladder exhaustion — the recorded
    no-latency outcome is an accounted result, never a silent hole (the
    exact-set gates below accept it; compile_csv renders the empty row)."""
    if (raw_dir / f"{raw_name}_{payload}.bin").exists():
        return True
    return read_rate_sidecar(
        rate_sidecar_path(raw_dir, raw_name, payload)) == DID_NOT_SUSTAIN

def samples_for(variant: str, payload: int) -> Tuple[Optional[int], int, int]:
    """(rate_hz_or_None, measured_samples, warmup) for one cell payload.

    G1 contract: CER_BENCH_TARGET_SAMPLES means
    MEASURED samples in every component, so this returns MEASURED counts
    ready to export. The quiescent schedule tuple stays (rate, TOTAL,
    warmup); the runner layer — here — derives measured = total − warmup
    before exporting, so every stack dumps identical per-payload measured
    counts (55000 @64B, ..., 2000 @4MiB and @16MiB).

    Quiescent: derived from the schedule (rate is meaningful). fixed100:
    the uniform (100, 2000 measured, 100) tuple at every size (the rate
    here is the TARGET — the fallback ladder overrides it per rung via
    docker_args_for_cell's rate_override). Backtoback: no rate
    (saturation); the CER_BENCH_TARGET_SAMPLES / CER_BENCH_WARMUP env
    pair ALREADY carries (measured, warmup) semantics — passed through
    unchanged. The ambient-refusal in refuse_ambient_sample_overrides keeps a
    stray export from silently degrading a full sweep, so a value that
    reaches this point was injected deliberately (smoke) or explicitly
    waved through."""
    if variant == "quiescent":
        rate, total, warmup = quiescent_schedule(payload)
        return rate, total - warmup, warmup
    if variant == "fixed100":
        rate, total, warmup = fixed100_schedule(payload)
        return rate, total - warmup, warmup
    measured = int(os.environ.get("CER_BENCH_TARGET_SAMPLES", str(BACKTOBACK_TARGET_SAMPLES)))
    warmup = int(os.environ.get("CER_BENCH_WARMUP", str(BACKTOBACK_WARMUP)))
    return None, measured, warmup

# ============================================================================
# Native line inventory
# ============================================================================

@dataclass(frozen=True)
class NativeBench:
    bin_name: str      # cargo bin target in native/
    raw_prefix: str    # CER_BENCH_RAW_NAME = f"{raw_prefix}_chrt{N}"
    chrt_on: bool      # False = chrt0-only (spin-bound gains nothing from RT)
    spawns_pong: bool = False  # 2-process benches spawn their pong themselves

NATIVE_BENCHES: Tuple[NativeBench, ...] = (
    # (cerulion_user_round_trip RETIRED: the
    # manual-tick pattern must not be showcased; workspace legs are the
    # user-facing Cerulion lines. The API-layer cost it isolated is gated
    # internally by cerulion_core's release latency tests.)
    # FLOOR: raw iceoryx2, upstream-canonical shape. Spin-bound — chrt0 only.
    NativeBench("raw_iceoryx2_round_trip", "iox2", chrt_on=False),
    # COMPARISON: zenoh SHM, z_ping/z_pong 2-process shape (the main binary
    # spawns zenoh_shm_round_trip_pong itself).
    NativeBench("zenoh_shm_round_trip", "zenoh_shm", chrt_on=True, spawns_pong=True),
)

# ============================================================================
# Workspace leg inventory
# ============================================================================

# split FIRST — it is the headline row: the hand-written 2-group
# `process_groups:` graph (`graph run rtt_bench_split`, zero flags,
# real-clock mp). Split REPLACES the flagless `default` leg because
# the derived process-per-node shape currently parks on an
# un-ringable doorbell (the park-wake bug) and
# reads ~25.9 µs p50 where split measures 4.92 µs — the representative
# multi-process number. The flagless default returns as the headline
# when the park-wake fix lands; until then run_workspace.sh REFUSES a requested
# `default` leg loudly. mono = --single-process, nothing else.
WORKSPACE_LEGS = ("split", "mono")

# TYPE-CLASS axis (METHODOLOGY § "The type-class axis"): "variable" is
# the INCUMBENT class — the shipped workspace legs already ride
# sensor_msgs/Image with the unbounded `data` field loaned per tick, so
# their pinned prefixes (cerulion_workspace_{split,mono}_chrt{N}) carry
# NO class token and stay byte-identical (renaming a pinned prefix
# silently drops the line from plots and orphans per-host smoke
# baselines). "pod" is the NEW fixed-PodPayload twin — its rows carry a
# `_pod` token. Enumeration order: variable first (the incumbent
# headline class).
WORKSPACE_MSG_CLASSES = ("variable", "pod")

def workspace_raw_name(leg: str, chrt: int, msg: str = "variable") -> str:
    # An unknown class must NOT fall through to the incumbent pinned
    # prefix: that silently maps it onto the VARIABLE class's name, which
    # is what every downstream consumer classifies on. (Ros2Cell.name has
    # the opposite failure mode for the same mistake — it appends the
    # unknown value as a token — so neither is left to inference.)
    if msg not in WORKSPACE_MSG_CLASSES:
        raise ValueError(
            f"workspace_raw_name: msg={msg!r} is not one of "
            f"{WORKSPACE_MSG_CLASSES}")
    if msg == "pod":
        return f"cerulion_workspace_{leg}_pod_chrt{chrt}"
    return f"cerulion_workspace_{leg}_chrt{chrt}"

def workspace_leg_skip_reason(leg: str, variant: str = "quiescent") -> Optional[str]:
    """Structural gate for a workspace leg (None = runnable).

    split x backtoback: --time-source virtual on a process_groups graph
    dispatches to the supervisor with the time-source IGNORED (real-clock
    mp has no uncapped mode; the period floor is 1 ms), so a "backtoback"
    split row would really be a 1 kHz-capped line masquerading as
    saturation (run_workspace.sh exits 77 on it; we skip it up front)."""
    if leg == "split" and variant == "backtoback":
        return ("split x backtoback is structurally unmeasurable — the "
                "supervisor ignores --time-source (no uncapped mp mode; "
                "a 1 kHz-capped line would masquerade as saturation); "
                "use the mono leg")
    return None

# ============================================================================
# ROS 2 cell matrix
# ============================================================================

ROS2_DISTROS = ("humble", "jazzy", "lyrical")
# The stock comparison RMWs: the three an unmodified ROS 2 install can
# select. The name is load-bearing: several rules below key on "is this
# one of the shipped vendors" (which rmws get the rel10 twin, which get a
# no_shm lane), and those rules must NOT silently acquire a new member.
STOCK_RMWS = ("cyclonedds", "fastdds", "zenoh")

# rmw_cerulion: ROS 2 over the Cerulion zero-copy transport, re-admitted
# to this suite after an earlier removal. The reason is
# specific and narrow: the README's ROS 2 chart draws stock and composed
# ROS 2 lines measured BY THIS HARNESS, and an rmw_cerulion series drawn
# beside them has to come from the same harness or the comparison is
# between two different experiments.
#
# It is a SEPARATE tuple, not a fourth entry in STOCK_RMWS, because it is
# not a stock vendor and the vendor-keyed rules above must keep meaning
# what they say. The matrix axis is the concatenation.
CERULION_RMWS = ("cerulion",)
MATRIX_RMWS = STOCK_RMWS + CERULION_RMWS

# TYPE-CLASS axis on the ROS 2 stack. The workspace stack spells the
# unbounded class `variable` (WORKSPACE_MSG_CLASSES) — the two
# vocabularies differ deliberately (METHODOLOGY § "The type-class axis").
# Element 0 is the incumbent class whose cell names carry no token.
ROS2_MSG_CLASSES = ("pod", "image")

# Runtime skip convention: a cell that exits rc=77 declared ITSELF
# structurally unrunnable (the ros2/run_bench.sh contract, e.g. the
# zenoh×loan NO-OP stub if a future enumeration change re-admits it).
RC_STRUCTURAL_SKIP = 77

@dataclass(frozen=True)
class Ros2Cell:
    distro: str   # humble | jazzy | lyrical
    rmw: str      # cyclonedds | fastdds | zenoh — or a usage-pattern LANE:
                  # stock (zero-config default) | composed (one-process
                  # composition); see ros2/memo.md
    shm: str      # shm | no_shm | zc — lanes: stock | ipcon | ipcoff
    recv: str     # rclcpp | loan (lanes: always rclcpp)
    qos: str      # be1 | rel10 — lanes: always stock
                  # (rmw_qos_profile_default)
    chrt: int     # 0 | 1
    msg: str = "pod"  # TYPE-CLASS axis (METHODOLOGY § "The type-class
                  # axis"): pod (default — the incumbent Pod<N> cells,
                  # names unchanged) | image (sensor_msgs/msg/Image,
                  # the unbounded variable class; matrix rmws ×
                  # shm × rclcpp × be1 only — lanes stay pod-only)

    def __post_init__(self) -> None:
        # `name` below DROPS the class token on the two lane branches, so
        # a lane cell built with msg="image" would mint a name identical
        # to its pod twin: two distinct cells, one CSV stem, one baseline
        # key. The lanes are pod-only by enumeration anyway (run_bench.sh
        # refuses image x {stock,composed}), so refuse the state HERE
        # rather than let `name` lie about it.
        if self.msg not in ROS2_MSG_CLASSES:
            raise ValueError(
                f"Ros2Cell.msg={self.msg!r} is not one of {ROS2_MSG_CLASSES}")
        if self.rmw in ("stock", "composed") and self.msg != ROS2_MSG_CLASSES[0]:
            raise ValueError(
                f"Ros2Cell: the {self.rmw} lane is pod-only — msg="
                f"{self.msg!r} would mint the same name as its pod twin "
                f"(the lane branches of `name` carry no class token)")

    @property
    def name(self) -> str:
        # Lane names carry no qos token: the lane DEFINES its qos
        # (stock = rmw_qos_profile_default) and run_bench.sh hard-errors
        # on anything else, so the token would repeat the lane label.
        # stock also drops the shm token (its shm field IS the lane
        # marker); composed keeps it — ipc{on,off} is the lane's one
        # real axis. The type-class token appears ONLY on non-pod cells
        # (the incumbent pod names are pinned — renaming would silently
        # drop them from plots and orphan per-host smoke baselines).
        if self.rmw == "stock":
            return f"{self.distro}_stock_{self.recv}_chrt{self.chrt}"
        if self.rmw == "composed":
            return f"{self.distro}_composed_{self.shm}_{self.recv}_chrt{self.chrt}"
        msg_tok = "" if self.msg == "pod" else f"_{self.msg}"
        return (f"{self.distro}_{self.rmw}_{self.shm}{msg_tok}"
                f"_{self.recv}_{self.qos}_chrt{self.chrt}")

@dataclass(frozen=True)
class SkipNote:
    """One '! skip' inventory line: a cell (or cell family) deliberately not
    enumerated, with the reason stated so absence is auditable."""
    scope: str    # concrete cell name, or a family glob for scope-level notes
    reason: str

def enumerate_ros2_cells(distro: str, chrt: int) -> Tuple[List[Ros2Cell], List[SkipNote]]:
    """Enumerate the ROS 2 cell matrix for one (distro, chrt).

    Returns (cells, structural_skips). The skips are the deliberate holes in
    the matrix a reader might otherwise expect, printed as a '! skip'
    inventory by every runner so nothing vanishes silently.
    """
    cells: List[Ros2Cell] = []
    skips: List[SkipNote] = []

    if distro != "jazzy":
        skips.append(SkipNote(
            f"{distro}_*_no_shm_*",
            "no_shm cells are enumerated on jazzy only (the May pin)"))

    # zc is deliberately fastdds-only: DataSharing is FastDDS's SEPARATE
    # zero-copy mechanism (its `shm` lane is the SHM transport with
    # DataSharing forced OFF by rmw_fastrtps' defaults — the out-of-box ROS 2
    # behavior). The other RMWs' shm lanes already carry their zero-copy
    # paths (cyclonedds: iceoryx/PSMX; zenoh: SHM provider), so a zc twin
    # would enumerate nothing new.
    skips.append(SkipNote(
        f"{distro}_{{cyclonedds,zenoh}}_zc_*",
        "zc (DataSharing) is a FastDDS-specific mechanism; the other RMWs' "
        "shm lanes already ride their zero-copy paths"))

    # TYPE-CLASS axis structural holes (the class hypothesis rendered as
    # skip inventory — run_bench.sh enforces the same combinations with
    # exit-77 gates for manual in-container runs): the image (variable /
    # unbounded) class enumerates on matrix rmws × shm × rclcpp × be1
    # ONLY.
    skips.append(SkipNote(
        f"{distro}_*_shm_image_loan_*",
        "unbounded types cannot loan — can_loan_messages gates on is_plain "
        "on every RMW that implements loans, so an image loan-take lane is "
        "structurally empty (the image class's receive cost IS the rclcpp "
        "lane's delivery memcpy)"))
    skips.append(SkipNote(
        f"{distro}_fastdds_zc_image_*",
        "FastDDS DataSharing requires plain, bounded types — an unbounded "
        "data vector disqualifies the type at the DDS layer; the image "
        "class's serialize+copy cost shows on the shm lane"))
    skips.append(SkipNote(
        f"{distro}_*_image_*_rel10_* / {distro}_*_no_shm_image_*",
        "image cells pin be1 × shm only (the rel10 pin was a pod SHM "
        "head-to-head; no_shm answers no type-class question)"))

    for rmw in MATRIX_RMWS:
        # shm everywhere; zc (FastDDS DataSharing zero-copy) is
        # fastdds-only with the same distro coverage as shm; no_shm only on
        # jazzy.
        shm_modes = ["shm"]
        if rmw == "fastdds":
            # The stock `shm` lane is rmw_fastrtps' out-of-box behavior
            # (data_sharing().off() unless RMW_FASTRTPS_USE_QOS_FROM_XML=1),
            # so the vendor-recommended zero-copy config is a SEPARATE
            # labeled lane: shmmode `zc` = fastdds_zc.xml (data_sharing
            # AUTOMATIC in the pub/sub default profiles) + QOS_FROM_XML —
            # the rmw_fastrtps README's own recipe. Both recv lanes; be1
            # everywhere + rel10 per the jazzy pin below.
            #
            # EXCEPT humble (measured on latency_bench:humble, 2026-08-12):
            # exporting RMW_FASTRTPS_USE_QOS_FROM_XML=1 — the zc recipe's
            # own REQUIRED env — crashes rmw node creation outright
            # (rc=139, "rcl node's rmw handle is invalid") with EITHER
            # profiles XML (fastdds_zc.xml or the plain shm profile), while
            # either XML WITHOUT the env is rc=0. The zc lane therefore
            # cannot exist on humble's Fast DDS 2.6 / rmw_fastrtps era —
            # a structural hole, not a flaky cell; skipped for all
            # recv/qos/chrt so the absence is auditable in the inventory.
            if distro == "humble":
                skips.append(SkipNote(
                    "humble_fastdds_zc_*",
                    "RMW_FASTRTPS_USE_QOS_FROM_XML=1 (the zc recipe's own "
                    "required env) crashes rmw node creation on humble — "
                    "rc=139, \"rcl node's rmw handle is invalid\", with "
                    "either profiles XML; either XML without the env is "
                    "rc=0 (measured on latency_bench:humble). "
                    "No zc lane exists on humble's Fast DDS 2.6/rmw era"))
            else:
                shm_modes.append("zc")
        if distro == "jazzy" and rmw != "cerulion":
            shm_modes.append("no_shm")
        elif distro == "jazzy":
            # rmw_cerulion has ONE data path: iceoryx2 shared memory.
            # There is no network transport to fall back to, so `no_shm`
            # names a configuration that cannot be built, not a cell
            # that is merely expected to fail. Recorded as a skip so the
            # absence is auditable in the inventory rather than looking
            # like an oversight; run_bench.sh refuses the same pair.
            skips.append(SkipNote(
                "jazzy_cerulion_no_shm_*",
                "rmw_cerulion's only data path is iceoryx2 shared memory "
                "- there is no non-SHM transport to measure, so a "
                "no_shm cell would carry a label no run can honour"))

        for shm in shm_modes:
            # loan (rcl_take_loaned_message) is an SHM-family-only lane
            # (shm + zc, never no_shm).
            recvs = ("rclcpp", "loan") if shm != "no_shm" else ("rclcpp",)
            for recv in recvs:
                if recv == "loan" and rmw == "zenoh":
                    skips.append(SkipNote(
                        Ros2Cell(distro, rmw, shm, recv, "be1", chrt).name,
                        "zenoh loan lane is a NO-OP stub upstream (rc=77 "
                        "structural skip — re-verify on new rmw_zenoh versions)"))
                    continue
                # lyrical × cyclonedds × loan — UPSTREAM-BROKEN, root
                # cause CLOSED by the 2026-08-13 fairness audit's minimal
                # repro (one .cpp, plain rcl, Pod1024, be1,
                # ROS_DISABLE_LOANED_MESSAGES=0, iox-roudi up; built +
                # run in latency_bench:lyrical on box-x86; evidence at
                # box-x86:~/loan-repro/): rmw_cyclonedds 4.1.4
                # NEVER INITIALIZES ArrayValueType::m_is_self_contained
                # (TypeSupport2.hpp:229 — declared, absent from the ctor
                # init list), so is_self_contained() returns an
                # INDETERMINATE bool (UB) for every message type with a
                # fixed-size array member — i.e. every Pod<N> here and
                # most real sensor types. Both garbage values are fatal:
                #   false → can_loan_messages=0 at the RMW level on pub
                #     AND sub; every rcl_take_loaned_message returns
                #     UNSUPPORTED (rmw_node.cpp:2833/3602) — the bench's
                #     Ready→take-fail 100%-core spin (pong echoed 0 of
                #     285k pings), and loaned PUBLISH silently downgrades
                #     on the rclcpp lane too (lyrical logs loaned=0).
                #   true (observed under gdb) → the guard passes and
                #     dds_take's loan path hits the rmw's unimplemented
                #     sertype op (serdata.cpp:660) → uncaught
                #     std::logic_error("not implemented") → SIGABRT
                #     inside rcl_take_loaned_message.
                # The DDS data plane is fine (PSMX iox loads, ports
                # created, plain take 200/200) — the break is rmw-side.
                # Bug present on the upstream lyrical branch tip as of
                # 2026-08-13; no matching upstream issue (nearest family
                # ros2/rmw_cyclonedds#585; loan API history PR#297).
                # jazzy/humble cyclonedds loan cells STAY — audited
                # CORRECT AND FAIR (flat ~10 µs RTT p50 64 B → 16 MiB,
                # full delivery).
                if recv == "loan" and rmw == "cyclonedds" and distro == "lyrical":
                    skips.append(SkipNote(
                        Ros2Cell(distro, rmw, shm, recv, "be1", chrt).name,
                        "UPSTREAM-BROKEN: rmw_cyclonedds 4.1.4 never "
                        "initializes ArrayValueType::m_is_self_contained "
                        "(TypeSupport2.hpp:229) — is_self_contained() is "
                        "an indeterminate bool (UB) for any array-bearing "
                        "type, so loans are either refused per call "
                        "(can_loan=0, take rc=UNSUPPORTED — the "
                        "Ready→take-fail 100%-core spin, pong 0/285k) or "
                        "the process SIGABRTs via an uncaught "
                        "std::logic_error inside rcl_take_loaned_message "
                        "(gdb-verified). Minimal plain-rcl repro: "
                        "box-x86:~/loan-repro/ (audit reproduction). "
                        "Observed on the audited lyrical revision; nearest "
                        "upstream: ros2/rmw_cyclonedds#585. jazzy/humble "
                        "cyclonedds loan cells stay (audited fair) — "
                        "re-verify when rmw_cyclonedds ships the "
                        "one-line init fix"))
                    continue
                # qos axis: be1 everywhere (the May pin); rel10 is
                # SHM-family-only (shm + zc, never no_shm), enumerated on
                # jazzy × {cyclonedds, fastdds} × rclcpp — which for the
                # zc mode means the fastdds zc lane inherits the same
                # jazzy rel10 pin (the pin was an SHM
                # head-to-head — the choice is made after results; no no_shm
                # rel10 cells exist).
                qoses = ["be1"]
                if (distro == "jazzy" and recv == "rclcpp"
                        and rmw in ("cyclonedds", "fastdds")):
                    if shm != "no_shm":
                        qoses.append("rel10")
                    else:
                        skips.append(SkipNote(
                            Ros2Cell(distro, rmw, shm, recv, "rel10", chrt).name,
                            "rel10 is SHM-only (the original rmw pin was an SHM "
                            "head-to-head)"))
                for qos in qoses:
                    cells.append(Ros2Cell(distro, rmw, shm, recv, qos, chrt))
                # TYPE-CLASS axis (METHODOLOGY § "The type-class
                # axis"): beside each pod cell's shm × rclcpp × be1
                # incarnation, an `image` twin — sensor_msgs/msg/Image
                # with its unbounded data array sized to the sweep
                # point. The class hypothesis is that unbounded types
                # force every RMW into full-serialize + delivery-memcpy
                # (no loan, no DataSharing), so the twin enumerates
                # ONLY where the type is structurally admissible:
                # shm mode (where the serialize+copy cost shows against
                # the pod baseline), rclcpp receive (loan take cannot
                # engage on a non-plain type), be1 (the pod cells' May
                # pin; the rel10 pin was a pod SHM head-to-head).
                if shm == "shm" and recv == "rclcpp":
                    cells.append(
                        Ros2Cell(distro, rmw, shm, recv, "be1", chrt,
                                 msg="image"))

    # Usage-pattern lanes (ros2/memo.md):
    # pseudo-rmw cells that never cross the matrix axes — run_bench.sh
    # hard-errors on any cross-labeled combination.
    #
    #   stock — the ZERO-CONFIG slowest-common lane (memo §3): what
    #     `ros2 run` gives you. rmw_fastrtps_cpp (the default rmw), NO
    #     profiles XML, NO transport env (Fast DDS's default UDPv4 +
    #     builtin copy-based SHM transport; DataSharing OFF at the rmw
    #     layer), qos=stock (rmw_qos_profile_default = RELIABLE/VOLATILE/
    #     KEEP_LAST(10)), plain publish + typed-callback receive, 3
    #     separate processes. No verify_shm gate — the lane claims no
    #     transport-engagement label (a per-cell provenance note is
    #     written instead).
    #   composed — the FASTEST-COMMON lane (memo §2): composition is
    #     mainstream (Nav2-default since Humble, PR #2750) while
    #     intra-process comms is the opt-in shade (off by default even
    #     composed; Nav2 gained the option in Kilted→Lyrical, PR #5804)
    #     — so BOTH shades are cells: one process, three
    #     manually-composed nodes on one single-threaded executor
    #     (composed_rtt_node), ipc{on,off} → use_intra_process_comms.
    #
    # Distro coverage: enumerated on ALL distros; jazzy is first-class,
    # humble/lyrical are loudly UNVERIFIED until swept (cmd_ros2 prints
    # the warning per lane cell off jazzy).
    cells.append(Ros2Cell(distro, "stock", "stock", "rclcpp", "stock", chrt))
    for ipc in ("ipcon", "ipcoff"):
        cells.append(Ros2Cell(distro, "composed", ipc, "rclcpp", "stock", chrt))

    return cells, skips

def print_skip_inventory(skips: Sequence[SkipNote], indent: str = "  ") -> None:
    for s in skips:
        print(f"{indent}! skip {s.scope} ({s.reason})")

def interleave_rmws(cells: Sequence[Ros2Cell]) -> List[Ros2Cell]:
    """RUN-order interleave for one (distro, chrt) block:
    cycle the rmw axis per position instead of running all
    cells of one rmw hours before its head-to-head arm's (against
    METHODOLOGY §10's own same-window rule). Within-rmw cell order is
    preserved; with `full --reps N` this is belt-and-braces on top of the
    rep-granularity round-robin. Enumeration (inventory, plots, skips) is
    untouched — this reorders execution only."""
    queues: Dict[str, List[Ros2Cell]] = {}
    for c in cells:
        queues.setdefault(c.rmw, []).append(c)
    out: List[Ros2Cell] = []
    while any(queues.values()):
        for rmw in list(queues):
            if queues[rmw]:
                out.append(queues[rmw].pop(0))
    return out

# ============================================================================
# Platform helpers
# ============================================================================

def find_chrt_prefix() -> Optional[List[str]]:
    """chrt -f 80 command prefix, or None if unavailable (non-Linux: None)."""
    if not IS_LINUX:
        return None
    if shutil.which("chrt") is None:
        return None
    try:
        if subprocess.run(["chrt", "-f", "80", "true"],
                          capture_output=True, timeout=5).returncode == 0:
            return ["chrt", "-f", "80"]
        # `sudo` needs its OWN guard: the `which` above is for chrt. A Linux
        # host that HAS chrt, cannot use it unprivileged, and ships no sudo
        # (a container, a hardened image) reached this line and raised
        # FileNotFoundError — an OSError, which the TimeoutExpired arm below
        # does not catch. That escaped as a traceback out of a routine
        # capability PROBE, turning every caller's documented setup-failure
        # exit into a crash. Absent sudo is simply "no escalation available",
        # which is the same answer as `sudo -n` being refused. OSError is
        # caught as well as guarded: `which` is a lookup, not a promise, and
        # an exec can fail for reasons other than ENOENT (a directory on
        # PATH, a bad interpreter, EACCES).
        if shutil.which("sudo") is not None and subprocess.run(
                ["sudo", "-n", "chrt", "-f", "80", "true"],
                capture_output=True, timeout=5).returncode == 0:
            return ["sudo", "-nE", "chrt", "-f", "80"]
    except subprocess.TimeoutExpired:
        print("  ! chrt probe timed out after 5s — treating chrt as "
              "unavailable; this is NOT a permissions problem",
              file=sys.stderr)
        return None
    except OSError as e:
        # ENOENT is the ordinary "not installed / no sudo" answer and is
        # already reported by the caller. Anything else — EACCES on the
        # binary, ENOMEM or EAGAIN at fork, EMFILE, ENOEXEC from a bad
        # shebang — means the probe could not RUN, which is a different
        # fact from "you lack rtprio". Both used to collapse to the same
        # silent None, and the caller then told the operator to edit
        # limits.conf, a remedy that cannot fix any of them, while the run
        # continued and recorded "chrt unavailable on this host" in the
        # manifest as its published provenance.
        if not isinstance(e, FileNotFoundError):
            print(f"  ! chrt probe could not run ({e}) — treating chrt as "
                  f"unavailable; this is an ENVIRONMENT fault, NOT a "
                  f"permissions problem", file=sys.stderr)
        return None
    return None

def warn_if_chrt_requested_but_unavailable(chrt: int) -> bool:
    """True = proceed; False = caller should skip chrt-on cells."""
    if chrt == 0:
        return True
    if IS_LINUX:
        if find_chrt_prefix() is None:
            user = os.environ.get("USER") or os.environ.get("LOGNAME") or "current user"
            print(f"  ! chrt -f 80 unavailable for {user} — chrt-on cells "
                  f"skipped", file=sys.stderr)
            # "If" — the probe reports an environment fault itself (see
            # find_chrt_prefix), and this line used to assert a permissions
            # cause for every one of them.
            print("    If that is a permissions problem: add 'rtprio 99' to "
                  "/etc/security/limits.conf, or configure passwordless "
                  "sudo.", file=sys.stderr)
            return False
        return True
    print(f"  ! chrt is Linux-only — skipping chrt-on cells on {PLATFORM_LABEL}",
          file=sys.stderr)
    return False

def cleanup_iceoryx() -> None:
    """Remove iceoryx2 / zenoh SHM state between cells."""
    iox_dir = Path("/tmp/iceoryx2")
    if iox_dir.exists():
        shutil.rmtree(iox_dir, ignore_errors=True)
        if iox_dir.exists() and IS_LINUX:
            # A root-owned chrt-on run may have left files behind.
            # Guarded + OSError-caught for the same reason as
            # find_chrt_prefix: absent sudo is "no escalation available",
            # not a reason to abort a cleanup pass.
            if shutil.which("sudo") is not None:
                try:
                    subprocess.run(["sudo", "-n", "rm", "-rf", str(iox_dir)],
                                   capture_output=True)
                except OSError:
                    pass
            if iox_dir.exists():
                print(f"  ! {iox_dir} survived cleanup (root-owned residue "
                      f"from a chrt-on run, and no usable sudo) — the next "
                      f"cell starts against a DIRTY SHM root", file=sys.stderr)
    shm_dir = Path("/dev/shm")
    if shm_dir.exists():
        for pattern in ("iceoryx_*", "iox2_*", "zenoh*", "zenohshm*", "*.zenoh"):
            for f in shm_dir.glob(pattern):
                try:
                    f.unlink()
                except OSError:
                    pass

_REAP_UNAVAILABLE_WARNED = False

def kill_stragglers() -> None:
    """Kill leftover bench binaries before starting a new cell."""
    if IS_WINDOWS:
        return
    patterns = [
        f"{NATIVE_BIN_DIR}/raw_iceoryx2_round_trip",
        f"{NATIVE_BIN_DIR}/zenoh_shm_round_trip",
    ]
    # pkill is procps and is routinely absent from minimal images. Guarding
    # the spawn was necessary (the crash was worse), but silence is not: a
    # survivor from a previous cell competes for CPU and inflates the NEXT
    # cell's p50, which is then written to a .bin and compiled into the CSV
    # at rc=0. Nothing downstream can see it — run.json records skips,
    # outcomes, governor and posture, but not whether reaping was possible.
    # So announce it, once per process, the way the sibling teardown reaper
    # already does.
    if shutil.which("pkill") is None:
        global _REAP_UNAVAILABLE_WARNED
        if not _REAP_UNAVAILABLE_WARNED:
            _REAP_UNAVAILABLE_WARNED = True
            print("  ! pkill not on PATH — leftover bench processes CANNOT "
                  "be reaped between cells. A survivor competes for CPU and "
                  "inflates the next cell's p50 with rc=0. Install procps "
                  "before a citable sweep.", file=sys.stderr)
    if shutil.which("pkill") is not None:
        for p in patterns:
            try:
                subprocess.run(["pkill", "-9", "-f", p], capture_output=True)
            except OSError:
                pass

def raise_fd_limit(target: int = 65536) -> None:
    """Raise RLIMIT_NOFILE toward `target` (ulimit -n 65536).

    An fd ceiling too low surfaces as iceoryx2 ServiceInCorruptedState (the
    documented pitfall) — warn LOUDLY when the target can't be reached
    instead of letting the run fail mysteriously later."""
    try:
        import resource
    except ImportError:            # Windows
        print("  ! cannot raise fd limit on this platform (no resource module) "
              "— iceoryx2 may hit ServiceInCorruptedState under fd pressure",
              file=sys.stderr)
        return
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    want = target if hard == resource.RLIM_INFINITY else min(target, hard)
    if soft < want:
        try:
            resource.setrlimit(resource.RLIMIT_NOFILE, (want, hard))
            soft = want
        except (ValueError, OSError) as e:
            print(f"  ! could not raise fd limit ({e})", file=sys.stderr)
    if soft < target:
        print(f"  ! fd limit is {soft} (< {target}) — iceoryx2 can fail with "
              f"ServiceInCorruptedState under fd pressure; raise the hard limit "
              f"(ulimit -n {target})", file=sys.stderr)

# zenoh-SHM needs a memlock ceiling well above the 8 MiB Linux default:
# the session's 64 MiB transport-optimization pool plus a per-size SHM
# provider (1 MiB floor, 2x payload at the top sizes => 32 MiB at 16 MiB
# payloads) all count against RLIMIT_MEMLOCK — and zenoh 1.7 WEDGES
# (blocks forever) instead of erroring when the limit is hit mid-run.
# Measured on box-x86 2026-08-12: at the 8 MiB default the zenoh
# line hangs deterministically at its 5th payload size (16 KB); with
# memlock unlimited the identical binary completes all 10 sizes.
# PITFALLS #12 documents the container twin (docker gets --ulimit
# memlock=-1); this is the bare-host half.
MEMLOCK_MIN_BYTES = 256 * 1024 * 1024

def ensure_memlock_for_zenoh() -> bool:
    """True = zenoh-SHM cells may run; False = caller must skip/refuse them.

    Tries, in order: soft->hard setrlimit in-process (children inherit),
    then `sudo -n prlimit --memlock=unlimited` on our own pid (the same
    passwordless-sudo pattern as the chrt fallback). Never proceeds
    silently into the wedge: a zenoh cell under a low memlock burns its
    whole watchdog window and produces nothing."""
    if not IS_LINUX:
        return True                # macOS: no RLIMIT_MEMLOCK interaction
    import resource
    def current() -> int:
        soft, _ = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        return soft
    soft, hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    if soft != resource.RLIM_INFINITY and (hard == resource.RLIM_INFINITY
                                           or hard > soft):
        try:
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (hard, hard))
        except (ValueError, OSError):
            pass
    if current() == resource.RLIM_INFINITY or current() >= MEMLOCK_MIN_BYTES:
        return True
    # Self-elevate via passwordless sudo; children inherit the raised limit.
    # This one mattered most: it runs on the smoke path BEFORE the
    # no-baseline check, so an unguarded spawn here raised FileNotFoundError
    # on a low-memlock sudo-less host and smoke exited 1 with a traceback —
    # never reaching the SMOKE_RC_NO_BASELINE return that the wrapper's skip
    # branch keys on. The escalation is optional; the `current()` re-check
    # below decides the outcome either way.
    if shutil.which("sudo") is None:
        print("  ! sudo not on PATH — cannot self-elevate the memlock limit; "
              "the fix below needs a privileged shell", file=sys.stderr)
    else:
        try:
            subprocess.run(["sudo", "-n", "prlimit", "--memlock=unlimited",
                            "--pid", str(os.getpid())], capture_output=True)
        except OSError as e:
            print(f"  ! sudo prlimit could not be started ({e})",
                  file=sys.stderr)
    if current() == resource.RLIM_INFINITY or current() >= MEMLOCK_MIN_BYTES:
        print("  memlock raised to unlimited via sudo prlimit (zenoh-SHM "
              "needs > the 8 MiB default)", file=sys.stderr)
        return True
    print(f"  ! RLIMIT_MEMLOCK is {current()} bytes (< {MEMLOCK_MIN_BYTES}) "
          f"and could not be raised — zenoh-SHM cells would WEDGE mid-sweep "
          f"(measured: hangs at the 5th payload size), so they are REFUSED.\n"
          f"    fix: `sudo prlimit --memlock=unlimited --pid $$` before the "
          f"run, or add 'cerulion - memlock unlimited' to "
          f"/etc/security/limits.d/ (PITFALLS #12).", file=sys.stderr)
    return False

def docker_available() -> bool:
    if shutil.which("docker") is None:
        return False
    try:
        rc = subprocess.run(["docker", "info"], capture_output=True,
                            timeout=10).returncode
    except (subprocess.TimeoutExpired, OSError):
        # OSError as well as the which() above, by this module's own rule:
        # which is a lookup, not a promise (EACCES, a bad interpreter, a
        # directory on PATH). "docker is unavailable" answers all of them.
        return False
    return rc == 0

def _short(path: Path, anchor: Path = REPO_ROOT) -> str:
    try:
        return str(path.relative_to(anchor))
    except ValueError:
        return str(path)

# ============================================================================
# Machine hash + run directory
# ============================================================================

class SmokeSetupError(Exception):
    """Crash/build/setup failure — maps to smoke exit code 3."""

_MACHINE_HASH_CACHE: Optional[str] = None

def compute_machine_hash() -> str:
    """16-char machine hash via the canonical bash lib (memoized per process).

    scripts/benchmarks/lib/machine_hash.sh::compute_live_machine_hash is the
    single source of truth. Reused via subprocess rather than reimplemented
    so the field derivation can never drift between this orchestrator, the
    smoke gate, and the repo-level baseline tooling."""
    global _MACHINE_HASH_CACHE
    if _MACHINE_HASH_CACHE is not None:
        return _MACHINE_HASH_CACHE
    if shutil.which("bash") is None:
        raise SmokeSetupError(
            "bash not on PATH — required for machine-hash computation "
            "(canonical impl lives in scripts/benchmarks/lib/machine_hash.sh)")
    if not MACHINE_HASH_LIB.exists():
        raise SmokeSetupError(f"machine-hash lib missing: {MACHINE_HASH_LIB}")
    try:
        r = subprocess.run(
            ["bash", "-c",
             f"source {shlex.quote(str(MACHINE_HASH_LIB))} && compute_live_machine_hash"],
            capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError) as e:
        raise SmokeSetupError(f"machine-hash computation timed out: {e}") from e
    mh = r.stdout.strip()
    if r.returncode != 0 or not re.fullmatch(r"[0-9a-f]{16}", mh):
        raise SmokeSetupError(
            f"machine-hash computation failed (rc={r.returncode}, "
            f"stdout={mh!r}, stderr={r.stderr.strip()!r})")
    _MACHINE_HASH_CACHE = mh
    return mh

def _git_sha() -> str:
    try:
        r = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                           text=True, cwd=REPO_ROOT, timeout=10)
        if r.returncode == 0:
            return r.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"

def _git_dirty() -> Optional[bool]:
    """True = uncommitted changes in the repo worktree; None = unknowable."""
    try:
        r = subprocess.run(["git", "status", "--porcelain"],
                           capture_output=True, text=True, cwd=REPO_ROOT,
                           timeout=10)
        if r.returncode == 0:
            return bool(r.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return None


# ============================================================================
# The `cerulion` CLI you validate with is part of the test
# ============================================================================
#
# A bench graph validated with a `cerulion` built from a
# DIFFERENT checkout reports a different check count than this tree's
# engine, and the difference is the bug. Nothing in the harness noticed,
# because resolution was "the first `cerulion` on a fixed list" —
# target/debug, then target/release, then PATH — with no question asked
# about WHICH source that binary was built from. A month-old artifact under
# target/ answers that list exactly as well as one built a minute ago.
#
# TWO questions, asked in this order, because they fail differently:
#
#   PROVENANCE — was this binary built from THIS checkout? Usually the path
#     answers it (a binary under the checkout's own target/ is its own
#     build). Under a SHARED `CARGO_TARGET_DIR` the path answers nothing,
#     so the binary's cargo depfile is read instead: it names the absolute
#     source paths of the tree that built it.
#   FRESHNESS — is it newer than the source it claims to be built from?
#     `cerulion --version` prints only the crate version, with no build id
#     or git SHA, so the identity available is TIME and the question is the
#     make question.
#
# Two consumers, two policies, and they genuinely disagree about what an
# UNDATABLE binary means:
#   - the parity harness (`resolve_cerulion_cli`) DECLINES one. Refusing
#     costs two skipped arms; accepting restores exactly the first-match
#     behaviour this check exists to end.
#   - the `CERULION=` override refusal (`refuse_foreign_cerulion_override`,
#     which asks `classify_cerulion_binary` directly) proceeds with a loud
#     warning. Refusing there blocks a whole measurement campaign because
#     `git` hiccuped once, and a false refusal switches a gate off faster
#     than a missing one.

# Verdicts a caller may see.
CLI_FRESH = "fresh"
CLI_STALE = "stale"
CLI_FOREIGN = "foreign"
CLI_UNVERIFIABLE = "unverifiable"
# The closed set, nameable — so an oracle can assert it covers every verdict
# rather than enumerate rows and hope.
CLI_VERDICTS = (CLI_FRESH, CLI_STALE, CLI_FOREIGN, CLI_UNVERIFIABLE)

# What the provenance question can answer.
PROV_IN_TREE = "in_tree"      # built from THIS checkout
PROV_FOREIGN = "foreign"      # built from some other tree
PROV_UNKNOWN = "unknown"      # could be either; nothing here can tell
# The closed set, mirroring CLI_VERDICTS — and, like it, ASSERTED against
# rather than merely declared: `check_cerulion_binary_freshness` requires
# its classifier table to drive every answer in here, so adding one
# without driving it fails the suite instead of shipping unexercised.
PROV_ANSWERS = (PROV_IN_TREE, PROV_FOREIGN, PROV_UNKNOWN)

CERULION_BIN_ENV = "CERULION"


class _NotTheDefault(Exception):
    """Internal: skip the default-path exemption without pretending a
    resolution failed."""
# The remedy every refusal carries. Spelled once so the refusals and
# the oracles that read them cannot drift by a word.
CLI_REBUILD_CMD = "cargo build -p cerulion_cli --release"

# The variable every reader below consults, spelled once.
CARGO_TARGET_DIR_ENV = "CARGO_TARGET_DIR"


def cargo_target_dir_setting() -> Optional[str]:
    """The `CARGO_TARGET_DIR` value CARGO would act on, or None when the
    variable is unset — and a REFUSAL when it is set to an empty string.

    Three readers here ask this question (`cargo_target_dir`,
    `_runner_rebuilds`, `cli_provenance`) and, before this function, each
    answered the empty value differently and all three were wrong in the
    same direction: they INVENTED a configuration cargo does not have.
    `cargo_target_dir()` mapped it to `<repo>/target` (a falsy check),
    `_runner_rebuilds()` inherited that and answered "yes, the runner
    rebuilds the file it runs", and `cli_provenance()` treated it as a
    custom dir, withdrew the path argument and deferred to the DEPFILE —
    which answers `in_tree` on a checkout that has a built CLI (the normal
    state when a bench runs) and `unknown` on one that does not. Three
    answers to one question, none of them cargo's, and the first of those
    two is full ACCEPTANCE — worse than the `unknown` an unbuilt desk
    shows, which is why reading the verdict off one desk misjudges it.

    Cargo REFUSES it outright. Measured on cargo 1.98.1, on a scratch
    crate, with `CARGO_TARGET_DIR=`::

        $ CARGO_TARGET_DIR= cargo build     # and check, and metadata
        error: the target directory is set to an empty string in the
        `CARGO_TARGET_DIR` environment variable
        rc=101

    So nothing in this checkout can be built OR enumerated while the
    variable is spelled that way, and every question this file asks about
    a cargo-built binary is unanswerable: the runner's own `cargo build`
    would exit 101 before writing anything, and a binary already sitting
    on disk belongs to some other configuration entirely. Refusing here
    matches cargo and says so; inventing a target dir would vouch for a
    binary the runner could never have produced.

    EXACTLY empty, never `.strip()`ed — also measured, on the same cargo:
    `CARGO_TARGET_DIR=' '` BUILDS, into a directory literally named " ".
    Whitespace is a legal (if unkind) directory name, so refusing it would
    be this guard inventing a rule cargo does not have, in the direction
    that switches a working setup off.

    Scope: every subcommand that BUILDS or RUNS something reaches this —
    the three readers below on the CLI-resolution paths, the unconditional
    preflight in `refuse_foreign_cerulion_override` (workspace, workspace
    leg, full, smoke), and `main`'s dispatch guard for the two that build
    without resolving a CLI (`native`, `ros2`), whose cargo invocation
    would otherwise fail with "cargo build failed in <dir>" and name
    neither the variable nor the remedy. A purely post-processing
    invocation (`compile-csv`, the plotters, `list-cells`) reaches none of
    them and is not refused."""
    raw = os.environ.get(CARGO_TARGET_DIR_ENV)
    if raw is None or raw != "":
        return raw
    raise SystemExit(
        f"{CARGO_TARGET_DIR_ENV} is set to an EMPTY string. Cargo refuses "
        f"that configuration outright — `cargo build`, `cargo check` and "
        f"`cargo metadata` all exit 101 with \"the target directory is set "
        f"to an empty string in the `{CARGO_TARGET_DIR_ENV}` environment "
        f"variable\" — so nothing in this checkout can be built or dated "
        f"while it is spelled that way, and this harness refuses it too "
        f"rather than inventing a target directory cargo does not have. "
        f"Unset it (`unset {CARGO_TARGET_DIR_ENV}`) to use this checkout's "
        f"default `target/`, or give it a path.")


def cargo_target_dir() -> Path:
    """Where cargo puts this workspace's build output.

    `CARGO_TARGET_DIR` is an ordinary developer setting, and hard-coding
    `REPO_ROOT/target` against it is worse than merely missing the binary:
    a freshly built in-tree CLI is then invisible, and the refusal tells the
    operator to run `cargo build`, which puts the output right back where
    nothing is looking. (The env var is read; a cargo config file is not —
    that would mean reimplementing cargo's config resolution here, and the
    env var is the shape CI and shells use.)

    An EMPTY value is refused by `cargo_target_dir_setting`, not defaulted:
    `if not env` treated it as unset and handed back `<repo>/target`, which
    is a directory cargo would never write under that spelling."""
    env = cargo_target_dir_setting()
    if env is None:
        return REPO_ROOT / "target"
    # A RELATIVE value is resolved by CARGO, from the directory cargo is
    # invoked in — and run_workspace.sh invokes it at the repo root, while
    # this process runs from wherever the operator happened to be.
    # `CARGO_TARGET_DIR=target-cache` therefore means `<repo>/target-cache`
    # to the build and `<cwd>/target-cache` to a naive `Path(env)`, so the
    # freshly built CLI would be looked for in a directory nothing writes.
    target = Path(env)
    return target if target.is_absolute() else REPO_ROOT / target


def _runner_rebuilds(default_cli: Path) -> bool:
    """Will run_workspace.sh's own `cargo build` write the file it then
    runs as `${CERULION:-<default_cli>}`?

    The question is WHERE CARGO WRITES, not whether a variable is set. An
    earlier revision asked the latter, which was right about the case it
    was written for (a shared cache elsewhere) and wrong about an ordinary
    one: `CARGO_TARGET_DIR=target`, or an absolute spelling of this
    checkout's own target dir, still has cargo build exactly
    `<repo>/target/release/cerulion` — the very file the runner runs — and
    the exemption was disabled anyway, so an ABSENT default override was
    refused as "not a file" before the rebuild that would have created it.
    A refusal of nothing, which is the failure mode this guard is most
    careful about.

    Resolved on both sides, so the relative spelling, the absolute one and
    an unset variable all give the same answer."""
    # UNSET needs no comparison at all: cargo's default target dir IS
    # `<workspace root>/target`, so the runner writes the file it runs by
    # definition. Answering this first also keeps the exemption working on
    # a checkout whose path cannot be resolved (a symlink loop), where the
    # comparison below could not run — the case the raw-path arm exists
    # for.
    #
    # An EMPTY value takes neither arm: `cargo_target_dir_setting` refuses
    # it. Answering True there would have been the worst of the three
    # readings — this function's True means "the runner's own `cargo build`
    # writes the very file it then runs", and under an empty value that
    # build exits 101 having written nothing at all.
    if cargo_target_dir_setting() is None:
        return True
    built = cargo_target_dir() / "release" / "cerulion"
    try:
        return built.resolve() == default_cli.resolve()
    except (OSError, RuntimeError):
        # A variable IS set and the two cannot be compared. That is not
        # evidence cargo writes here, and claiming the exemption on a
        # guess is the direction that lets a stale binary through.
        return False


def _is_lexically_under(candidate: Path, parent: Path) -> bool:
    """Path-string containment, symlinks NOT followed."""
    try:
        candidate.relative_to(parent)
    except ValueError:
        return False
    return True


def _deleted_by_the_runner(candidate: Path, doomed: Path) -> bool:
    """Would `rm -rf <doomed>` remove the file `candidate` names?

    LEXICAL, because that is what `rm -rf` is. It removes the ENTRY it is
    given and recurses without following symlinks, so resolving both sides
    (what `_resolves_under` does, correctly, for "where does this binary
    live") answers a different question and gets both directions wrong:

      - a `cerulion` SYMLINK sitting inside a real `target/debug` resolves
        to its target elsewhere, so a resolved check says "not under" and
        ACCEPTS it — while `rm -rf` deletes the entry and the override is
        gone before the run. MEASURED: entry deleted, target survives.
      - when `target/debug` is ITSELF a symlink, `rm -rf` removes only the
        link, so everything in its target survives — while a resolved
        check says "under" and REFUSES a binary that was never at risk.
        MEASURED: target's contents survive.

    Two lexical questions, then. The raw one catches the first case. The
    second covers a candidate reached from OUTSIDE that lives inside the
    tree being deleted (a symlink elsewhere pointing into it): its target
    really is removed — but only when `doomed` is a real directory, since
    a symlinked `doomed` costs its target nothing."""
    if _is_lexically_under(candidate, doomed):
        return True
    try:
        # ONLY a symlink is spared. `rm -rf <entry>` removes whatever the
        # entry is — a directory's contents recursively, and a regular
        # file outright — so a file-valued `target/debug` is deleted too,
        # and an external symlink pointing at it dangles before the run.
        # Excluding "not a directory" spared that case for no reason.
        if doomed.is_symlink():
            return False
        return _is_lexically_under(candidate.resolve(), doomed.resolve())
    except (OSError, RuntimeError):
        return False


def _resolves_under(candidate: Path, parent: Path) -> bool:
    """Is `candidate` inside `parent`, symlinks resolved on both sides?"""
    try:
        candidate.resolve().relative_to(parent.resolve())
    except (ValueError, OSError):
        return False
    return True


def _depfile_tokens(deps: str) -> List[str]:
    """Split a depfile's dependency half into paths.

    NOT `str.split()`. Cargo and rustc escape a literal space in a path as
    `\\ `, per the Makefile depfile convention they inherit, so a bare
    whitespace split tears `/home/dev/my tree/src/main.rs` into two
    fragments — neither of which starts with the checkout root, while both
    still end in `.rs`, so the scan concluded "sources listed, none of them
    ours" and declined this checkout's own build. Anyone whose path
    contains a space has that."""
    return [t.replace("\\ ", " ").replace("\\\\", "\\")
            for t in re.findall(r"(?:[^\s\\]|\\.)+", deps)]


def _depfile_names_this_tree(candidate: Path) -> Optional[bool]:
    """Does the binary's cargo depfile name sources inside THIS checkout?

    True / False / None (no readable depfile, so no answer).

    `cargo build` writes `<binary>.d` beside the binary, listing the
    ABSOLUTE paths of every source that went into it — a provenance record
    that survives a shared target directory, which a path check cannot."""
    # Beside the RESOLVED binary. The containment check that got us here
    # resolves symlinks, so a symlinked candidate (a PATH entry, a
    # `CERULION=` override) passes it while its depfile sits next to the
    # real file — looking beside the raw path found nothing, answered
    # PROV_UNKNOWN, and the override refusal treats UNKNOWN as
    # proceed-with-a-warning, so a symlink into another checkout's build
    # slipped through with a warning where a refusal was owed.
    try:
        real = candidate.resolve()
    except (OSError, RuntimeError):
        real = candidate
    for dep in (real.with_suffix(".d"), candidate.with_suffix(".d")):
        try:
            text = dep.read_text(encoding="utf-8", errors="replace")
            break
        except OSError:
            continue
    else:
        return None
    # REALPATH on both sides, not a raw prefix compare. REPO_ROOT is built
    # from `Path(__file__).resolve()`, so it is already canonical and
    # `{REPO_ROOT, REPO_ROOT.resolve()}` was a ONE-element set claiming to
    # be two spellings. Cargo records the path IT saw, which on a host
    # reached through a symlinked prefix (`/var` -> `/private/var` on
    # macOS, `/home` -> `/export/home` at many sites) is the unresolved
    # one — so a canonical-only comparison matched nothing and declined a
    # binary this checkout really built.
    root_real = os.path.realpath(str(REPO_ROOT)) + os.sep
    root_raw = str(REPO_ROOT) + os.sep
    # EVERY `.rs` token, not the first one. A depfile lists everything that
    # contributed — generated `$OUT_DIR` sources included, which under a
    # shared target dir sit outside the checkout by construction — and the
    # set is written SORTED, so which token comes first is an accident of
    # path spelling.
    saw_rs = False
    for line in text.splitlines():
        # `<out>: <src> <src> ...` — take the dependency half.
        _, _, deps = line.partition(": ")
        for token in _depfile_tokens(deps):
            if not token.endswith(".rs"):
                continue
            saw_rs = True
            if token.startswith(root_raw):
                return True
            if os.path.realpath(token).startswith(root_real):
                return True
    # Sources listed, none of them ours: another tree's build. No `.rs`
    # tokens at all is not an answer, it is an unreadable depfile.
    return False if saw_rs else None


def cli_provenance(candidate: Path) -> str:
    """PROV_IN_TREE / PROV_FOREIGN / PROV_UNKNOWN for one candidate binary.

    The DEFAULT `REPO_ROOT/target` is proof by construction, and only
    while it IS the default: nothing but this checkout's own cargo writes
    there unless somebody points another checkout at it explicitly.

    A `CARGO_TARGET_DIR` is never proof, wherever it points.
    `$CARGO_TARGET_DIR/release/cerulion` is whatever tree built last, and
    exporting one in a shell rc is the standard way to share a build cache
    across checkouts — so calling it "in tree" would hand the parity
    harness a foreign binary with no complaint at all. Setting one therefore
    withdraws the path argument
    from the default location TOO, because a custom target dir can be
    NESTED inside `REPO_ROOT/target` (`CARGO_TARGET_DIR=$PWD/target/shared`
    is an ordinary thing to write) and would otherwise be readmitted by the
    very check it is supposed to bypass — measured: that spelling returned
    `in_tree` for a binary with no depfile at all.

    Where the path cannot answer, the depfile does; where it cannot be
    read, nobody here can tell, which is PROV_UNKNOWN (declined by the
    harness, warned about by the override refusal) rather than a guess in
    either direction.

    An EMPTY value is none of those three answers: `cargo_target_dir_setting`
    refuses it before the question is asked. What the bare `is None` test
    produced there was whatever the DEPFILE said — `in_tree` on a checkout
    with a built CLI, which ACCEPTS the binary outright, and PROV_UNKNOWN
    on one without, which downgrades the override refusal to a warning.
    Both let the run proceed through a configuration in which cargo cannot
    build at all; the first does it without even a warning."""
    default_target = REPO_ROOT / "target"
    if (cargo_target_dir_setting() is None
            and _resolves_under(candidate, default_target)):
        return PROV_IN_TREE
    # Under EITHER target dir this tree might have written to, the depfile
    # decides. Outside both, no evidence is needed: it is another tree's.
    if not (_resolves_under(candidate, default_target)
            or _resolves_under(candidate, cargo_target_dir())):
        return PROV_FOREIGN
    answer = _depfile_names_this_tree(candidate)
    if answer is True:
        return PROV_IN_TREE
    if answer is False:
        return PROV_FOREIGN
    return PROV_UNKNOWN


# Files that can change the root workspace's `cerulion` binary: its Rust
# sources and manifests, plus the `.msg` corpus that `native_ros2_messages`
# regenerates from (its build.rs declares `rerun-if-changed=msg/`, and
# cerulion_cli depends on that crate), which is why editing a `.msg` really
# does rebuild the CLI.
#
# The excluded tops are trees that cannot: `benches/*` and `examples/*` are
# their own cargo workspaces and `demos/` is not built by this workspace at
# all. Including them would move the floor every time someone edited a
# bench script and refuse a CLI built minutes earlier, which is how a gate
# gets switched off.
#
# SCOPE, and what narrows it. The floor is derived from the files
# that can change THIS workspace's `cerulion` binary. Two filters decide
# that set, and both are deliberately conservative — over-inclusion costs
# a LOUD skip naming the rebuild, while under-inclusion mints a confident
# CLI_FRESH, which is the failure this whole check exists to end.
#
#   1. The excluded tops (below) drop trees that cannot affect it at all:
#      `benches/*` and `examples/*` are their own cargo workspaces and
#      `demos/` is not built by this workspace.
#   2. `_cli_member_closure()` drops workspace members `cerulion_cli` does
#      not link — the viz tree, rmw_cerulion, cerulion_wsd, the test
#      fixtures (232 of 1436 files on this tree). It is derived from the
#      MANIFESTS, never by invoking cargo: this harness is stdlib-only by
#      design and must run where cargo does not.
#
# RESIDUALS of the manifest derivation, each in the safe direction:
#   * Features are NOT resolved. Every path dependency in every dependency
#     table is taken, dev- and build-dependencies included, so a member
#     reachable only through a disabled feature is still counted.
#   * Registry and git dependencies are not in this tree, so nothing here
#     can date them — but `Cargo.lock` is root-level and always kept, so a
#     `cargo update` still moves the floor.
#   * The SUFFIX filter is `.rs` / `.msg` / the two manifest names, so a
#     non-source asset a crate embeds is not dated even when its crate IS
#     in the closure — `cerulion_accountd/src/team.rs` does
#     `include_str!("../assets/team.html")`, and editing that HTML moves
#     nothing. Pre-existing (it is the suffix filter, not the closure), and
#     named here so this list reads as what it is rather than as total.
#   * Source that reaches OUTSIDE its own crate directory would be missed:
#     a build script reading a corpus elsewhere, or an `include_str!` /
#     `include_bytes!` with a `../` escape. MEASURED on this tree rather
#     than assumed: the closure holds exactly one build script
#     (`native_ros2_messages/build.rs`, `rerun-if-changed=msg/` — its own),
#     and the two cross-crate `include_str!`s
#     (`cerulion_cli_engine/src/{auth,templates}.rs`) are both inside
#     `#[cfg(test)]`, so neither reaches the binary. Moving one of those
#     out of `#[cfg(test)]` — `templates.rs` reads from `test_fixtures/`,
#     which is NOT in the closure — would need this note revisited.
#   * Anything the scanner cannot model — an unreadable or unparseable
#     manifest, a dependency path escaping the checkout, no
#     `cerulion_cli/Cargo.toml` at all — makes the closure UNAVAILABLE,
#     and the walk falls back to the whole workspace, which is exactly the
#     pre-narrowing behaviour.
_CLI_SOURCE_EXCLUDED_TOPS = ("benches", "examples")
_CLI_SOURCE_NAMES = ("Cargo.toml", "Cargo.lock")
_CLI_SOURCE_SUFFIXES = (".rs", ".msg")

# A ONE-TUPLE, not the value: the answer may legitimately be None
# ("undatable"), and a bare None sentinel could not tell that apart from
# "not computed yet" — which would re-walk the tree on every call.
_CLI_SOURCE_FLOOR_CACHE: Optional[Tuple[Optional[int]]] = None

# How far ahead of the wall clock a source timestamp may sit before the
# tree's timestamps stop being evidence of anything. A commit made under a
# skewed clock, or a `touch -t` in the future, would otherwise push the
# floor past every binary that will ever be built here and make the check
# refuse EVERYTHING. Generous: ordinary skew is milliseconds, a genuinely
# future-dated tree is hours or days out.
CLI_SOURCE_FLOOR_FUTURE_GRACE_NS = 60 * 1_000_000_000


# The dependency tables a member manifest may declare. `[dependencies]`,
# `[dev-dependencies]`, `[build-dependencies]` and every
# `[target.'cfg(..)'.<one of those>]` share the same LAST header segment,
# which is what the scanner keys on.
_DEP_TABLE_NAMES = ("dependencies", "dev-dependencies", "build-dependencies")

# "There is a value here and this scanner cannot read it" — distinct from
# None, which means "there is no such key". The two must not collapse: the
# first has to WIDEN the walk (the pre-narrowing whole-workspace floor),
# the second is an ordinary registry dependency that is simply not in this
# tree. Collapsing them drops a real member silently, which is the one
# direction that mints a confident CLI_FRESH for a stale binary.
_UNREADABLE = object()


def _toml_strip_comment(line: str) -> str:
    """`line` without a trailing `#` comment. Quotes are respected: a `#`
    inside a string is data, and dropping the rest of the line there would
    silently truncate a dependency's path."""
    out: List[str] = []
    quote: Optional[str] = None
    i = 0
    while i < len(line):
        c = line[i]
        if quote is not None:
            # TOML literal strings ('...') process no escapes; basic
            # strings ("...") do.
            if quote == '"' and c == "\\":
                out.append(line[i:i + 2])
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in "\"'":
            quote = c
        elif c == "#":
            break
        out.append(c)
        i += 1
    return "".join(out)


def _toml_header_segments(header: str) -> List[str]:
    """`target.'cfg(unix)'.dependencies` -> ['target', 'cfg(unix)',
    'dependencies']. Split on dots OUTSIDE quotes, because a quoted
    segment may hold one."""
    segs: List[str] = []
    cur: List[str] = []
    quote: Optional[str] = None
    for c in header:
        if quote is not None:
            cur.append(c)
            if c == quote:
                quote = None
        elif c in "\"'":
            quote = c
            cur.append(c)
        elif c == ".":
            segs.append("".join(cur))
            cur = []
        else:
            cur.append(c)
    segs.append("".join(cur))
    return [s.strip().strip("'\"") for s in segs]


# `name = value`, `"quoted name" = value`, and the DOTTED form
# `name.workspace = true` / `name.path = "..."`, which is ordinary modern
# cargo and the dominant spelling in this repo's own `[package]` tables.
# Without the dotted arm a single such line made the whole manifest
# unreadable and switched the narrowing off — safe, but silently inert.
_TOML_KEY_RE = re.compile(
    r'^([A-Za-z0-9_\-]+|"[^"]*")((?:\.[A-Za-z0-9_\-]+)*)\s*=\s*(.*?)\s*$')


def _toml_value_is_closed(value: str) -> bool:
    """Has this TOML value finished, or does it continue on the next line?

    BOTH bracket kinds: an inline table `{ .. }` and an array `[ .. ]` may
    each span lines, and a reader that counted only braces turned an
    ordinary multi-line `features = [` into an unreadable manifest.

    String-blind, and the scope of that: a `}` or `]` inside a
    string value closes the join EARLY, so the tail of a real value is then
    read as ordinary lines. That tail cannot invent a dependency --
    `_TOML_KEY_RE` either matches it (a spurious entry with no `path`,
    skipped) or does not (None, widen) -- but it CAN hide the `path` of the
    entry it truncated. No manifest in this closure has a bracket inside a
    dependency-table string (checked); if one appears, the correct fix is a
    string-aware counter, not this docstring."""
    return (value.count("{") <= value.count("}")
            and value.count("[") <= value.count("]"))


def _manifest_declarations(text: str, want) -> "Optional[List[Tuple[str, str]]]":
    """Every `(name, raw TOML value text)` declared in a table whose header
    segments `want()` accepts, IN FILE ORDER, or None when the manifest
    holds a shape this deliberately small scanner does not model.

    A LIST, not a dict: one dependency may legitimately be declared in
    several tables (a `[target.'cfg(windows)'.dependencies]` entry beside a
    `[dependencies]` one), and a name-keyed dict silently keeps only the
    last — dropping whichever member the other declaration reached.

    Small ON PURPOSE: it reads exactly the two facts the closure needs
    (`path = "..."` and `workspace = true`) out of cargo's own manifest
    dialect and answers None rather than guessing at anything else. The
    caller turns None into "cannot narrow", which is the pre-narrowing
    whole-workspace walk. A full TOML parser is not in the stdlib before
    3.11 and this harness must run on older interpreters.

    Two spellings are folded to the inline shape: `[dep-table.NAME]`
    sub-tables (the whole body becomes NAME's value) and dotted keys
    (`NAME.path = "x"` becomes NAME's value `path = "x"`). Sub-table bodies
    are parsed with the SAME key regex as inline tables, so an unreadable
    line abandons the manifest on both paths.

    `want` is `Callable[[List[str]], bool]` over the header's segments; it
    is never called with an empty list."""
    decls: "List[Tuple[str, str]]" = []
    table: List[str] = []
    sub: Optional[int] = None            # index into decls for a sub-table
    pending: "Optional[Tuple[int, List[str]]]" = None
    for raw in text.splitlines():
        line = _toml_strip_comment(raw)
        if pending is not None:
            idx, parts = pending
            parts.append(line)
            joined = "".join(parts)
            if sub is not None:
                if _toml_value_is_closed(joined):
                    decls[idx] = (decls[idx][0], joined)
                    pending = None
                continue
            if _toml_value_is_closed(joined):
                decls[idx] = (decls[idx][0], joined)
                pending = None
            continue
        stripped = line.strip()
        m = re.match(r"^\[\[?([^\]]*)\]\]?$", stripped)
        if m is not None:
            table = _toml_header_segments(m.group(1))
            sub = None
            # A header is a SUB-TABLE only when it is not itself a wanted
            # table. `want()` can accept a header AND its parent, and
            # testing the parent first folded such a table into a single
            # entry so only its first `path =` was visible.
            if not want(table) and len(table) >= 2 and want(table[:-1]):
                sub = len(decls)
                decls.append((table[-1], ""))
            continue
        if not stripped:
            continue
        in_wanted = bool(table) and want(table)
        if sub is None and not in_wanted:
            continue
        m = _TOML_KEY_RE.match(stripped)
        if m is None:
            # A line inside a table the closure reads that this scanner
            # cannot parse. Guessing here is how a closure goes silently
            # under-inclusive, so the whole derivation is abandoned.
            return None
        name, dotted, value = m.group(1).strip('"'), m.group(2), m.group(3)
        if sub is not None:
            # Inside `[dep-table.NAME]`: these keys belong to NAME. The
            # SAME join as the inline form -- a multi-line `features = [`
            # array is ordinary cargo here, and handling it on only one of
            # the two spellings made `[dependencies.X]` widen where an
            # equivalent `X = { .. }` was accepted.
            decls[sub] = (decls[sub][0],
                          decls[sub][1] + f"{name}{dotted} = {value}\n")
            if not _toml_value_is_closed(value):
                pending = (sub, [decls[sub][1]])
            continue
        if dotted:
            # `serde.workspace = true` -> ("serde", "workspace = true")
            decls.append((name, f"{dotted.lstrip('.')} = {value}"))
            continue
        if not _toml_value_is_closed(value):
            decls.append((name, value))
            pending = (len(decls) - 1, [value])
        else:
            decls.append((name, value))
    if pending is not None:
        return None                      # an inline table that never closed
    return decls


# Both TOML string forms. Cargo accepts `path = '../x'` exactly as readily
# as `path = "../x"`, and reading only one of them dropped the dependency.
# TOML lets a bare key be quoted (`"path" = "x"` is the same key as
# `path = "x"`), so both spellings have to be recognised -- an
# unrecognised one reads as "no path key" and drops the dependency.
_TOML_PATH_KEY = r'(?:\bpath|"path")\s*=\s*'
_TOML_PATH_RE = re.compile(
    _TOML_PATH_KEY + r"""(?:"([^"]*)"(?!")|'([^']*)'(?!'))""")
_TOML_PATH_KEY_RE = re.compile(_TOML_PATH_KEY)


def _toml_path_value(value: str):
    """The `path = ...` of a dependency value: the string, None when there
    is no such key, or `_UNREADABLE` when there is one this scanner cannot
    read. THREE answers, because the last two have opposite consequences —
    see `_UNREADABLE`."""
    m = _TOML_PATH_RE.search(value)
    if m is not None:
        got = m.group(1) if m.group(1) is not None else m.group(2)
        # An EMPTY capture is not an empty path, it is a string form this
        # regex cannot read. `path = """a"""` is the case: the `(?!")`
        # lookahead rejects the match at offset 0, the engine retries one
        # character along, and `""` then matches with an empty body. So
        # BOTH guards are load-bearing -- the lookahead alone still yields
        # "", and returning that would resolve the dependency to its PARENT
        # crate's directory, a wrong answer wearing the shape of a right one.
        return got if got else _UNREADABLE
    return _UNREADABLE if _TOML_PATH_KEY_RE.search(value) else None


def _toml_inherits_workspace(value: str) -> bool:
    return re.search(r"\bworkspace\s*=\s*true\b", value) is not None


def _repo_relative_dir(base_rel: str, path: str) -> Optional[str]:
    """`path`, read relative to `base_rel`, as a repo-relative POSIX
    directory — or None when it is absolute or escapes the checkout.

    normpath, never resolve(): `git ls-files` names paths as they are
    spelled in the tree, and following a symlink out of it would produce a
    prefix no listed path can ever match."""
    if os.path.isabs(path):
        return None
    joined = os.path.normpath(os.path.join(base_rel, path) if base_rel
                              else path).replace(os.sep, "/")
    if joined == ".":
        return ""                        # the checkout root itself
    if joined == ".." or joined.startswith("../"):
        return None
    return joined


# The crate directory whose closure the freshness floor is derived from.
_CLI_CRATE_DIR = "crates/cerulion_cli"

# `[patch.<registry>]` and nothing deeper: `[patch.crates-io.NAME]` is a
# SUB-table of it. A prefix predicate accepted both depths, which made the
# sub-table spelling parse as ordinary keys and lose its path.
def _is_patch_table(segs: List[str]) -> bool:
    return len(segs) == 2 and segs[0] == "patch"


def _cli_member_closure() -> "Tuple[Optional[set], str]":
    """(the repo-relative directories of the workspace members
    `cerulion_cli` links, or None; a reason when it is None).

    Derived from the manifests, NEVER by invoking cargo: a breadth-first
    walk from `cerulion_cli/Cargo.toml` over every PATH dependency, with
    `workspace = true` entries resolved through the root manifest's
    `[workspace.dependencies]` table (which is how nearly every internal
    dependency in this tree is spelled). `[patch]` entries carrying a path
    join the closure too — they change what the CLI compiles.

    Over-inclusive by construction (features unresolved; dev- and
    build-dependencies counted) and CONSERVATIVE on failure: every shape
    the scanner cannot model returns None WITH A REASON, and the caller
    then walks the whole workspace exactly as it did before this narrowing
    existed — and says so once, so the feature cannot go silently inert.
    See the RESIDUALS note above `_CLI_SOURCE_EXCLUDED_TOPS`."""
    def _read(manifest: Path) -> Optional[str]:
        try:
            return manifest.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return None

    root_text = _read(REPO_ROOT / "Cargo.toml")
    if root_text is None:
        return None, "the root Cargo.toml could not be read"
    ws = _manifest_declarations(root_text,
                                lambda s: s == ["workspace", "dependencies"])
    patches = _manifest_declarations(root_text, _is_patch_table)
    if ws is None:
        return None, "the root [workspace.dependencies] table is unreadable"
    if patches is None:
        return None, "the root [patch] tables are unreadable"
    # name -> path, or _UNREADABLE. A name PRESENT with no path is an
    # inherited registry dependency: real, and not in this tree.
    #
    # MERGED per name, never last-wins: dotted keys make one entry arrive as
    # several declarations (`cerulion_core.path = "x"` then
    # `cerulion_core.version = "1"`), and keeping only the last would read
    # the path-less one and silently demote a real member -- along with
    # everything reachable only through it -- to "registry dependency".
    ws_text: Dict[str, str] = {}
    for name, value in ws:
        ws_text[name] = (ws_text.get(name, "") + "\n" + value).strip()
    ws_paths = {n: _toml_path_value(v) for n, v in ws_text.items()}

    if not (REPO_ROOT / _CLI_CRATE_DIR / "Cargo.toml").is_file():
        # Not this workspace (a fixture tree, a sparse checkout). Nothing
        # to narrow against.
        return None, f"{_CLI_CRATE_DIR}/Cargo.toml is not in this tree"

    def _member_tables(segs: List[str]) -> bool:
        # `[workspace.dependencies]` is the ROOT's table, read above; a
        # member manifest's own dependency tables are what this walks.
        return segs[-1] in _DEP_TABLE_NAMES and segs[:1] != ["workspace"]

    # Patch targets are WALKED, not merely added: a patched crate is
    # compiled from this tree, so ITS path dependencies change the CLI too.
    # Seeding the frontier is the whole fix -- `seen.add()` alone dropped
    # them.
    seen: set = set()
    patched_from: Dict[str, str] = {}
    frontier = [_CLI_CRATE_DIR]
    for name, value in patches:
        p = _toml_path_value(value)
        if p is _UNREADABLE:
            return None, (f"the root patches `{name}` with a `path` this "
                          f"scanner cannot read")
        if p is None:
            continue
        nxt = _repo_relative_dir("", p)
        if nxt is None:
            return None, f"the root patches `{name}` from outside this checkout"
        if nxt == "":
            return None, (f"the root patches `{name}` at the checkout root, "
                          f"which no closure can narrow")
        # Remember where it came from: once seeded, a patch target is
        # indistinguishable from a member, so a failure while WALKING it
        # would otherwise report a bare directory and send someone looking
        # through `[dependencies]` tables for a name only `[patch]` has.
        patched_from[nxt] = name
        frontier.append(nxt)
    while frontier:
        rel = frontier.pop()
        if rel in seen:
            # The dedup is not tidiness: this workspace contains a real
            # dev-dependency CYCLE (cerulion_core <-> native_ros2_messages),
            # and without it the walk does not terminate.
            continue
        seen.add(rel)
        via = (f" (patched in as `{patched_from[rel]}`)"
               if rel in patched_from else "")
        text = _read(REPO_ROOT / rel / "Cargo.toml")
        if text is None:
            return None, f"{rel}/Cargo.toml{via} could not be read"
        decls = _manifest_declarations(text, _member_tables)
        if decls is None:
            return None, (f"{rel}/Cargo.toml{via} holds a shape this "
                          f"scanner cannot read")
        for name, value in decls:
            path = _toml_path_value(value)
            if path is _UNREADABLE:
                return None, (f"{rel}/Cargo.toml declares `{name}` with a "
                              f"`path` this scanner cannot read")
            if path is not None:
                nxt = _repo_relative_dir(rel, path)
            elif _toml_inherits_workspace(value):
                if name not in ws_paths:
                    return None, (f"{rel}/Cargo.toml inherits `{name}` from "
                                  f"the workspace, which does not declare it")
                inherited = ws_paths[name]
                if inherited is _UNREADABLE:
                    return None, (f"the root declares `{name}` with a `path` "
                                  f"this scanner cannot read")
                if inherited is None:
                    continue             # inherited registry dep: not in-tree
                nxt = _repo_relative_dir("", inherited)
            else:
                continue                 # no `path` key: registry or git
            if nxt is None:
                return None, (f"{rel}/Cargo.toml declares `{name}` at a path "
                              f"outside this checkout")
            if nxt == "":
                # The checkout ROOT itself. Nothing is outside a closure
                # that contains it, so narrowing would be a no-op wearing
                # the shape of a narrowing -- and a `""` entry silently
                # matching every file is worse than saying so.
                return None, (f"{rel}/Cargo.toml declares `{name}` at the "
                              f"checkout root, which no closure can narrow")
            frontier.append(nxt)

    return seen, ""


def _within_cli_closure(rel: str, closure: "set") -> bool:
    """Is this repo-relative FILE inside one of the closure's directories?

    A root-level file (`Cargo.toml`, `Cargo.lock`) is always in: the lock
    is how a registry-dependency bump reaches this gate at all."""
    if "/" not in rel:
        return True
    return any(rel.startswith(d + "/") for d in closure)


# Printed at most once per process: the narrowing going unavailable is the
# safe direction, but a feature that can switch itself off in silence is
# one nobody can tell from a feature that is working.
_CLI_CLOSURE_NOTE_SHOWN = False


def _tracked_cli_sources() -> Optional[List[str]]:
    """Repo-relative paths of the files that can change the CLI, or None
    when git cannot answer.

    TRACKED and UNTRACKED are asked SEPARATELY, and that separation is the
    point. A module you have written but not yet `git add`ed is compiled
    (it must be, or the build fails), so an index-only walk would let every
    edit to it pass unseen and mint a confident `CLI_FRESH`. But the
    caller's totality rule — every listed path must `stat()` or the whole
    tree is undatable — turns any single unreadable entry into a gate that
    is OFF, and an untracked list is exactly where unreadable entries live:
    a dangling `.rs`-named symlink, an editor lock file, a path removed
    with `rm` rather than `git rm`. One of those anywhere outside the
    excluded trees made every candidate CLI_UNVERIFIABLE and degraded this
    whole check to a non-failing skip, for a reason having nothing to do
    with staleness.

    So an untracked entry that is not a real, readable file is DROPPED
    here — it was never a build input — while a TRACKED one is passed
    through to be counted and to make the tree undatable, which is the
    intended and separately-pinned behaviour (a sparse checkout really
    cannot be dated). `.gitignore` still applies to the untracked half, so
    `target/` stays out.

    Members `cerulion_cli` does not link are dropped as well, by
    `_cli_member_closure()`. When that derivation cannot answer, the
    walk keeps the WHOLE workspace — the pre-narrowing behaviour, and
    the safe direction."""
    global _CLI_CLOSURE_NOTE_SHOWN
    closure, why = _cli_member_closure()
    if closure is None and not _CLI_CLOSURE_NOTE_SHOWN:
        _CLI_CLOSURE_NOTE_SHOWN = True
        print(f"[note] the CLI freshness floor could not be narrowed to "
              f"`{_CLI_CRATE_DIR}`'s member closure ({why}) — dating this "
              f"checkout against the WHOLE workspace instead, which is "
              f"stricter, never looser", file=sys.stderr)

    def _ask(args: List[str]) -> Optional[List[str]]:
        try:
            r = subprocess.run(["git", "ls-files", "-z"] + args,
                               capture_output=True, cwd=REPO_ROOT, timeout=60)
        except (OSError, subprocess.SubprocessError):
            return None
        if r.returncode != 0:
            return None
        out: List[str] = []
        for raw in r.stdout.split(b"\0"):
            if not raw:
                continue
            rel = raw.decode("utf-8", "replace")
            if rel.split("/", 1)[0] in _CLI_SOURCE_EXCLUDED_TOPS:
                continue
            name = rel.rsplit("/", 1)[-1]
            if not (name in _CLI_SOURCE_NAMES
                    or name.endswith(_CLI_SOURCE_SUFFIXES)):
                continue
            if closure is not None and not _within_cli_closure(rel, closure):
                continue
            out.append(rel)
        return out

    tracked = _ask(["--cached"])
    untracked = _ask(["--others", "--exclude-standard"])
    if tracked is None or untracked is None:
        return None
    keep = list(tracked)
    for rel in untracked:
        path = REPO_ROOT / rel
        try:
            if path.is_file():
                keep.append(rel)
        except OSError:
            # Unreadable, so not a build input, so not this gate's business.
            continue
    return keep


def usable_source_floor(floor_ns: Optional[int], now_ns: int,
                        grace_ns: int = CLI_SOURCE_FLOOR_FUTURE_GRACE_NS
                        ) -> Optional[int]:
    """PURE: the floor to date a binary against, or None when the tree's
    timestamps cannot be evidence.

    A floor in the FUTURE is not "everything is stale" — it is "this tree's
    clocks disagree with this one", and answering `stale` there would
    refuse every binary anyone could build."""
    if floor_ns is None or floor_ns > now_ns + grace_ns:
        return None
    return floor_ns


def cerulion_source_floor_ns() -> Optional[int]:
    """The newest instant this checkout's CLI source could have changed, or
    None when that is unknowable.

    ONE term: the newest source MTIME. That is the make question, and it
    answers every shape that moves source — an edit, a `git checkout`
    (which rewrites the mtimes of the files it touches), a clone, a `git
    archive` export (every file stamped with the commit's time).

    HEAD's COMMITTER TIME was a second term and is deliberately GONE.
    `git commit`, `--amend`, rebase and merge all move `%ct` without
    touching a byte of source, so a `max()` against it marked every
    already-built binary stale for the length of an editing session —
    observed repeatedly while this change was being written, on a binary
    built from byte-identical code. Build, commit, run the gate is the
    ordinary order, so the harm was routine; the case the term defended
    (mtimes OLDER than the commit that contains them) needs a tool that
    restores recorded per-file times, and git records none.

    TOTALITY: the walk must read every file it was given. Skipping an
    unreadable one does not lower the running maximum, but it does stop the
    true maximum from being reached, and the effect on the operator is the
    same — a stale binary reported as a confident `CLI_FRESH`, which emits
    nothing at all because a fresh verdict carries no reason. A sparse
    checkout is UNDATABLE, which is the correct claim about a sparse
    checkout.

    Cached on success only; a None answer is not cached, so one transient
    `git` failure at start-up cannot leave a whole process undatable."""
    global _CLI_SOURCE_FLOOR_CACHE
    if _CLI_SOURCE_FLOOR_CACHE is not None:
        return _CLI_SOURCE_FLOOR_CACHE[0]
    # `or []` collapses "git could not answer" and "git named nothing" into
    # a walk that reads nothing, which the totality check below turns into
    # the same undatable answer as a file it could not stat.
    sources = _tracked_cli_sources() or []
    floor: Optional[int] = None
    read = 0
    for rel in sources:
        try:
            mtime = (REPO_ROOT / rel).stat().st_mtime_ns
        except OSError:
            continue
        read += 1
        if floor is None or mtime > floor:
            floor = mtime
    if read != len(sources) or read == 0:
        return None
    floor = usable_source_floor(floor, time.time_ns())
    if floor is None:
        return None
    _CLI_SOURCE_FLOOR_CACHE = (floor,)
    return floor


def classify_cerulion_binary(provenance: str,
                             binary_mtime_ns: Optional[int],
                             floor_ns: Optional[int]) -> Tuple[str, str]:
    """PURE verdict on one candidate binary: (verdict, reason).

    `reason` is empty exactly when the verdict is CLI_FRESH — every other
    verdict has to be able to tell an operator what is wrong with the
    binary it just declined."""
    if provenance == PROV_FOREIGN:
        return (CLI_FOREIGN,
                "it was built from a source tree that is not this checkout")
    if provenance == PROV_UNKNOWN:
        return (CLI_UNVERIFIABLE,
                "it sits under a shared CARGO_TARGET_DIR and carries no "
                "readable cargo depfile, so nothing here can say which "
                "checkout built it")
    if binary_mtime_ns is None:
        return (CLI_UNVERIFIABLE, "its modification time could not be read")
    if floor_ns is None:
        return (CLI_UNVERIFIABLE,
                "this checkout's source timestamps could not be read (is "
                "`git` on PATH, and is every source file readable?), so "
                "nothing here can date it")
    if binary_mtime_ns < floor_ns:
        # Ceiling, not floor division: a sub-second delta IS staleness (a
        # rebuild lands well inside a second), and reporting "0 s older"
        # would read as a rounding artefact on the one line an operator has
        # to act on.
        behind_s = -(-(floor_ns - binary_mtime_ns) // 1_000_000_000)
        return (CLI_STALE,
                f"it is {behind_s} s older than the newest source the CLI "
                f"is built from, so it was built from code this tree no "
                f"longer holds")
    return (CLI_FRESH, "")


def cerulion_cli_candidates() -> Tuple[List[Path], List[str]]:
    """Every USABLE `cerulion` this host offers, in a deterministic order
    (this tree's two build profiles, then PATH), plus one note for each
    path that LOOKS like a candidate and is not one.

    `is_file()` + `X_OK`, not `exists()`: a DIRECTORY named `cerulion`
    passes `exists()` and would be handed to subprocess as a binary, and a
    mode-000 file passes it too. The override refusal already asks
    `is_file()`, and two policies over one classifier must agree.

    The notes are not decoration. A dangling `target/release/cerulion`
    symlink drops out of `exists()` entirely, and the caller would then
    tell an operator to build a binary they can see sitting right there —
    the wrong remedy for the problem they have."""
    out: List[Path] = []
    unusable: List[str] = []
    target = cargo_target_dir()
    raw = [target / "release" / "cerulion", target / "debug" / "cerulion"]
    which = shutil.which("cerulion")
    if which:
        raw.append(Path(which))
    for c in raw:
        try:
            if c.is_file() and os.access(str(c), os.X_OK):
                out.append(c)
                continue
            # Order matters: a dangling symlink is neither a file nor a
            # directory nor `exists()`, so it has to be asked first.
            if c.is_symlink():
                unusable.append(f"{_short(c)}: a dangling symlink")
            elif c.is_dir():
                unusable.append(f"{_short(c)}: a directory, not a binary")
            elif c.exists():
                unusable.append(f"{_short(c)}: not executable")
        except OSError as e:
            unusable.append(f"{_short(c)}: cannot be examined ({e})")
    return (out, unusable)


def resolve_cerulion_cli() -> Tuple[Optional[Path], str]:
    """The `cerulion` binary this checkout's source describes, plus a note.

    Returns (path, note) when a binary is provably built from this checkout
    AND newer than its source — `note` is empty unless some OTHER candidate
    was unusable, which is worth saying even on a good day. Returns (None,
    refusal) otherwise: stale, foreign, undatable, or nothing to choose
    from.

    UNDATABLE is DECLINED here, deliberately. The `CERULION=` override
    refusal takes the opposite view of the same condition (it proceeds with
    a warning) because the costs differ: refusing here skips two arms,
    refusing there blocks a campaign. Accepting one here would restore
    exactly the earlier behaviour — first plausible binary wins,
    nothing asked about its provenance."""
    # An empty CARGO_TARGET_DIR is a REFUSAL from the readers below, and a
    # refusal raised through this function would take the WHOLE calling
    # process with it — for the parity checker that means ~40 later arms
    # never run and the failure reads as a crash rather than as the
    # configuration problem it is. This function already HAS a channel for
    # "no binary can be vouched for here": it returns (None, refusal). The
    # entry points keep the hard refusal (see the preflight in
    # refuse_foreign_cerulion_override) because there a run is about to
    # start; here two arms are skipped and everything else proceeds.
    # Ask the ONE question that refuses, and convert ONLY that refusal.
    # Wrapping the two calls below instead was correct today and correct
    # by accident: nothing else under them raises SystemExit right now, so
    # any hard refusal added to either later would be silently downgraded
    # to a soft "no usable binary" — the shape where a catch swallows a
    # refusal that should have been fatal. Asking directly keeps the scope
    # exact under later edits, and `cli_provenance` further down (a third
    # reader, outside the old try entirely) is covered by the same call.
    try:
        cargo_target_dir_setting()
    except SystemExit as e:
        return (None, str(e))
    floor = cerulion_source_floor_ns()
    candidates, unusable = cerulion_cli_candidates()
    aside = ("; also " + "; ".join(unusable)) if unusable else ""
    if not candidates:
        detail = (" — " + "; ".join(unusable)) if unusable else ""
        return (None,
                f"no usable `cerulion` binary in this checkout "
                f"(target/release, target/debug) or on PATH{detail}. Build "
                f"one with `{CLI_REBUILD_CMD}`")
    # Ranked by (mtime, -order): FRESHEST wins — the rule run_workspace.sh
    # already applies to the cdylibs — and candidate order breaks a tie
    # deterministically, so two profiles written in the same nanosecond do
    # not resolve differently from one run to the next.
    fresh: List[Tuple[int, int, Path]] = []
    declined: List[str] = []
    for order, candidate in enumerate(candidates):
        try:
            mtime: Optional[int] = candidate.stat().st_mtime_ns
        except OSError:
            mtime = None
        verdict, reason = classify_cerulion_binary(
            cli_provenance(candidate), mtime, floor)
        if verdict == CLI_FRESH:
            # mtime is not None on this arm: classify_cerulion_binary
            # answers CLI_UNVERIFIABLE when it could not be read.
            fresh.append((mtime if mtime is not None else -1, -order,
                          candidate))
        else:
            declined.append(f"{_short(candidate)}: {verdict} — {reason}")
    if fresh:
        # NOT .capitalize(): it lowercases every character after the
        # first, and `aside` carries filesystem paths — an operator would
        # be shown `/users/dev/...` for a path that only exists as
        # `/Users/Dev/...` on a case-sensitive filesystem.
        head = aside.lstrip("; ")
        return (max(fresh)[2], head[:1].upper() + head[1:] if head else "")
    return (None,
            "every `cerulion` on this host was declined (" +
            "; ".join(declined + unusable) +
            f") — rebuild this checkout's own with `{CLI_REBUILD_CMD}`")


def refuse_foreign_cerulion_override(subcommand: str) -> None:
    """Refuse a workspace-driving subcommand when an exported CERULION=
    points at a binary this checkout's source does not describe.

    run_workspace.sh rebuilds `target/release/cerulion` and then runs
    `$CERULION`, so the rebuild it performs protects the DEFAULT path and
    nothing else: an exported override survives it untouched, and every
    workspace row would then be measured through a binary built from other
    code while the run's manifest records this checkout's SHA.

    An in-tree, provably-fresh override is fine and stays supported — the
    knob is documented in run_workspace.sh's header. So is an UNDATABLE
    one, with a loud warning: refusing there would block a whole campaign
    because `git` hiccuped once. (The parity harness takes the opposite
    view of the same condition; see resolve_cerulion_cli.)

    SCOPE: this covers every path bench.py drives (the workspace,
    full and smoke subcommands, and run_workspace_leg itself). A DIRECT
    `bash workspace/run_workspace.sh <leg>` invocation does not go through
    bench.py and is not covered — carrying the check there too would mean a
    second implementation of this policy in shell, which is the drift the
    shared classifier exists to avoid."""
    # The CARGO_TARGET_DIR preflight rides here, ahead of everything else,
    # because this function is the ONE seam every workspace-driving path
    # already passes through — and because a run under an empty value
    # cannot be built at all, whether or not CERULION is exported.
    #
    # It is called UNCONDITIONALLY, above the `raw is None` early return
    # below: without a CERULION override the readers are never reached on
    # this path, so a refusal that rode only the override arm would leave
    # the ordinary `bench.py smoke` run sailing into a configuration cargo
    # refuses, and would make this file's own documentation false.
    cargo_target_dir_setting()
    raw = os.environ.get(CERULION_BIN_ENV)
    # EMPTY means unset, because that is exactly what the consumer means by
    # it: run_workspace.sh reads `${CERULION:-<default>}`, and `:-`
    # substitutes the default for a NULL value as well as an absent one.
    # Refusing `CERULION=` would refuse a run the runner performs correctly
    # with its own build.
    if raw is None or raw == "":
        return
    # WHITESPACE-ONLY is NOT null, and the difference is the whole point of
    # this guard. `${CERULION:-...}` substitutes nothing for `'   '` — it
    # passes the spaces straight through — so the runner reaches
    # `[ ! -x "   " ]`, aborts, and does so only AFTER rebuilding the CLI
    # and after cleanup_iceoryx() has unlinked SHM shared with every other
    # tenant on the machine. Treating it as unset here (an earlier revision
    # used `raw.strip()`) approved precisely the run this refusal exists to
    # stop; verified against the real shell, not inferred.
    if not raw.strip():
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} is only whitespace — refusing to "
            f"run '{subcommand}'. The shell does NOT treat that as unset: "
            f"`${{{CERULION_BIN_ENV}:-<default>}}` substitutes a default "
            f"only for an empty value, so run_workspace.sh would take the "
            f"spaces literally and abort on `[ ! -x ]` after its rebuild "
            f"and after the SHM sweep. Unset it properly (`unset "
            f"{CERULION_BIN_ENV}`) to use this checkout's own build.")
    if raw.rstrip().endswith(("/", os.sep)):
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} ends with a path separator — "
            f"refusing to run '{subcommand}'. The shell cannot exec that "
            f"(ENOTDIR) while pathlib normalises the separator away, so "
            f"this check would vouch for a path run_workspace.sh cannot "
            f"run. Drop the trailing separator, or unset it (`unset "
            f"{CERULION_BIN_ENV}`).")
    candidate = Path(raw)
    # A RELATIVE override is ambiguous by construction: this process and
    # run_workspace.sh do not share a working directory (the runner cds to
    # its own script dir), so the path checked here and the path executed
    # there can be two different files — and then the check has approved
    # something else. Refuse rather than guess which one it is.
    if not candidate.is_absolute():
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} is a relative path — refusing to "
            f"run '{subcommand}'. run_workspace.sh changes directory before "
            f"it runs ${CERULION_BIN_ENV}, so a relative override names "
            f"one file here and possibly another there, and this check "
            f"would then have vouched for the wrong binary. Give an "
            f"absolute path, or unset it (`unset {CERULION_BIN_ENV}`) to "
            f"use this checkout's own build.")
    # The one override the runner's own rebuild DOES protect, and it is
    # asked HERE — above the existence and executability checks, not below
    # them. Naming `$REPO_ROOT/target/release/cerulion` (a CI wrapper, a
    # shell rc) is the same ask as not setting it at all, and
    # run_workspace.sh BUILDS that exact path before running it. Asked
    # after `is_file()`, the exemption was unreachable in the case it
    # exists for: on a fresh checkout the binary does not exist yet, so
    # the export was refused as "not a file" while the identical run with
    # CERULION unset succeeded. The literal path from the runner's own
    # `${CERULION:-...}`, NOT cargo_target_dir() — under a shared
    # CARGO_TARGET_DIR the runner still defaults to the former, and
    # exempting a path it will not run would be exempting the wrong file.
    #
    # RAW **or** RESOLVED, because a lexical comparison alone misses the
    # same file named through a symlinked checkout prefix — and then the
    # `is_file()` check below refuses an unbuilt default that the runner
    # would have built, which is the very false refusal hoisting this
    # block was meant to end. (An earlier revision dropped `.resolve()`
    # here on the grounds that resolving a path that does not exist is
    # not meaningful. That was wrong: `Path.resolve()` has been
    # non-strict since 3.6, so it resolves the symlinked PREFIX and keeps
    # the missing tail, which is exactly what this needs.) Raw is kept as
    # well as resolved, so the exemption still holds where resolution
    # cannot run at all.
    # ONLY when the runner will actually build the path it runs. The whole
    # premise here is "run_workspace.sh rebuilds exactly this file, so
    # naming it is the same ask as not setting it" — and under a custom
    # CARGO_TARGET_DIR that is FALSE: the runner's `cargo build -p
    # cerulion_cli --release` writes to the custom dir while its
    # `${CERULION:-$REPO_ROOT/target/release/cerulion}` default still names
    # the repo one, so the binary it runs is whatever was left there,
    # rebuilt by nothing. Exempting it would measure a campaign through
    # stale code (or fail on a missing file) with the manifest recording
    # this checkout.
    #
    # With the variable set, this override gets no exemption and falls
    # through to the ordinary checks, which is right in all three shapes:
    # absent -> refused early instead of by the runner after its rebuild
    # and the SHM sweep; stale -> refused, since nothing will rebuild it;
    # fresh -> accepted, and it really is the binary that runs.
    #
    # (That the runner's own default is unbuildable under a custom target
    # dir is a run_workspace.sh limitation this guard does not try to fix;
    # it just stops vouching for it.)
    default_cli = REPO_ROOT / "target" / "release" / "cerulion"
    runner_builds_default = _runner_rebuilds(default_cli)
    if runner_builds_default and candidate == default_cli:
        return
    try:
        if not runner_builds_default:
            raise _NotTheDefault
        # Canonicalise the DIRECTORY and keep the final NAME, on the
        # default side. The question this arm must ask is "is the candidate
        # the runner's default path, spelled differently" — and the two
        # spellings that matter pull in opposite directions:
        #
        #   a symlinked PREFIX (`/var` -> `/private/var`, `/home` ->
        #   `/export/home`) means both sides need their DIRECTORIES
        #   canonicalised, or the same file compares unequal;
        #
        #   a symlinked DEFAULT BINARY (`target/release/cerulion` itself a
        #   link to a foreign build) means the final component must NOT be
        #   followed — `candidate.resolve() == default_cli.resolve()` then
        #   asks only "do these end at the same file", which ANY other
        #   symlink to that foreign binary also satisfies, exempting it
        #   from every provenance check below.
        #
        # Resolving the parent and re-attaching the name answers the first
        # and refuses the second.
        default_canonical = default_cli.parent.resolve() / default_cli.name
        if candidate.resolve() == default_canonical:
            return
    except _NotTheDefault:
        pass
    except (OSError, RuntimeError):
        # BOTH, and RuntimeError is not defensive padding: on a symlink
        # loop CPython 3.10's `Path.resolve()` re-raises ELOOP as
        # `RuntimeError("Symlink loop from ...")`, which an OSError-only
        # handler would let escape as an unhandled crash from a guard
        # whose whole job is to refuse cleanly. (3.13 stopped raising for
        # loops under strict=False, so the behaviour is version-dependent
        # and the outcome — exempt or refuse, never crash — is what the
        # oracle asserts.)
        #
        # A resolution that cannot complete is not evidence that this
        # ISN'T the default; it is no evidence either way, so fall
        # through to the ordinary checks rather than exempt on a guess.
        # The RAW comparison above has already answered the exact-path
        # case, which is what keeps the exemption working here.
        pass
    if not candidate.is_file():
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} is set in the environment but is "
            f"not a file — refusing to run '{subcommand}', which would hand "
            f"it to run_workspace.sh. Unset it (`unset "
            f"{CERULION_BIN_ENV}`) to use this checkout's own build.")
    # The runner DELETES this one out from under itself. run_workspace.sh
    # does `rm -rf "$SCRIPT_DIR/target/debug"` (its freshest-wins guard, so
    # a stale debug cdylib cannot shadow release) where SCRIPT_DIR is the
    # bench WORKSPACE — `benches/latency/workspace` — and then runs
    # $CERULION. An override living in that directory is therefore removed
    # between the check and the run, and the runner fails its own `-x`
    # test after the rebuild and after the SHM sweep.
    #
    # Normally unreachable: that path is outside REPO_ROOT/target, so it
    # classifies FOREIGN and is refused below anyway. It becomes reachable
    # only if CARGO_TARGET_DIR points at the bench workspace's own target
    # dir, which makes the depfile the arbiter and can answer IN_TREE. A
    # narrow shape, but a refusal that names the real reason beats one
    # that calls it foreign, and beats the runner discovering it later.
    doomed = WORKSPACE_DIR / "target" / "debug"
    if _deleted_by_the_runner(candidate, doomed):
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} lives under {_short(doomed)}, "
            f"which run_workspace.sh DELETES (`rm -rf`) before it runs "
            f"${CERULION_BIN_ENV} — its freshest-wins guard against a "
            f"stale debug cdylib. The override would be removed between "
            f"this check and the run. Point it at a release build, or "
            f"unset it (`unset {CERULION_BIN_ENV}`) to use this "
            f"checkout's own.")
    if not os.access(str(candidate), os.X_OK):
        # run_workspace.sh does check `[ ! -x "$CERULION" ]`, but only
        # after ITS OWN rebuild — and this file's cleanup_iceoryx() has by
        # then unlinked SHM shared with every other tenant on the machine,
        # which is the ordering the wiring arm requires this refusal to
        # come before.
        raise SystemExit(
            f"{CERULION_BIN_ENV}={raw!r} is not executable — refusing to "
            f"run '{subcommand}' here rather than letting run_workspace.sh "
            f"discover it after its rebuild and after the SHM sweep. "
            f"`chmod +x` it, or unset it (`unset {CERULION_BIN_ENV}`).")
    try:
        mtime: Optional[int] = candidate.stat().st_mtime_ns
    except OSError:
        mtime = None
    verdict, reason = classify_cerulion_binary(
        cli_provenance(candidate), mtime, cerulion_source_floor_ns())
    if verdict in (CLI_FRESH, CLI_UNVERIFIABLE):
        if verdict == CLI_UNVERIFIABLE:
            print(f"  ! {CERULION_BIN_ENV}={raw} cannot be dated against "
                  f"this checkout ({reason}) — proceeding UNVERIFIED",
                  file=sys.stderr)
        return
    raise SystemExit(
        f"{CERULION_BIN_ENV}={raw!r} is set in the environment and {reason} "
        f"— refusing to run '{subcommand}'. run_workspace.sh rebuilds "
        f"target/release/cerulion and then runs ${CERULION_BIN_ENV}, so the "
        f"rebuild does not protect an override: every workspace row would "
        f"be measured through that binary while the manifest records THIS "
        f"checkout. Unset it (`unset {CERULION_BIN_ENV}`), or point it at a "
        f"fresh build of this tree (`{CLI_REBUILD_CMD}`).")

def resolve_run_dir(args: argparse.Namespace) -> Path:
    """Run directory: --run-dir override, else
    results/<machine-hash>-<date>-<variant>/.

    Date (not timestamp) so a multi-invocation campaign on one day
    accumulates into ONE run directory. The pacing VARIANT is part of the
    key: a `--variant backtoback` run after a
    quiescent sweep on the same day lands in its own dir and can never
    silently destroy the quiescent raws. Raw samples land under rep<k>/
    (see rep_dir_of); a LEGACY rep-less dir stays readable by compile-csv /
    plots via an explicit --run-dir."""
    if getattr(args, "run_dir", None):
        return Path(args.run_dir).resolve()
    try:
        mh = compute_machine_hash()
    except SmokeSetupError as e:
        raise SystemExit(
            f"cannot resolve the run directory: {e}\n"
            f"Pass --run-dir explicitly to override.") from e
    # The READER subcommands (compile-csv, plots) leave --variant unset by
    # default so an explicit --run-dir can take the run's RECORDED variant
    # (see cmd_plots). With no --run-dir there is nothing to read it from,
    # so the default run dir is the primary variant's.
    # `args.variant`, not `getattr(.., None)`: every caller is either a
    # real argparse namespace or one of cmd_full's keyword-built ones, and
    # all of them define `variant`. A getattr default would silently absorb
    # a typo into the primary variant instead of raising. An AttributeError
    # is the right answer to a missing field; None is the right answer to
    # "unstated". (cmd_full's own Namespaces always set `run_dir`, so they
    # return above this line and never reach it; all five of them do set
    # `variant`, so there is no live AttributeError — the point is that a
    # future one that did not would raise rather than resolve silently.)
    variant = args.variant or PACING_VARIANTS[0]
    return RESULTS_ROOT / f"{mh}-{time.strftime('%Y-%m-%d')}-{variant}"

def rep_dir_of(run_dir: Path, rep: int) -> Path:
    """rep<k>/ under the run dir — the A1 rep dimension. Reps NEVER
    overwrite each other; re-running the SAME rep index re-measures that rep
    (the per-prefix stale-mix guard clears its old .bins loudly)."""
    if rep < 1:
        raise SystemExit(f"--rep must be >= 1, got {rep}")
    return run_dir / f"rep{rep}"

def raw_dir_of(run_dir: Path) -> Path:
    return run_dir / "raw"

# ============================================================================
# Environment provenance: cpu governor / turbo state
# ============================================================================

def read_cpu_governor_state() -> Tuple[str, str]:
    """(governor, turbo_boost) — read from sysfs on Linux; 'unknown' where
    unreadable (macOS has no cpufreq sysfs). turbo_boost ∈ {on, off,
    unknown}: intel_pstate exposes no_turbo (1 = turbo OFF); the generic
    cpufreq driver exposes boost (1 = boost ON)."""
    governor = "unknown"
    boost = "unknown"
    if not IS_LINUX:
        return governor, boost
    try:
        governor = Path("/sys/devices/system/cpu/cpu0/cpufreq/"
                        "scaling_governor").read_text().strip()
    except OSError:
        pass
    try:
        no_turbo = Path("/sys/devices/system/cpu/intel_pstate/"
                        "no_turbo").read_text().strip()
        boost = "off" if no_turbo == "1" else "on"
    except OSError:
        try:
            b = Path("/sys/devices/system/cpu/cpufreq/boost").read_text().strip()
            boost = "on" if b == "1" else "off"
        except OSError:
            pass
    return governor, boost

def print_governor_state_loudly() -> Tuple[str, str]:
    """Print the governor + turbo/boost state at every data-producing
    subcommand start (and record it in run.json). A citable campaign
    requires the `performance` governor on the bench machine (README
    prerequisites); anything else gets a loud warning, never a refusal —
    macOS / unknown states are functional-only."""
    governor, boost = read_cpu_governor_state()
    print(f"[env] cpu governor: {governor}; turbo/boost: {boost}")
    if IS_LINUX and governor != "performance":
        print(f"  ! cpu governor is {governor!r}, not 'performance' — "
              f"DVFS can move small-payload p50s by double digits; set the "
              f"performance governor before a citable sweep "
              f"(recorded in run.json either way)", file=sys.stderr)
    return governor, boost

# ============================================================================
# Run manifest (run.json)
# ============================================================================

MANIFEST_SCHEMA_VERSION = 1
MANIFEST_NAME = "run.json"

def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")

def manifest_variant_of(run_dir: Path) -> Optional[str]:
    """The pacing variant `<run_dir>/run.json` records, or None.

    Deliberately the SAME acceptance rule as the reader it guards —
    `compile_csv.read_manifest_variant`: the top-level `variant` key, and
    only if it is a KNOWN variant. Not a superset. A guard that accepted
    more than its reader would refuse runs the reader would have compiled
    (a typo'd `"quiescant"` reads as None there and would have read as a
    contradiction here — and the refusal would have told the operator to
    pass a value argparse rejects), and scanning `invocations[]`, which
    plot.py does and compile_csv does not, would refuse on evidence the
    reader structurally ignores. Two copies of one rule is already the
    hazard; two copies of two rules is worse.

    None for absent, unreadable, malformed, or not-a-known-variant — every
    one of those means "the manifest cannot contradict you", which is the
    only question the caller asks. The copy exists because the import runs
    one way (compile_csv imports bench), so bench cannot borrow it.
    """
    path = run_dir / MANIFEST_NAME
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    recorded = data.get("variant") if isinstance(data, dict) else None
    return recorded if recorded in PACING_VARIANTS else None


class RunManifest:
    """Merge-written per-run provenance manifest at
    <run-dir>/run.json (the run dir = the variant-keyed
    results/<mh>-<date>-<variant>/ root, shared by all reps).

    Each data-producing bench.py invocation appends ONE invocation record:
    git sha + dirty flag, machine hash, uname, cpu governor + turbo/boost,
    the argv, per-cell start/end timestamps + outcomes, the skip inventory
    with reasons, and the docker image ids (+ their embedded repo-sha
    labels) actually used. This is the artifact that makes the pacing
    variant, §10 same-window claims, and comparison image provenance
    verifiable offline — pre-A5 nothing wrote a manifest at all and
    machine_hash.sh's header cited a run.json that did not exist.

    Refusals happen at INIT (cheap, before any cell runs): a corrupt
    existing manifest, or one whose variant differs from this invocation's
    (only reachable via an explicit --run-dir override, since the variant
    is in the default dir key — exactly the cross-variant mislabeling
    hazard of synthesis #7). A failed WRITE at finalize() warns loudly but
    never voids measured data."""

    def __init__(self, run_dir: Path, subcommand: str, variant: str,
                 rep: Optional[int]) -> None:
        self.path = run_dir / MANIFEST_NAME
        existing = self._read_existing()
        if existing is not None:
            prior = existing.get("variant")
            if prior != variant:
                raise SystemExit(
                    f"{self.path}: this run dir's manifest records variant "
                    f"{prior!r} but this invocation is --variant {variant} — "
                    f"refusing to mix pacing variants in one run dir (a "
                    f"mislabeled saturation number in a published figure is "
                    f"a data-integrity story). Use the default variant-keyed "
                    f"run dir, or a different --run-dir.")
        try:
            mh = compute_machine_hash()
        except SmokeSetupError as e:
            mh = "unknown"
            print(f"  ! run manifest: machine hash unavailable ({e}) — "
                  f"recording 'unknown'", file=sys.stderr)
        governor, boost = read_cpu_governor_state()
        self.entry: dict = {
            "subcommand": subcommand,
            "variant": variant,
            "rep": rep,
            "argv": list(sys.argv),
            "git_sha": _git_sha(),
            "git_dirty": _git_dirty(),
            "machine_hash": mh,
            "uname": " ".join(platform.uname()),
            "platform": PLATFORM_LABEL,
            "cpu_governor": governor,
            "turbo_boost": boost,
            # C-state posture (CER_BENCH_DMA_LOCK): "tuned" = cap held /
            # device passed; "stock" = no bench-side tuning. Postures live
            # in SEPARATE run dirs by operator discipline; this key is what
            # makes a mislabeled dir auditable offline.
            "dma_lock_posture": resolved_dma_posture(),
            "started_at": _now_iso(),
            "ended_at": None,
            "cells": [],
            "skips": [],
            "docker_images": {},
        }

    def _read_existing(self) -> Optional[dict]:
        if not self.path.exists():
            return None
        try:
            data = json.loads(self.path.read_text())
        except (OSError, ValueError) as e:
            raise SystemExit(
                f"{self.path}: existing run manifest is unreadable or "
                f"corrupt ({e}) — fix or move it before running; provenance "
                f"is never silently clobbered.")
        if not isinstance(data, dict):
            raise SystemExit(
                f"{self.path}: existing run manifest is not a JSON object — "
                f"fix or move it before running.")
        return data

    def record_cell(self, cell: str, payload: Optional[int], outcome: str,
                    started_at: str) -> None:
        self.entry["cells"].append({
            "cell": cell,
            "payload": payload,   # None = the process sweeps all sizes itself
            "outcome": outcome,   # ok | fail | skip
            "started_at": started_at,
            "ended_at": _now_iso(),
        })

    def record_skip(self, scope: str, reason: str) -> None:
        self.entry["skips"].append({"scope": scope, "reason": reason})

    def record_docker_image(self, tag: str) -> None:
        self.entry["docker_images"][tag] = docker_image_provenance(tag)

    def finalize(self) -> None:
        self.entry["ended_at"] = _now_iso()
        try:
            data = self._read_existing()
        except SystemExit as e:
            print(f"  ! run manifest became unreadable mid-run: {e} — this "
                  f"invocation's provenance record is LOST (measured data "
                  f"is intact)", file=sys.stderr)
            return
        if data is None:
            data = {"schema_version": MANIFEST_SCHEMA_VERSION,
                    "variant": self.entry["variant"],
                    "machine_hash": self.entry["machine_hash"],
                    "invocations": []}
        data.setdefault("invocations", []).append(self.entry)
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_name(self.path.name + ".tmp")
            tmp.write_text(json.dumps(data, indent=2) + "\n")
            tmp.replace(self.path)
        except OSError as e:
            print(f"  ! failed to write run manifest {self.path}: {e} — "
                  f"measured data is intact; this invocation's provenance "
                  f"record is LOST unless re-run", file=sys.stderr)

# ============================================================================
# Ambient-env refusal
# ============================================================================

# True only while _run_smoke_cells drives cells (it sets the smoke override
# in os.environ deliberately); lets refuse_ambient_sample_overrides distinguish
# smoke-owned env from an ambient leak.
_SMOKE_ACTIVE = False

def refuse_ambient_sample_overrides(subcommand: str,
                                    variant: Optional[str],
                                    *, produces: bool = True) -> None:
    """Refuse a full-fidelity sweep with smoke/sample overrides exported in
    the ambient environment.

    Spawned bench processes inherit the environment, so an exported
    CER_BENCH_SMOKE_N (quiescent) or CER_BENCH_TARGET_SAMPLES/CER_BENCH_WARMUP
    (backtoback) would silently shrink every cell to smoke-grade sample
    counts and the sweep artifact would be statistically meaningless with no
    visible signal. The smoke subcommand injects the overrides itself,
    per-subprocess, and never needs them exported.

    `variant = None` means the run's pacing variant is unknown; it takes
    the strictest reading and refuses the backtoback pair too.

    `produces=False` is for a CONSUMER of a finished run (compile-csv): no
    cells run, so the harm is not a thin sweep but expected sample counts
    read from the post-processing shell instead of from the run's own
    schedule. Same refusal, with its own reason."""
    if _SMOKE_ACTIVE:
        return
    harm = ("the override is inherited by every spawned bench process and "
            "would silently produce a low-fidelity sweep"
            if produces else
            "the expected sample counts would be read from this shell "
            "instead of from the run's own schedule, so a valid run could "
            "be rejected — or a short one accepted")
    if "CER_BENCH_SMOKE_N" in os.environ:
        raise SystemExit(
            f"CER_BENCH_SMOKE_N is set in the environment — refusing to run "
            f"'{subcommand}' ({harm}). Unset it: `unset CER_BENCH_SMOKE_N`. "
            f"The smoke subcommand does not need it exported.")
    if variant in ("backtoback", None):
        leaked = [k for k in ("CER_BENCH_TARGET_SAMPLES", "CER_BENCH_WARMUP")
                  if k in os.environ]
        if leaked:
            scope = (f"backtoback '{subcommand}'" if variant == "backtoback"
                     else f"'{subcommand}' against a run whose pacing variant "
                          f"is unrecoverable")
            raise SystemExit(
                f"{'/'.join(leaked)} is set in the environment — refusing to "
                f"run {scope} with overridden sample counts ({harm}). "
                f"Unset with: `unset {' '.join(leaked)}`.")

# ============================================================================
# Cargo build (native crate)
# ============================================================================

_BUILT: set = set()

def ensure_built(bench_dir: Path) -> None:
    """cargo build --release, once per dir per process. ALWAYS rebuild before
    measuring (a stale binary once voided a whole aarch64
    result set); the once-per-process memo only dedups within a single
    bench.py invocation."""
    if str(bench_dir) in _BUILT:
        return
    if shutil.which("cargo") is None:
        raise SystemExit(
            "cargo is not on PATH — the bench binaries are built from source "
            "on every run (a stale binary voided a whole result set once). "
            "Install a Rust toolchain (https://rustup.rs) and re-run.")
    print(f"[build] cargo build --release in {_short(bench_dir)}")
    try:
        rc = subprocess.run(["cargo", "build", "--release"],
                            cwd=bench_dir).returncode
    except OSError as e:
        raise SystemExit(f"cargo build could not be started in {bench_dir}: "
                         f"{e}") from e
    if rc != 0:
        raise SystemExit(f"cargo build failed in {bench_dir}")
    _BUILT.add(str(bench_dir))

# ============================================================================
# Subcommand: native
# ============================================================================

def native_timeout_s(variant: str) -> int:
    """Watchdog budget for ONE native bench process (which sweeps every
    payload size in a single invocation — the budget bounds the whole
    internal sweep, not one size).

    CER_BENCH_NATIVE_TIMEOUT_S overrides (strict integer parse — garbage is
    a loud refusal, never a silent default). Defaults: 1800 s quiescent
    (the tail-resolved schedule sums to ~12.6 min of pacing — ~70 s at 4 MiB
    + ~205 s at 16 MiB), 900 s backtoback, scaled down
    to 300 s under the smoke gate's ~1000-sample cells."""
    raw = os.environ.get("CER_BENCH_NATIVE_TIMEOUT_S")
    if raw is not None:
        try:
            t = int(raw)
        except ValueError:
            raise SystemExit(
                f"CER_BENCH_NATIVE_TIMEOUT_S must be an integer number of "
                f"seconds, got {raw!r}") from None
        if t <= 0:
            raise SystemExit(
                f"CER_BENCH_NATIVE_TIMEOUT_S must be > 0, got {t}")
        return t
    if _SMOKE_ACTIVE:
        return 300
    # fixed100's nominal sweep is short (10 sizes x ~21 s at the target),
    # but a laddering size re-runs at up to 4 rungs (~378 s worst-case at
    # 16 MiB) — the quiescent budget covers the pathological all-sizes-
    # ladder case with headroom.
    return 900 if variant == "backtoback" else 1800

def run_native_bench(bench: NativeBench, chrt: int, variant: str,
                     raw_dir: Path, log_dir: Path) -> bool:
    """Run one native bench binary (it sweeps every payload size of the
    active sweep internally — the pinned 10, or the ambient
    CER_BENCH_PAYLOAD_SIZES restriction, which the binary reads itself).
    Returns True on success: rc == 0, no watchdog kill, and the EXACT
    per-size .bin set present (one file per swept payload size — a
    stale-mix or partial sweep must never pass on `some bins exist`)."""
    raw_name = f"{bench.raw_prefix}_chrt{chrt}"
    sweep = ambient_payload_restriction() or PAYLOAD_SIZES
    bin_path = NATIVE_BIN_DIR / bench.bin_name
    if not bin_path.exists():
        print(f"  ! FAIL {bench.bin_name} (binary missing at "
              f"{_short(bin_path)} — cargo build first)")
        return False

    cleanup_iceoryx()
    kill_stragglers()

    env = os.environ.copy()
    env["CER_BENCH_RAW_DUMP_DIR"] = str(raw_dir)
    env["CER_BENCH_RAW_NAME"] = raw_name
    env[PACING_ENV] = variant
    if dma_lock_enabled():
        env["CERULION_CPU_DMA_LOCK"] = "1"
    # else STOCK posture: never inject the cap var — the bin itself skips
    # the C-state lock (acquire_dma_lock reads the inherited
    # CER_BENCH_DMA_LOCK=0) and says so on its stderr, which lands in the
    # per-cell log. A contradictory ambient CERULION_CPU_DMA_LOCK export is
    # refused twice over: by dma_lock_enabled() here, and (since the
    # contract, not this script, is what a direct run must honour) by
    # acquire_dma_lock in the bin itself.

    cmd: List[str] = []
    if chrt == 1:
        prefix = find_chrt_prefix()
        if prefix is None:
            print(f"  ! skip {bench.bin_name} chrt=on — chrt not available")
            return False
        cmd.extend(prefix)
    cmd.append(str(bin_path))

    # Stale-mix guard: clear this prefix's SWEPT-size .bins from any prior
    # run immediately before spawning (after every skip-return above, so a
    # skipped cell never destroys a prior run's data), so the exact-set
    # gate below can only ever see files this invocation wrote — a stale
    # 16MB .bin beside a fresh partial sweep is exactly the mix that
    # voided the aarch64 result set. The fixed100 .rate sidecars ride the
    # same guard: a stale achieved-rate (or did_not_sustain) label from a
    # prior run must never describe this run's rows. Under an ambient
    # restriction ONLY the restricted sizes are cleared: the other sizes'
    # .bins are a prior full sweep the restricted run deliberately merges
    # over.
    stale = [p for s in sweep
             for p in (raw_dir / f"{raw_name}_{s}.bin",
                       rate_sidecar_path(raw_dir, raw_name, s),
                       # usage sidecars ride the same guard: a prior usage
                       # run's .usage.csv must never sit beside this run's
                       # fresh .bins claiming to describe them.
                       usage_sidecar_path(raw_dir, raw_name, s))
             if p.exists()]
    for f in stale:
        f.unlink()
    if stale:
        print(f"    cleared {len(stale)} stale {raw_name}_* .bin/.rate "
              f"file(s) from a prior run")

    log_dir.mkdir(parents=True, exist_ok=True)
    timeout_s = native_timeout_s(variant)

    def _invoke(inv_env: Dict[str, str], log_path: Path,
                usage_out: Optional[Path]) -> Tuple[Optional[int], bool]:
        """One bench-binary invocation under the watchdog. Returns
        (rc_or_None, timed_out). Under usage mode a sampler descends
        from the spawned process (which owns everything the bench forks
        — the sudo/chrt wrapper, the zenoh pong)."""
        inv_timed_out = False
        with log_path.open("wb") as logf:
            # start_new_session=True (POSIX) puts the bench — and everything
            # it spawns: the sudo/chrt wrapper, the zenoh pong subprocess —
            # into its own process group so an expired watchdog kills the
            # whole tree, not just the leader.
            proc = subprocess.Popen(cmd, env=inv_env, stdout=subprocess.DEVNULL,
                                    stderr=logf,
                                    start_new_session=not IS_WINDOWS)
            sampler = (start_usage_sampler(usage_out, log_dir,
                                           parent_pid=proc.pid)
                       if usage_out is not None else None)
            try:
                rc: Optional[int] = proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                inv_timed_out = True
                print(f"    ! TIMEOUT after {timeout_s}s — killing the bench "
                      f"process group (override: CER_BENCH_NATIVE_TIMEOUT_S)",
                      file=sys.stderr)
                if IS_WINDOWS:
                    proc.kill()
                else:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError) as e:
                        # A sudo-wrapped chrt run is a root-owned group; an
                        # unprivileged killpg gets EPERM. Escalate best-effort.
                        print(f"    ! killpg failed ({e})", file=sys.stderr)
                        # Same guard, same reason as find_chrt_prefix: this
                        # is best-effort cleanup, and a missing sudo must not
                        # turn a teardown into a crash that strands the very
                        # process group it is trying to reap.
                        if shutil.which("sudo") is not None:
                            print(f"      trying `sudo -n kill -9 -- "
                                  f"-{proc.pid}`", file=sys.stderr)
                            try:
                                subprocess.run(
                                    ["sudo", "-n", "kill", "-9", "--",
                                     f"-{proc.pid}"], capture_output=True)
                            except OSError as e2:
                                print(f"    ! sudo kill failed ({e2}) — the "
                                      f"group may survive", file=sys.stderr)
                        else:
                            print("    ! no sudo on PATH — cannot escalate; "
                                  "the group may survive", file=sys.stderr)
                try:
                    rc = proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    print("    ! bench process still alive 30s after the kill "
                          "— abandoning the handle; the next cell's straggler "
                          "sweep will retry", file=sys.stderr)
                    rc = None
                # Best-effort: the zenoh pong is a subprocess of the main
                # binary; make sure a survivor can never wedge the next cell.
                if not IS_WINDOWS and shutil.which("pkill") is not None:
                    try:
                        subprocess.run(["pkill", "-x",
                                        "zenoh_shm_round_trip_pong"],
                                       capture_output=True)
                    except OSError:
                        pass
            finally:
                stop_usage_sampler(sampler)
        return rc, inv_timed_out

    start = time.monotonic()
    timed_out = False
    rc = 0
    if usage_enabled():
        # Per-size invocations for per-size usage attribution: the binary
        # sweeps sizes internally, so from outside a single invocation the
        # size boundaries are invisible — restrict each invocation to ONE
        # size (the same CER_BENCH_PAYLOAD_SIZES mechanism as a partial
        # re-sweep; .bin contract + gates unchanged). Announced loudly:
        # process bring-up now happens once per size instead of once per
        # sweep, which does not touch the timed windows (each size carries
        # its own warmup) but IS a run-shape difference worth recording.
        print(f"  → {bench.bin_name} chrt={chrt}  ({USAGE_ENV}=1: one "
              f"invocation per size + .usage.csv sidecars, watchdog "
              f"{timeout_s}s each; logs: {_short(log_dir)})")
        for size in sweep:
            env_size = env.copy()
            env_size["CER_BENCH_PAYLOAD_SIZES"] = str(size)
            log_path = log_dir / f"{raw_name}_{size}.log"
            print(f"    → payload={size}")
            rc_i, to_i = _invoke(env_size, log_path,
                                 usage_sidecar_path(raw_dir, raw_name, size))
            timed_out = timed_out or to_i
            if rc_i != 0 or rc_i is None:
                rc = rc_i if rc_i is not None else 1
    else:
        log_path = log_dir / f"{raw_name}.log"
        print(f"  → {bench.bin_name} chrt={chrt}  (log: {_short(log_path)}, "
              f"watchdog {timeout_s}s)")
        rc, timed_out = _invoke(env, log_path, None)
    elapsed = time.monotonic() - start
    # Exact-set gate: the binaries sweep the active sizes internally, so
    # success means EVERY swept per-size payload is ACCOUNTED — a .bin,
    # or (fixed100) a did_not_sustain sidecar recording an exhausted
    # ladder (a recorded no-latency outcome, not a hole) — never
    # `glob count > 0`, which a partial sweep (or a survivor of the
    # stale-mix guard) passes.
    missing = [s for s in sweep if not payload_accounted(raw_dir, raw_name, s)]
    dns = [s for s in sweep
           if read_rate_sidecar(rate_sidecar_path(raw_dir, raw_name, s))
           == DID_NOT_SUSTAIN]
    # Existence is not enough: compile_csv.py requires EXACTLY
    # samples_for(variant, payload)[1] samples per .bin, so a binary that
    # exits 0 having dumped a short (or non-multiple-of-8) file would make
    # a standalone `bench.py native` report a pass for an artifact the CSV
    # step then rejects. Apply the same schedule here, at the cell —
    # skipping the payloads that legitimately mint no .bin: a
    # did_not_sustain rung is ACCOUNTED for by its sidecar, not by samples.
    short = []
    for size in sweep:
        bin_path = raw_dir / f"{raw_name}_{size}.bin"
        if size in missing or size in dns or not bin_path.exists():
            continue
        want = samples_for(variant, size)[1]
        nbytes = bin_path.stat().st_size
        if nbytes != want * 8:
            short.append((size, nbytes, want))
    ok = rc == 0 and not timed_out and not missing and not short
    status = "ok" if ok else "FAIL"
    rc_str = f"rc={rc}" + (" after watchdog kill" if timed_out else "")
    good = len(sweep) - len(missing) - len(dns) - len(short)
    print(f"    {status} {rc_str} bins="
          f"{good}/{len(sweep)} "
          f"({elapsed:.0f}s)")
    if dns:
        print(f"    ! DID NOT SUSTAIN the fixed100 ladder at payloads: "
              f"{', '.join(map(str, dns))} — no latency minted (empty rows "
              f"downstream)", file=sys.stderr)
    if missing:
        print(f"    missing .bins for payloads: {', '.join(map(str, missing))}",
              file=sys.stderr)
    for size, nbytes, want in short:
        got = nbytes // 8
        tail = " (not a whole number of u64 samples)" if nbytes % 8 else ""
        print(f"    payload {size}: {got} samples in "
              f"{raw_name}_{size}.bin{tail} — the {variant} schedule pins "
              f"{want} measured samples", file=sys.stderr)
    return ok

def cmd_native(args: argparse.Namespace) -> int:
    check_ambient_pacing(args.variant)
    set_dma_basis("host")
    announce_posture("native bins")
    require_dma_posture_deliverable("native bins")
    refuse_ambient_sample_overrides("native", args.variant)
    restriction = ambient_payload_restriction()
    if restriction is not None:
        print(f"[restriction] CER_BENCH_PAYLOAD_SIZES — native sweeps run "
              f"ONLY: {', '.join(map(str, restriction))} (other sizes' .bins "
              f"are kept — in-place partial re-sweep)")
    ensure_built(NATIVE_DIR)
    raise_fd_limit()
    run_dir = resolve_run_dir(args)
    raw_dir = raw_dir_of(rep_dir_of(run_dir, args.rep))
    raw_dir.mkdir(parents=True, exist_ok=True)
    log_dir = raw_dir / "_logs"
    print_governor_state_loudly()
    manifest = RunManifest(run_dir, "native", args.variant, args.rep)

    chrt_modes: List[int] = []
    if args.chrt in ("0", "both"):
        chrt_modes.append(0)
    if args.chrt in ("1", "both"):
        if warn_if_chrt_requested_but_unavailable(1):
            chrt_modes.append(1)
        else:
            manifest.record_skip("*_chrt1", "chrt -f 80 unavailable on this "
                                            "host — chrt-on cells skipped")
    if not chrt_modes:
        # `--chrt 1` on a host without chrt: the loop below would run ZERO
        # cells and return 0, so a wrapper would read an entirely
        # unmeasured run as a pass. (`--chrt both` always keeps chrt0, so
        # its documented skip behaviour is untouched.)
        # finalize() FIRST: the *_chrt1 skip was just recorded, and a
        # refusal that never writes run.json leaves compile-csv with no
        # manifest to read the pacing variant from.
        manifest.finalize()
        print("native: --chrt 1 was requested but chrt -f 80 is unavailable "
              "on this host — no runnable cells; refusing to report an "
              "unmeasured run as a pass.", file=sys.stderr)
        return 2

    benches: List[NativeBench] = list(NATIVE_BENCHES)
    if args.bin:
        benches = [b for b in benches
                   if b.bin_name == args.bin or b.raw_prefix == args.bin]
        if not benches:
            print(f"no native bench matches '{args.bin}'", file=sys.stderr)
            print(f"available: {', '.join(b.bin_name for b in NATIVE_BENCHES)}",
                  file=sys.stderr)
            return 2

    # zenoh-SHM wedges (blocks forever) under the default 8 MiB memlock —
    # gate those cells up front rather than burning their watchdog window.
    if any(b.bin_name.startswith("zenoh") for b in benches):
        if not ensure_memlock_for_zenoh():
            dropped = [b.bin_name for b in benches
                       if b.bin_name.startswith("zenoh")]
            benches = [b for b in benches
                       if not b.bin_name.startswith("zenoh")]
            print(f"  ! skip {', '.join(dropped)} (memlock too low — see "
                  f"above)", file=sys.stderr)
            manifest.record_skip(", ".join(dropped),
                                 "RLIMIT_MEMLOCK too low and could not be "
                                 "raised — zenoh-SHM would wedge")
            if not benches:
                manifest.finalize()
                return 2

    n_ok = n_fail = 0
    try:
        for chrt in chrt_modes:
            print(f"\n############### native variant={args.variant} chrt={chrt} "
                  f"rep={args.rep} ({PLATFORM_LABEL}) ###############")
            for b in benches:
                if chrt == 1 and not b.chrt_on:
                    print(f"  ! skip {b.raw_prefix}_chrt1 (spin-bound bench — "
                          f"chrt0 only by design)")
                    manifest.record_skip(f"{b.raw_prefix}_chrt1",
                                         "spin-bound bench — chrt0 only by "
                                         "design")
                    continue
                started = _now_iso()
                ok = run_native_bench(b, chrt, args.variant, raw_dir, log_dir)
                manifest.record_cell(f"{b.raw_prefix}_chrt{chrt}", None,
                                     "ok" if ok else "fail", started)
                if ok:
                    n_ok += 1
                else:
                    n_fail += 1
    finally:
        manifest.finalize()

    print(f"\nnative sweep: {n_ok} passed, {n_fail} failed "
          f"(raw → {_short(raw_dir)})")
    return 0 if n_fail == 0 else 1

# ============================================================================
# Subcommand: workspace
# ============================================================================

def run_workspace_leg(leg: str, chrt: int, variant: str, raw_dir: Path,
                      log_dir: Path, sizes: Sequence[int],
                      msg: str = "variable") -> str:
    """Drive workspace/run_workspace.sh for one (leg, chrt, msg class).
    Returns "ok", "skip" (runner exited rc=77 = self-declared structural
    skip, e.g. split x backtoback), or "fail".

    Interface contract with the runner:
      - invoked as `bash run_workspace.sh <leg>` with leg ∈ {split, mono}
        (a requested `default` leg is REFUSED loudly by the runner —
        because of the park-wake bug);
      - reads CER_BENCH_RAW_DUMP_DIR / CER_BENCH_RAW_NAME (set here),
        CER_BENCH_PACING, CER_BENCH_CHRT ∈ {0,1} (runner applies the chrt
        wrap around the cerulion CLI itself — sudo would strip the env
        otherwise), CER_BENCH_PAYLOAD_SIZES (whitespace-separated restriction;
        unset = the pinned 10-size sweep), CER_BENCH_SMOKE_N /
        CER_BENCH_TARGET_SAMPLES / CER_BENCH_WARMUP sample overrides;
      - sample counts follow the G1 contract: CER_BENCH_TARGET_SAMPLES
        means MEASURED samples. Backtoback (measured, warmup) is exported
        explicitly below; quiescent legs take NO counts from here — the
        runner derives per-size measured = total − warmup from its own
        schedule_for copy (the four-place schedule lockstep: native/src/
        lib.rs::quiescent_schedule, ros2/run_bench.sh, bench.py::
        quiescent_schedule, workspace/run_workspace.sh::schedule_for);
      - deliberately does NOT set CERULION_NETWORK (both legs run the product's
        default permissive posture
        — the network gateway spawns, parks at zero demand, and sits
        outside the measured SHM chain), rebuilds binary + cdylibs, runs
        the watchdog + flood guard, and writes ${RAW_NAME}_<size>.bin per
        payload.
    CER_BENCH_WALL_STAMP=1 is REQUIRED on workspace legs: deterministic
    stamps are the platform default, and without wall stamps no wall
    latency samples are collected.
    """
    runner = WORKSPACE_DIR / "run_workspace.sh"
    if not runner.exists():
        print(f"  ! FAIL workspace leg {leg}: runner missing at {_short(runner)}")
        return "fail"
    raw_name = workspace_raw_name(leg, chrt, msg)

    # BEFORE cleanup_iceoryx(), which unlinks every iceoryx/zenoh segment in
    # /dev/shm -- state SHARED with any other tenant on this box, and not
    # ours to destroy on the way to refusing. The same rule applies one
    # level up, at this function's own callers.
    #
    # Kept in this function as well as at its callers: this is where bash is
    # actually spawned, and `_run_smoke_cells` calls it directly.
    if shutil.which("bash") is None:
        raise SystemExit(
            "bash not on PATH — the workspace legs are driven by "
            "run_workspace.sh. Refusing before cleanup_iceoryx() rather "
            "than wiping this machine's shared memory on the way to "
            "failing. Install bash and re-run.")

    # Same placement, same reason: this is where the environment carrying
    # CERULION= is actually handed to run_workspace.sh, and `_run_smoke_cells`
    # calls this function directly. The leg commands refuse earlier too — a
    # refusal that costs seconds beats one that arrives after cells have run.
    refuse_foreign_cerulion_override(f"workspace leg {leg}")

    cleanup_iceoryx()

    env = os.environ.copy()
    env["CER_BENCH_RAW_DUMP_DIR"] = str(raw_dir)
    env["CER_BENCH_RAW_NAME"] = raw_name
    # TYPE-CLASS axis: variable (default — the incumbent Image legs,
    # byte-identical invocation) | pod (the fixed-PodPayload twin chain;
    # the runner rebuilds the pod crates per size with the baked array
    # length). See METHODOLOGY § "The type-class axis".
    env["CER_BENCH_MSG"] = msg
    env[PACING_ENV] = variant
    env["CER_BENCH_CHRT"] = str(chrt)
    env["CER_BENCH_WALL_STAMP"] = "1"
    if dma_lock_enabled():
        env["CERULION_CPU_DMA_LOCK"] = "1"
    # else STOCK posture: the runner reads the inherited CER_BENCH_DMA_LOCK=0
    # itself (its loud banner, and `env -u CERULION_CPU_DMA_LOCK` on the
    # graph so an inherited cap var cannot survive into it). Popping
    # nothing here is safe because a contradictory ambient export is
    # refused twice over — by dma_lock_enabled() here, and by the runner
    # itself, which must honour the contract on a direct invocation too.
    if variant == "backtoback":
        # G1: CER_BENCH_TARGET_SAMPLES means MEASURED samples. Export the
        # backtoback (measured, warmup) pair explicitly rather than relying
        # on ambient inheritance (payload is irrelevant in this variant).
        # Quiescent legs deliberately get no counts: the runner derives
        # per-size measured from its own schedule_for (four-place lockstep).
        _rate, measured, warmup = samples_for(variant, 0)
        env["CER_BENCH_TARGET_SAMPLES"] = str(measured)
        env["CER_BENCH_WARMUP"] = str(warmup)
    if tuple(sizes) != PAYLOAD_SIZES:
        env["CER_BENCH_PAYLOAD_SIZES"] = " ".join(str(s) for s in sizes)

    # Stale-mix guard (same rule as run_native_bench): clear THIS
    # prefix's swept-size .bins from a prior run immediately before
    # spawning — after every early return above (including the
    # structurally-unrunnable legs, refused by cmd_workspace before they
    # reach here; a runner that declares itself unrunnable at RUNTIME, rc=77,
    # has by then cleared bins that were stale for that leg anyway). Without
    # it, a runner that dies
    # before its per-size loop (a failed rebuild, a refused leg) leaves the
    # previous rep's exact-count .bins in place; the `missing` check below
    # is satisfied by them and compile-csv then aggregates the OLD samples
    # as this run's.
    # EVERY per-size artifact of this prefix, not just the .bin — the same
    # set run_native_bench clears. compile_csv builds its row set from the
    # UNION of the .bin walk and a `*.rate` glob, so a surviving sidecar
    # introduces a (cell, size) row on its own: a run that dies before a
    # later size leaves that size's OLD `.rate` in place and the CSV
    # publishes the prior run's did_not_sustain verdict against this run's
    # sweep. A stale `.usage.csv` describes the wrong invocation the same
    # way. Under an ambient restriction only the restricted sizes are
    # cleared, so a partial re-sweep still merges over a prior full one.
    # ASYMMETRY worth naming: run_native_bench places its clear AFTER every
    # skip-return, so a skipped native cell never destroys a prior run's
    # data. This leg cannot — rc=77 is only knowable post-spawn — so a leg
    # that turns out to be a runtime skip loses a prior did_not_sustain
    # `.rate` too, and its row degrades from an ACCOUNTED verdict to a
    # missing-`.bin` bad row. Loud either way (the bad row is reported),
    # and the alternative is worse: keeping the sidecar publishes a
    # previous run's verdict as this run's.
    stale = [p for s in sizes
             for p in (raw_dir / f"{raw_name}_{s}.bin",
                       rate_sidecar_path(raw_dir, raw_name, s),
                       usage_sidecar_path(raw_dir, raw_name, s))
             if p.exists()]
    # BEFORE the artifact sweep, and in THIS leg. Two things have to be true
    # and a previous pass got neither: the refusal belongs where the bash
    # spawn is, and it must precede the unlink that would otherwise delete
    # the prior run's .bin/.rate/.usage.csv on the way to failing. (It was
    # mis-anchored onto the wrong leg's stale sweep, so run_native_bench —
    # whose command is the compiled binary and which spawns no bash at all —
    # refused instead.) cleanup_iceoryx above deliberately still runs first:
    # it clears ambient SHM scratch, not prior-run results.
    #
    # The sibling legs are preflighted by their CALLERS, verified rather than
    # assumed: cmd_native calls ensure_built (cargo) before run_native_bench,
    # and cmd_ros2 calls ensure_docker_image (which gates on
    # docker_available) before run_ros2_cell_payload.
    #
    # cmd_workspace's own guard is indirect and PARTIAL, which is the whole
    # gap: it calls resolve_run_dir, which computes the machine hash, which
    # shells out to bash — so a bash-less `bench.py workspace` already
    # refuses on the DEFAULT path. But resolve_run_dir returns early when
    # --run-dir is given, before the hash is computed, and cmd_full always
    # passes one down. On those paths nothing stood here.
    # (the bash refusal now sits above cleanup_iceoryx -- see there)
    for f in stale:
        f.unlink()
    if stale:
        print(f"    cleared {len(stale)} stale {raw_name}_* .bin/.rate"
              f"/.usage.csv from a prior run")

    log_path = log_dir / f"{raw_name}.log"
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"  → workspace leg={leg} msg={msg} chrt={chrt}  "
          f"(log: {_short(log_path)})")
    start = time.monotonic()
    with log_path.open("wb") as logf:
        try:
            rc = subprocess.run(["bash", str(runner), leg], cwd=WORKSPACE_DIR,
                                env=env, stdout=logf,
                                stderr=subprocess.STDOUT).returncode
        except OSError as e:
            # which() above is a lookup, not a promise (EACCES, a broken
            # symlink, a bad interpreter). A leg that could not START is a
            # setup failure with a named cause, not a traceback.
            raise SystemExit(f"could not start `bash {runner}`: {e}") from e
    elapsed = time.monotonic() - start
    if rc == RC_STRUCTURAL_SKIP:
        print(f"    ! skip (runner rc={RC_STRUCTURAL_SKIP} — leg declared "
              f"itself structurally unrunnable here; see log)")
        return "skip"
    # A payload is accounted by its .bin OR (fixed100) a did_not_sustain
    # sidecar — the runner writes the sidecar when a size exhausted the
    # fallback ladder; that is a recorded outcome, not a hole.
    missing = [s for s in sizes if not payload_accounted(raw_dir, raw_name, s)]
    dns = [s for s in sizes
           if read_rate_sidecar(rate_sidecar_path(raw_dir, raw_name, s))
           == DID_NOT_SUSTAIN]
    # Same rule as run_native_bench: existence is not enough, the COUNT has
    # to match the schedule. The runner enforces its own equality gate, but
    # against its own copy of the schedule — checking here, against
    # `samples_for`, is what would catch a drift between the two mirrors
    # before compile-csv (a separate, --allow-partial-skippable step) does.
    # A did_not_sustain payload mints no .bin and is skipped.
    short = []
    for size in sizes:
        bin_path = raw_dir / f"{raw_name}_{size}.bin"
        if size in missing or size in dns or not bin_path.exists():
            continue
        want = samples_for(variant, size)[1]
        nbytes = bin_path.stat().st_size
        if nbytes != want * 8:
            short.append((size, nbytes, want))
    ok = rc == 0 and not missing and not short
    status = "ok" if ok else "FAIL"
    print(f"    {status} rc={rc} "
          f"bins={len(sizes) - len(missing) - len(dns) - len(short)}/{len(sizes)} "
          f"({elapsed:.0f}s)")
    if dns:
        print(f"    ! DID NOT SUSTAIN the fixed100 ladder at payloads: "
              f"{', '.join(map(str, dns))} — no latency minted (empty rows "
              f"downstream)", file=sys.stderr)
    if missing:
        print(f"    missing .bins for payloads: {', '.join(map(str, missing))}",
              file=sys.stderr)
    for size, nbytes, want in short:
        got = nbytes // 8
        tail = " (not a whole number of u64 samples)" if nbytes % 8 else ""
        print(f"    payload {size}: {got} samples in "
              f"{raw_name}_{size}.bin{tail} — the {variant} schedule pins "
              f"{want} measured samples", file=sys.stderr)
    return "ok" if ok else "fail"

def cmd_workspace(args: argparse.Namespace) -> int:
    check_ambient_pacing(args.variant)
    set_dma_basis("host")
    announce_posture("workspace legs")
    require_dma_posture_deliverable("workspace legs")
    refuse_ambient_sample_overrides("workspace", args.variant)
    refuse_foreign_cerulion_override("workspace")
    raise_fd_limit()
    # Preflight FIRST, the way cmd_native calls ensure_built before it
    # resolves a run dir. This leg had none: resolve_run_dir returns early
    # when --run-dir is given -- so it never reaches the machine-hash bash
    # call that guards the default path -- and cmd_full always passes one.
    # A bash-less host therefore created the run directory, built a
    # RunManifest and finalized it, and only then refused inside the leg,
    # leaving artifacts that describe a run which never happened.
    if shutil.which("bash") is None:
        print("workspace: bash not on PATH — the workspace legs are driven "
              "by run_workspace.sh. Refusing before creating a run "
              "directory or a manifest.", file=sys.stderr)
        return 3

    run_dir = resolve_run_dir(args)
    raw_dir = raw_dir_of(rep_dir_of(run_dir, args.rep))
    raw_dir.mkdir(parents=True, exist_ok=True)
    log_dir = raw_dir / "_logs"
    restriction = ambient_payload_restriction()
    if args.sizes is not None:
        sizes: Sequence[int] = args.sizes
    elif restriction is not None:
        sizes = restriction
        print(f"[restriction] CER_BENCH_PAYLOAD_SIZES — workspace legs run "
              f"ONLY: {', '.join(map(str, restriction))} (in-place partial "
              f"re-sweep)")
    else:
        sizes = PAYLOAD_SIZES
    print_governor_state_loudly()
    manifest = RunManifest(run_dir, "workspace", args.variant, args.rep)

    legs = WORKSPACE_LEGS if args.leg == "both" else (args.leg,)
    msg_classes = (WORKSPACE_MSG_CLASSES if args.msg_class == "both"
                   else (args.msg_class,))
    chrt_modes: List[int] = []
    if args.chrt in ("0", "both"):
        chrt_modes.append(0)
    if args.chrt in ("1", "both"):
        if warn_if_chrt_requested_but_unavailable(1):
            chrt_modes.append(1)
        else:
            manifest.record_skip("cerulion_workspace_*_chrt1",
                                 "chrt -f 80 unavailable on this host — "
                                 "chrt-on cells skipped")
    if not chrt_modes:
        # See cmd_native: an empty chrt set would run zero legs and return
        # 0. `--chrt both` keeps chrt0 and is unaffected.
        # finalize() FIRST: the *_chrt1 skip was just recorded, and a
        # refusal that never writes run.json leaves compile-csv with no
        # manifest to read the pacing variant from.
        manifest.finalize()
        print("workspace: --chrt 1 was requested but chrt -f 80 is "
              "unavailable on this host — no runnable legs; refusing to "
              "report an unmeasured run as a pass.", file=sys.stderr)
        return 2

    n_ok = n_fail = n_skip = 0
    try:
        for chrt in chrt_modes:
            print(f"\n############### workspace variant={args.variant} "
                  f"chrt={chrt} rep={args.rep} ({PLATFORM_LABEL}) "
                  f"###############")
            for leg in legs:
              for msg in msg_classes:
                name = workspace_raw_name(leg, chrt, msg)
                reason = workspace_leg_skip_reason(leg, args.variant)
                if reason is not None:
                    print(f"  ! skip {name} ({reason})")
                    manifest.record_skip(name, reason)
                    n_skip += 1
                    continue
                started = _now_iso()
                outcome = run_workspace_leg(leg, chrt, args.variant, raw_dir,
                                            log_dir, sizes, msg)
                manifest.record_cell(name, None, outcome, started)
                if outcome == "ok":
                    n_ok += 1
                elif outcome == "skip":
                    n_skip += 1
                else:
                    n_fail += 1
    finally:
        manifest.finalize()

    print(f"\nworkspace sweep: {n_ok} passed, {n_fail} failed, {n_skip} skipped")
    return 0 if n_fail == 0 else 1

# ============================================================================
# Subcommand: ros2
# ============================================================================

# Docker image label carrying the repo git sha the image was built from.
# Without it an existing image silently satisfying
# ensure_docker_image could carry a baked bench driver from an arbitrarily
# old checkout with nothing to betray it.
BENCH_IMAGE_SHA_LABEL = "com.cerulion.bench.repo_git_sha"

def _docker_image_field(tag: str, fmt: str) -> Optional[str]:
    """One `docker image inspect --format` read; None when the image is
    missing or the daemon errors."""
    # Defensive, not load-bearing: every caller today reaches this behind
    # ensure_docker_image's docker_available() gate (record_docker_image runs
    # one line after it; warn_if_image_stale is called from inside it), so
    # the guard cannot currently fire. Kept because this function's own
    # contract is "None when the image is missing or the daemon errors", and
    # a docker-less host is that, not a traceback.
    if shutil.which("docker") is None:
        return None
    try:
        r = subprocess.run(["docker", "image", "inspect", "--format", fmt,
                            tag], capture_output=True, text=True, timeout=30)
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    return r.stdout.strip()

def docker_image_provenance(tag: str) -> Dict[str, Optional[str]]:
    """{id, repo_git_sha} for the run manifest — the sha comes from the
    BENCH_IMAGE_SHA_LABEL embedded at build time (None on images that
    predate the labeling)."""
    label_fmt = '{{index .Config.Labels "' + BENCH_IMAGE_SHA_LABEL + '"}}'
    return {
        "id": _docker_image_field(tag, "{{.Id}}"),
        "repo_git_sha": _docker_image_field(tag, label_fmt) or None,
    }

def warn_if_image_stale(tag: str) -> None:
    """LOUD staleness warning when an existing image's embedded repo sha
    differs from HEAD. Deliberately NOT an auto-rebuild: a
    mid-campaign silent rebuild would swap the measured compared/vendor
    code between cells, which is worse than a stale image — the warning
    names --build-image and the manifest records the image id + sha."""
    head = _git_sha()
    label = docker_image_provenance(tag)["repo_git_sha"]
    if head == "unknown":
        print(f"  ! cannot verify {tag} freshness (git sha unavailable) — "
              f"image repo-sha label: {label or '<none>'}", file=sys.stderr)
        return
    if label is None:
        print(f"  ! STALE-IMAGE CHECK: {tag} carries no "
              f"{BENCH_IMAGE_SHA_LABEL} label (built before sha labeling) — "
              f"its baked bench driver is of unknown vintage. Rebuild with "
              f"--build-image before a citable sweep.",
              file=sys.stderr)
        return
    if label != head:
        print(f"  ! STALE IMAGE {tag}: built at repo sha {label[:12]} but "
              f"HEAD is {head[:12]} — a baked bench driver from that old "
              f"checkout would be measured silently. Rebuild with "
              f"--build-image before a citable sweep (NOT auto-rebuilding: "
              f"a mid-campaign silent rebuild is worse; the image id + sha "
              f"are recorded in run.json).",
              file=sys.stderr)
    elif _git_dirty():
        print(f"  ! {tag} matches HEAD ({head[:12]}) but the worktree is "
              f"DIRTY — the sha label cannot prove the image matches the "
              f"code on disk.", file=sys.stderr)

def ensure_docker_image(distro: str, force_rebuild: bool = False) -> None:
    if not docker_available():
        raise SystemExit("docker is not available — install Docker Desktop / "
                         "Docker Engine first")
    tag = image_tag(distro)
    if not force_rebuild:
        if subprocess.run(["docker", "image", "inspect", tag],
                          capture_output=True).returncode == 0:
            warn_if_image_stale(tag)
            return
    # Build context is the REPO ROOT: the Dockerfile's COPY paths are
    # repo-root-relative (benches/latency/ros2/...), and the co-located
    # BuildKit Dockerfile.dockerignore keeps .git/ and target/ out of the
    # context. BuildKit is FORCED here because that per-Dockerfile ignore
    # file is BuildKit-only: Docker >= 23 already defaults to it, older
    # engines honor the env, and an ambient DOCKER_BUILDKIT=0 (or a legacy
    # builder default) would silently tar .git/ + target/ (tens of GB)
    # into the context — so an explicit 0 is refused, never overridden, and
    # every other case is pinned to 1 in the child environment (an engine
    # with no BuildKit at all then REFUSES loudly, "buildkit not supported
    # by daemon", instead of quietly uploading the repo root).
    if os.environ.get("DOCKER_BUILDKIT", "").strip() == "0":
        raise SystemExit(
            "DOCKER_BUILDKIT=0 is exported — the bench image build REQUIRES "
            "BuildKit (the co-located Dockerfile.dockerignore that keeps "
            ".git/ and target/ out of the repo-root build context is a "
            "BuildKit-only feature; the legacy builder would upload tens of "
            "GB). Unset it: `unset DOCKER_BUILDKIT`.")
    print(f"[docker] building {tag} from {_short(REPO_ROOT)} "
          f"(DOCKER_BUILDKIT=1)")
    build_env = dict(os.environ, DOCKER_BUILDKIT="1")
    rc = subprocess.run([
        "docker", "build",
        "--build-arg", f"ROS_DISTRO={distro}",
        # Embed the repo sha so a later invocation can detect staleness
        # (see warn_if_image_stale; recorded per run in run.json).
        "--label", f"{BENCH_IMAGE_SHA_LABEL}={_git_sha()}",
        "-t", tag,
        "-f", str(ROS2_DIR / "docker" / "Dockerfile"),
        str(REPO_ROOT),
    ], env=build_env).returncode
    if rc != 0:
        raise SystemExit(
            f"docker build failed for {tag} — if the daemon reported that "
            f"BuildKit is unsupported, upgrade Docker (>= 23 ships it by "
            f"default): the legacy builder ignores "
            f"docker/Dockerfile.dockerignore and would upload the whole "
            f"repo root as build context.")

def warn_no_shm_host_sysctls() -> None:
    """no_shm cells run with --network host and rely on HOST sysctls
    (rmem/wmem/ipfrag). Docker refuses --sysctl with host
    networking, so the values must be raised on the host. Warn loudly when
    they look too small instead of letting large UDP cells fail opaquely."""
    if not IS_LINUX:
        return
    checks = (("/proc/sys/net/core/rmem_max", 134217728),
              ("/proc/sys/net/core/wmem_max", 134217728),
              ("/proc/sys/net/ipv4/ipfrag_high_thresh", 134217728))
    for path, want in checks:
        try:
            val = int(Path(path).read_text().strip())
        except (OSError, ValueError):
            continue
        if val < want:
            print(f"  ! host sysctl {path} = {val} < {want} — large no_shm "
                  f"(UDP) payloads may drop; raise it before the sweep",
                  file=sys.stderr)

def _cerulion_repo_mount(cell: "Ros2Cell") -> List[str]:
    """Docker args that let a cerulion cell build librmw_cerulion.so.

    EMPTY for every other rmw, so nothing about the stock lanes changes:
    they get the same container they got before this lane existed.

    WHY A MOUNT AND NOT A BAKED-IN .so. The crate's build.rs runs bindgen
    against whatever ROS headers it finds on AMENT_PREFIX_PATH, and the
    introspection struct layouts drift across distros, so ONE .so in the
    image would be ABI-correct for at most one of the three. Building
    inside the cell's own container, from the tree mounted here, makes
    the binary match both the distro it runs against and the tree this
    run was launched from. `ensure_rmw_cerulion` in run_bench.sh is the
    other half.

    WRITABLE, deliberately. cargo writes to CARGO_TARGET_DIR, which
    points inside the mount (REPO_ROOT/target) so the second and later
    containers of a campaign get a freshness no-op instead of repeating a
    full release build. A read-only mount would either cost a full build
    per cell or need a second scratch mount to hold the same artifacts
    under a different name.
    """
    if cell.rmw != "cerulion":
        return []
    return [
        "-v", f"{REPO_ROOT}:/work",
        "-e", "CER_RMW_REPO=/work",
    ]


def docker_args_for_cell(cell: Ros2Cell, payload: int, variant: str,
                         raw_dir: Path, attempt: int,
                         extra_env: Optional[Dict[str, str]] = None,
                         rate_override: Optional[int] = None) -> List[str]:
    """`docker run` flags for ONE (cell, payload) container invocation.

    One payload per container invocation: the per-payload rate /
    sample counts travel as env, so a container never needs a second
    schedule lookup — but ros2/run_bench.sh keeps the same schedule as
    in-container defaults (the four-place lockstep: native/src/lib.rs::
    quiescent_schedule, ros2/run_bench.sh, bench.py::quiescent_schedule,
    workspace/run_workspace.sh::schedule_for). G1 contract: the exported
    CER_BENCH_TARGET_SAMPLES is the MEASURED count (samples_for already
    derived total − warmup for quiescent), so every .bin sample-count gate
    checks measured.

    `rate_override` (fixed100 fallback ladder): pins this invocation's
    publish rate to one ladder rung — it replaces the schedule rate in
    BOTH the exported CER_BENCH_TARGET_RATE_HZ and the wall-ceiling
    arithmetic (a 20 Hz rung's nominal window is 5x the 100 Hz one, so
    the ceiling must scale with the rung actually run)."""
    rate, measured, warmup = samples_for(variant, payload)
    if rate_override is not None:
        rate = rate_override
    # Per-(cell, payload) wall ceiling: 2x the schedule's nominal pacing
    # window + 60 s bring-up/drain headroom, floored at the pre-tail-resolved
    # 300 s. The flat 300 s ceiling predates the tail-resolved schedule
    # extension — the 16 MiB window alone is now ~205 s nominal, so a flat
    # ceiling would kill legitimately-pacing cells.
    if rate is not None:
        nominal_s = -(-(measured + warmup) // rate)   # ceil-div
        cell_timeout_s = max(300, 2 * nominal_s + 60)
    else:
        cell_timeout_s = 300
    # Per-rung name component keeps a laddering cell's containers distinct
    # even if a prior rung's container lingers past --rm.
    rung_tag = f"_r{rate_override}" if rate_override is not None else ""
    args = [
        "--rm",
        "--name", f"{IMAGE_PREFIX}_{cell.name}_{payload}{rung_tag}_{attempt}",
        "--shm-size=4g",
        "--cap-add", "SYS_NICE",
        "--ulimit", "rtprio=99",
        "--ulimit", "memlock=-1",
        "-v", f"{raw_dir}:/raw",
        # run_bench.sh + configs are bind-mounted read-only over the baked-in
        # copies so driver iteration doesn't require an image rebuild.
        "-v", f"{ROS2_DIR / 'run_bench.sh'}:/bench/run_bench.sh:ro",
        "-v", f"{ROS2_DIR / 'configs'}:/bench/configs:ro",
        "-v", f"{ROS2_DIR / 'verify_shm.sh'}:/bench/verify_shm.sh:ro",
        *_cerulion_repo_mount(cell),
        "-e", "CER_BENCH_RAW_DUMP_DIR=/raw",
        "-e", f"CER_BENCH_RAW_NAME={cell.name}",
        "-e", f"{PACING_ENV}={variant}",
        # ONE payload per container invocation: SIZES restricts the driver's
        # size loop to exactly this payload (run_bench.sh word-splits it),
        # and the MEASURED sample count for that payload travels explicitly
        # below (G1: TARGET means measured; warmup rides on top).
        "-e", f"SIZES={payload}",
        "-e", f"CER_BENCH_TARGET_SAMPLES={measured}",
        "-e", f"CER_BENCH_WARMUP={warmup}",
        "-e", f"CER_BENCH_QOS={cell.qos}",
        # run_bench.sh's override precedence reads the bare spellings; passed
        # with the SAME values as the CER_BENCH_* pair above.
        "-e", f"TARGET_SAMPLES={measured}",
        "-e", f"WARMUP={warmup}",
        # Cell selectors (run_bench.sh pins single values from these).
        "-e", f"RMWS={cell.rmw}",
        "-e", f"SHM_MODE={cell.shm}",
        "-e", f"RECV_PATH={cell.recv}",
        # TYPE-CLASS axis: pod (default) | image — msg_class_dispatch.hpp
        # picks the message type; run_bench.sh validates + gates.
        "-e", f"CER_BENCH_MSG={cell.msg}",
        "-e", f"CHRT_MODE={'on' if cell.chrt else 'off'}",
        "-e", "WITH_CHRT=0",   # chrt is a cell axis here, not an in-container sweep
        "-e", "WITH_SHM=0",
        "-e", f"BENCH_CELL_TIMEOUT_S={cell_timeout_s}",
        # A5/C5 compared-stack version provenance: run_bench.sh dumps the
        # installed ros-*/rmw/dds package versions once per cell into
        # _logs/<cell>_versions.txt (the container side is wired by
        # ros2/run_bench.sh; harmless when an older driver ignores it).
        "-e", "CER_BENCH_DUMP_VERSIONS=1",
    ]
    if rate is not None:
        args.extend(["-e", f"CER_BENCH_TARGET_RATE_HZ={rate}"])
    # Appended LAST so a caller-supplied value wins over the derived one —
    # that is how the smoke dispatcher pins its counts (smoke_sample_env).
    for k, v in (extra_env or {}).items():
        args.extend(["-e", f"{k}={v}"])

    # Network namespace: no_shm (UDP) needs host networking on Linux so the
    # host's rmem/wmem/ipfrag sysctls apply (docker refuses --sysctl together
    # with --network host). shm cells stay on the default bridge with the
    # ipfrag bump for the discovery-time UDP traffic.
    if cell.shm == "no_shm":
        if IS_LINUX:
            args.extend(["--network", "host"])
        else:
            args.extend(["--sysctl", "net.ipv4.ipfrag_high_thresh=134217728"])
    else:
        args.extend(["--sysctl", "net.ipv4.ipfrag_high_thresh=134217728"])

    # STOCK posture (CER_BENCH_DMA_LOCK=0) omits the device: the container
    # then has no /dev/cpu_dma_latency, the in-container bins print their
    # loud no-device note and run uncapped — the untuned-host shape.
    #
    # THE FLAG FOLLOWS THE LABEL — one rule, so the two cannot disagree.
    # Passing the device is what CAPS a ROS 2 cell, so deciding it by any
    # predicate other than the posture would let a container run capped
    # under a `tuned-uncapped` manifest (or the reverse).
    #
    # What that single rule buys, on the mode-0600 host this round is
    # about: a ros2-ONLY sweep resolves on the CONTAINER basis (root can
    # open a device this process cannot), so the posture is `tuned`, the
    # device is passed, and the cells really are capped. A `full` sweep on
    # the same host resolves on the HOST basis, because its native and
    # workspace legs measure here as this user and genuinely cannot cap —
    # so the posture is `tuned-uncapped` and the device is withheld. That
    # withholding is deliberate: handing it over would cap the compared stack's
    # cells while Cerulion's ran uncapped, inside ONE run dir under ONE
    # label that describes neither half.
    if IS_LINUX and resolved_dma_posture() == "tuned":
        args.extend(["--device",
                     "/dev/cpu_dma_latency:/dev/cpu_dma_latency"])

    args.append(image_tag(cell.distro))
    return args

# run_bench.sh exit code for "one or more sizes produced no/short .bin"
# — under fixed100 this is the cannot-sustain signal that walks the
# fallback ladder (unless the log proves a verify_shm failure, which is a
# transport problem no lower rate can fix).
RC_SIZE_FAILED = 13
# run_bench.sh's exit code for "a size's DELIVERY receipts could not
# describe its samples". Deliberately NOT RC_SIZE_FAILED: 13 is the
# fixed100 cannot-sustain signal and drives the rate ladder, and a lower
# rate fixes nothing about an incoherent receipt set — the cell would
# re-run just as incoherent, a lower rung would eventually "succeed", and
# the run would publish a fabricated rate-limitation claim about a stack
# with no rate problem. Structural discrimination, not a log-string grep.
RC_DELIVERY_INCOHERENT = 14

def _log_names_verify_shm_failure(log_path: Path) -> bool:
    """True when a container log records a verify_shm hard failure —
    rc=13 then means 'transport label unverifiable', NOT 'rate not
    sustained', so the fixed100 ladder must NOT step down on it."""
    try:
        return b"verify_shm FAILED" in log_path.read_bytes()
    except OSError:
        return False

def run_ros2_cell_payload(cell: Ros2Cell, payload: int, variant: str,
                          raw_dir: Path, log_dir: Path,
                          max_attempts: int = 3,
                          extra_env: Optional[Dict[str, str]] = None) -> str:
    """Run one (cell, payload) container with retries (3 attempts,
    fresh container each, partial .bins cleared between attempts).

    Returns "ok", "skip" (the container exited rc=77 = self-declared
    structural skip), "did_not_sustain" (fixed100 only: the fallback
    ladder was exhausted — a valid recorded outcome, not a harness
    failure), or "fail".

    fixed100 fallback ladder: the cell runs at the 100 Hz target first;
    a rung whose attempts all end rc=13 (no/short .bin within the
    in-container watchdog — the delivery-accounting "received < measured
    target" signal; run_bench.sh's sample-count gate is an equality
    check) steps DOWN the ladder (100→50→20→sensor floor) and re-runs
    this payload in a fresh container at the next rung. The rung that
    SUSTAINS is recorded in the `.rate` sidecar beside the `.bin`; a
    non-target rung is announced loudly (plots annotate the point).
    Hard failures never ladder: rc=2/11/12 (setup / is_plain / DMA
    lock), a verify_shm rc=13 (transport label unverifiable — a lower
    rate fixes nothing), and any unexpected rc give up as "fail"."""
    log_dir.mkdir(parents=True, exist_ok=True)
    bin_path = raw_dir / f"{cell.name}_{payload}.bin"
    rate_path = rate_sidecar_path(raw_dir, cell.name, payload)
    # Stale-sidecar guard (every variant): a leftover fixed100 label from
    # a prior run must never describe this run's rows. Usage sidecars ride
    # the same rule (a prior usage run's rows must not describe this run).
    rate_path.unlink(missing_ok=True)
    usage_path = usage_sidecar_path(raw_dir, cell.name, payload)
    usage_path.unlink(missing_ok=True)

    fixed100 = variant == "fixed100"
    rungs: List[Optional[int]] = (
        [int(r) for r in fixed100_ladder(payload)] if fixed100 else [None])
    for rung_i, rung in enumerate(rungs):
        attempts = (max_attempts if rung_i == 0
                    else min(max_attempts, FIXED100_FALLBACK_ATTEMPTS))
        rung_tag = f"_r{rung}" if rung is not None else ""
        rung_note = f" @ {rung} Hz" if rung is not None else ""
        for attempt in range(1, attempts + 1):
            bin_path.unlink(missing_ok=True)   # clear a prior partial attempt
            log_path = (log_dir /
                        f"{cell.name}_{payload}{rung_tag}_attempt{attempt}.log")
            print(f"    attempt {attempt}/{attempts}{rung_note}  →  "
                  f"{_short(log_path)}")
            cmd = ["docker", "run", *docker_args_for_cell(
                cell, payload, variant, raw_dir, attempt,
                extra_env=extra_env, rate_override=rung)]
            # Usage sampling (CER_BENCH_USAGE=1): the container's processes
            # are sampled from the HOST — the sampler polls `docker inspect`
            # for the init PID (the container name is minted above) and
            # descends. A re-attempt / ladder rung re-arms the sampler, so
            # the surviving sidecar describes the SAME invocation the .bin
            # came from (matching the .rate rule).
            sampler = (start_usage_sampler(
                usage_path, log_dir,
                docker_name=f"{IMAGE_PREFIX}_{cell.name}_{payload}"
                            f"{rung_tag}_{attempt}")
                if usage_enabled() else None)
            try:
                with log_path.open("wb") as logf:
                    rc = subprocess.run(cmd, stdout=logf,
                                        stderr=subprocess.STDOUT).returncode
            finally:
                stop_usage_sampler(sampler)
            if rc == RC_STRUCTURAL_SKIP:
                print(f"    ! skip (runtime rc={RC_STRUCTURAL_SKIP} — the cell "
                      f"declared itself structurally unrunnable; see log)")
                return "skip"
            if rc == 0 and bin_path.exists():
                if fixed100 and rung is not None:
                    write_rate_sidecar(
                        rate_path, str(rung),
                        f"fixed100 achieved-rate sidecar: "
                        f"target={FIXED100_RATE_HZ}Hz "
                        f"ladder={list(map(int, fixed100_ladder(payload)))}Hz")
                    if rung != FIXED100_RATE_HZ:
                        print(f"    ok — FALLBACK: achieved {rung} Hz, not "
                              f"the {FIXED100_RATE_HZ} Hz target (recorded "
                              f"in {rate_path.name}; plots annotate the "
                              f"point '@{rung}Hz')")
                        return "ok"
                print("    ok")
                return "ok"
            state = "present" if bin_path.exists() else "missing"
            if fixed100 and rc in (2, 11, 12):
                # Setup / is_plain / DMA-lock failures — no rate fixes
                # these; give up hard instead of minting a fake
                # did_not_sustain verdict.
                print(f"    FAIL (rc={rc}, bin={state}) — hard failure "
                      f"class; the fixed100 ladder does not apply")
                return "fail"
            if rc == RC_DELIVERY_INCOHERENT:
                # Never laddered, in either pacing mode: the cell produced
                # samples its own receipts contradict, which no rate
                # changes. The .bin was already removed by the runner.
                print(f"    FAIL (rc={rc}, bin={state}) — delivery receipts "
                      f"contradict the samples; a lower rate fixes nothing "
                      f"(no ladder, no retry)")
                return "fail"
            if fixed100 and rc == RC_SIZE_FAILED \
                    and _log_names_verify_shm_failure(log_path):
                print(f"    FAIL (rc={rc}, bin={state}) — verify_shm "
                      f"failed; a lower rate fixes nothing (no ladder)")
                return "fail"
            if attempt < attempts:
                print(f"    FAIL (rc={rc}, bin={state}); retrying")
            else:
                print(f"    FAIL (rc={rc}, bin={state}) — rung exhausted "
                      f"after {attempts} attempt(s)" if fixed100 else
                      f"    FAIL (rc={rc}, bin={state}) — GIVE UP after "
                      f"{attempts} attempts")
        if not fixed100:
            return "fail"
        # rc=13 (no/short .bin within the watchdog) is the cannot-sustain
        # signal; any OTHER unexpected rc that exhausted its attempts is
        # a hard failure, not a rate problem.
        if rc != RC_SIZE_FAILED:
            print(f"    FAIL — rung{rung_note} died with rc={rc} (not the "
                  f"rc={RC_SIZE_FAILED} cannot-sustain signal); giving up "
                  f"hard rather than walking the ladder on a broken cell")
            return "fail"
        if rung_i < len(rungs) - 1:
            print(f"    NOT SUSTAINED{rung_note} — stepping down the "
                  f"fixed100 ladder {[int(r) for r in rungs]} Hz")
    # fixed100 ladder exhausted: record the no-latency outcome.
    bin_path.unlink(missing_ok=True)
    write_rate_sidecar(
        rate_path, DID_NOT_SUSTAIN,
        f"fixed100 ladder {[int(r) for r in rungs]}Hz exhausted: "
        f"no rung sustained; no latency minted")
    print(f"    DID NOT SUSTAIN — payload {payload} ran at NO fixed100 "
          f"ladder rung {[int(r) for r in rungs]} Hz; no .bin, no latency "
          f"minted (the CSV row renders 'did not sustain')")
    return "did_not_sustain"

def cmd_ros2(args: argparse.Namespace) -> int:
    check_ambient_pacing(args.variant)
    set_dma_basis("container")
    announce_posture("ros2 cells")
    require_dma_posture_deliverable("ros2 cells")
    # ROS 2 cells take their sample counts from `samples_for` inside
    # docker_args_for_cell, so an exported CER_BENCH_SMOKE_N DOES shrink
    # them (same as the native/workspace runners) — the refusal below is
    # the single gate for all three. The warning this replaces claimed the
    # opposite and was false in the only branch it could run in: the
    # ros2_env that pins those counts exists only while _SMOKE_ACTIVE, and
    # the branch required NOT _SMOKE_ACTIVE. It also said "ignoring" one
    # line above a refusal that exits.
    refuse_ambient_sample_overrides("ros2", args.variant)

    distros: List[str] = [args.distro] if args.distro else list(ROS2_DISTROS)
    chrt_modes: List[int] = []
    if args.chrt in ("0", "both"):
        chrt_modes.append(0)
    if args.chrt in ("1", "both") and warn_if_chrt_requested_but_unavailable(1):
        chrt_modes.append(1)
    if not chrt_modes:
        # Same rule as cmd_native/cmd_workspace: `--chrt 1` on a host
        # without chrt leaves no runnable cell, and the sweep below would
        # exit 0 having measured nothing.
        print("ros2: --chrt 1 was requested but chrt -f 80 is unavailable "
              "on this host — no runnable cells; refusing to report an "
              "unmeasured run as a pass.", file=sys.stderr)
        return 2

    # A --cells filter that matches NOTHING is an operator typo, not an
    # empty sweep — refuse loudly with the available names (mirrors
    # cmd_native's --bin handling) BEFORE any docker build runs.
    if args.cells:
        all_cells: List[Ros2Cell] = []
        for distro in distros:
            for chrt in chrt_modes:
                cells, _skips = enumerate_ros2_cells(distro, chrt)
                all_cells.extend(cells)
        if not any(args.cells in c.name for c in all_cells):
            # An empty enumeration reaches this only for a distro whose
            # whole matrix is skipped (the chrt-less case now returns above),
            # and it must refuse for the same reason: a 0-cell sweep exiting
            # 0 reads as a green ROS 2 pass.
            print(f"no ros2 cell matches --cells '{args.cells}'",
                  file=sys.stderr)
            if all_cells:
                print("available cells:", file=sys.stderr)
                for c in all_cells:
                    print(f"  {c.name}", file=sys.stderr)
            else:
                print("(the enumeration itself is empty for the requested "
                      "distro/chrt set — see the skip warnings above)",
                      file=sys.stderr)
            return 2

    # Preflight FIRST, like cmd_native's ensure_built. The per-distro
    # ensure_docker_image below refuses on a docker-less host -- but it runs
    # AFTER resolve_run_dir, raw_dir.mkdir, print_governor_state_loudly and
    # the RunManifest, so the refusal arrived having already created a run
    # directory and a manifest for a sweep that cannot start. The
    # image-BUILD stays where it is (it is per distro and genuinely belongs
    # in the loop); only the "is docker here at all" question moves, which
    # is the half that decides whether any cell can run.
    if not docker_available():
        print("ros2: docker is not available — install Docker Desktop / "
              "Docker Engine first. Refusing before creating a run "
              "directory or a manifest.", file=sys.stderr)
        return 3

    run_dir = resolve_run_dir(args)
    raw_dir = raw_dir_of(rep_dir_of(run_dir, args.rep))
    raw_dir.mkdir(parents=True, exist_ok=True)
    log_dir = raw_dir / "_logs"
    # Set via set_defaults / cmd_full — a forgotten field raises loudly.
    restriction = ambient_payload_restriction()
    if args.sizes is not None:
        sizes: Sequence[int] = args.sizes
    elif restriction is not None:
        sizes = restriction
        print(f"[restriction] CER_BENCH_PAYLOAD_SIZES — ros2 cells run "
              f"ONLY: {', '.join(map(str, restriction))} (in-place partial "
              f"re-sweep)")
    else:
        sizes = PAYLOAD_SIZES
    print_governor_state_loudly()
    manifest = RunManifest(run_dir, "ros2", args.variant, args.rep)
    if args.chrt in ("1", "both") and 1 not in chrt_modes:
        manifest.record_skip("*_chrt1", "chrt -f 80 unavailable on this "
                                        "host — chrt-on cells skipped")

    for distro in distros:
        ensure_docker_image(distro, force_rebuild=args.build_image)
        manifest.record_docker_image(image_tag(distro))

    n_ok = n_fail = n_skip = n_dns = 0
    try:
        for distro in distros:
            for chrt in chrt_modes:
                cells, skips = enumerate_ros2_cells(distro, chrt)
                if args.cells:
                    cells = [c for c in cells if args.cells in c.name]
                # Run order interleaves the rmw axis so
                # head-to-head arms run adjacently instead of
                # hours apart; enumeration/inventory order is untouched.
                cells = interleave_rmws(cells)
                print(f"\n############### ros2 distro={distro} chrt={chrt} "
                      f"variant={args.variant} rep={args.rep} "
                      f"({PLATFORM_LABEL}) — {len(cells)} cells, "
                      f"rmw-interleaved run order ###############")
                print_skip_inventory(skips)
                for s in skips:
                    manifest.record_skip(s.scope, s.reason)
                n_skip += len(skips)
                if any(c.shm == "no_shm" for c in cells):
                    warn_no_shm_host_sysctls()
                for cell in cells:
                    # Platform gate: large-UDP no_shm cells rely on host
                    # sysctls reachable only with Linux host networking.
                    if cell.shm == "no_shm" and not IS_LINUX:
                        reason = (f"no_shm + non-Linux host; rmem_max cannot "
                                  f"be raised from a {PLATFORM_LABEL} host")
                        print(f"  ! skip {cell.name} ({reason})")
                        manifest.record_skip(cell.name, reason)
                        n_skip += 1
                        continue
                    # Usage-pattern lanes: jazzy is the first-class
                    # (verified) distro; humble/lyrical lane cells are
                    # enumerated by design but UNVERIFIED until a sweep
                    # runs them — loud, so a first humble/lyrical
                    # failure reads as an unswept-distro finding, not a
                    # silent harness regression (see ros2/memo.md).
                    if (cell.rmw in ("stock", "composed")
                            and cell.distro != "jazzy"):
                        print(f"  ! UNVERIFIED lane cell {cell.name}: the "
                              f"stock/composed lanes are verified on jazzy "
                              f"only so far — this {cell.distro} cell has "
                              f"never been swept", file=sys.stderr)
                    print(f"  [cell] {cell.name}")
                    for payload in sizes:
                        print(f"   payload={payload}")
                        started = _now_iso()
                        outcome = run_ros2_cell_payload(
                            cell, payload, args.variant, raw_dir, log_dir)
                        manifest.record_cell(cell.name, payload, outcome,
                                             started)
                        if outcome == "ok":
                            n_ok += 1
                        elif outcome == "skip":
                            n_skip += 1
                        elif outcome == "did_not_sustain":
                            # A valid recorded outcome (fixed100 ladder
                            # exhausted; sidecar written, empty CSV row
                            # downstream) — counted separately, never a
                            # harness failure.
                            n_dns += 1
                        else:
                            n_fail += 1
    finally:
        manifest.finalize()

    print(f"\nros2 sweep: {n_ok} (cell,payload) ok, {n_fail} failed, "
          f"{n_skip} skipped"
          + (f", {n_dns} did-not-sustain (fixed100 ladder exhausted — "
             f"empty rows, no latency minted)" if n_dns else ""))
    return 0 if n_fail == 0 else 1

# ============================================================================
# Subcommand: compile-csv / plots
# ============================================================================

def cmd_compile_csv(args: argparse.Namespace) -> int:
    script = LATENCY_DIR / "compile_csv.py"
    cmd = [sys.executable, str(script)]
    if args.raw_dir:
        # Legacy explicit-raw-dir interface: one raw dir, single rep. The
        # run directory is NOT computed here, and that is the fix: this used
        # to resolve the default results/<machine-hash>-<date>-<variant>/
        # path and pass it as --out-dir even in this mode, so a legacy
        # `compile-csv --raw-dir X` wrote its CSVs into a run dir unrelated
        # to X while compile_csv.py documents (and implements) the raw-dir
        # default as X's PARENT — which is also where it reads the manifest
        # variant from, so the two halves of one invocation disagreed about
        # where the run lives. Computing it was harmful a second way:
        # resolve_run_dir needs a machine hash, and a host that cannot
        # produce one got a hard refusal on a path that never needed it.
        # Forward --out-dir only when the caller states one; otherwise
        # compile_csv.py applies its own documented default.
        cmd.extend(["--raw-dir", str(args.raw_dir)])
        if args.out_dir:
            cmd.extend(["--out-dir", str(args.out_dir)])
        # An explicit --variant has NO effect in this mode — compile_csv.py
        # takes the pacing variant from the raw dir's parent's run.json, and
        # since this branch no longer resolves a run dir there is nothing
        # left for the flag to steer. Dropping it silently is the mislabel
        # this seam exists to prevent: the variant decides the exact-sample
        # -count gate, so a caller who states the wrong one is asking for a
        # run to be checked against the wrong schedule. cmd_plots refuses
        # exactly this mismatch; so does this. (A MATCHING value is
        # accepted, and an unreadable manifest is not a refusal — the flag
        # simply had nothing to contradict.)
        if args.variant is not None:
            recorded = manifest_variant_of(Path(args.raw_dir).parent)
            if recorded is not None and recorded != args.variant:
                print(f"compile-csv: --variant {args.variant} but "
                      f"{Path(args.raw_dir).parent / MANIFEST_NAME} records "
                      f"variant {recorded!r} — the variant decides the "
                      f"exact-sample-count gate, so compiling this raw dir "
                      f"as {args.variant} would check it against the wrong "
                      f"schedule. Pass --variant {recorded}, or omit it to "
                      f"take the manifest's value.", file=sys.stderr)
                return 2
    else:
        run_dir = resolve_run_dir(args)
        cmd.extend(["--out-dir", str(Path(args.out_dir) if args.out_dir
                                    else run_dir)])
        # Run-dir interface: compile_csv discovers rep<k>/raw/ subdirs and
        # aggregates across reps (A1); a legacy rep-less dir (raw/ directly
        # under the run dir) is read as a single rep. The pacing variant is
        # read from the run.json manifest for the exact-count gate (A3).
        cmd.extend(["--run-dir", str(run_dir)])
    # compile_csv.py is strict by default (a missing / 0-sample /
    # wrong-sample-count payload row is a hard nonzero exit listing the bad
    # rows); this flag demotes those to loud warnings for deliberately
    # partial sweeps.
    if args.allow_partial:
        cmd.append("--allow-partial")
    return subprocess.run(cmd).returncode

def cmd_plots(args: argparse.Namespace) -> int:
    run_dir = resolve_run_dir(args)
    script = LATENCY_DIR / "plot.py"
    cmd = [sys.executable, str(script),
           "--results-dir", str(run_dir),
           "--out-dir", str(run_dir / "plots"),
           "--chrt", args.chrt]
    # The variant decides which workspace legs plot.py EXPECTS (split x
    # backtoback is never produced). Forward it ONLY when the caller stated
    # one: plot.py's --variant defaults to None and then adopts the run
    # dir's run.json manifest, which is authoritative — it is the file the
    # sweep wrote. Forwarding a parser default instead meant `plots
    # --run-dir <a fixed100 run>` sent "quiescent" and plot.py refused the
    # render as a mislabel, so that adoption branch was unreachable through
    # bench.py and every explicit --run-dir had to repeat a variant the run
    # dir already records. An explicit --variant is still cross-checked
    # against the manifest and still refused on a mismatch.
    if args.variant is not None:
        cmd.extend(["--variant", args.variant])
    for g in (args.group or ["all"]):
        cmd.extend(["--group", g])
    if args.skip_missing:
        cmd.append("--skip-missing")
    return subprocess.run(cmd).returncode

# ============================================================================
# Subcommand: list-cells (inventory print — runs nothing)
# ============================================================================

def cmd_list_cells(args: argparse.Namespace) -> int:
    chrt_modes = (0, 1) if args.chrt == "both" else (int(args.chrt),)
    distros = [args.distro] if args.distro else list(ROS2_DISTROS)
    total = 0

    print(f"# benches/latency cell inventory (variant={args.variant}; "
          f"enumeration only — nothing runs)")
    print("\nnative host lines:")
    for chrt in chrt_modes:
        for b in NATIVE_BENCHES:
            if chrt == 1 and not b.chrt_on:
                print(f"  ! skip {b.raw_prefix}_chrt1 (spin-bound bench — "
                      f"chrt0 only by design)")
                continue
            print(f"  {b.raw_prefix}_chrt{chrt}  [{b.bin_name}"
                  f"{' + pong subprocess' if b.spawns_pong else ''}]")
            total += 1

    print("\nworkspace legs (real `cerulion graph run`; type-class axis "
          "variable|pod — METHODOLOGY § 'The type-class axis'):")
    for chrt in chrt_modes:
        for leg in WORKSPACE_LEGS:
            for msg in WORKSPACE_MSG_CLASSES:
                reason = workspace_leg_skip_reason(leg, args.variant)
                name = workspace_raw_name(leg, chrt, msg)
                if reason is not None:
                    print(f"  ! skip {name} ({reason})")
                    continue
                print(f"  {name}")
                total += 1

    for distro in distros:
        for chrt in chrt_modes:
            cells, skips = enumerate_ros2_cells(distro, chrt)
            print(f"\nros2 cells distro={distro} chrt={chrt}: {len(cells)}")
            for c in cells:
                print(f"  {c.name}")
            print_skip_inventory(skips)
            total += len(cells)

    print(f"\ntotal enumerated cells: {total} "
          f"(payload sweep: {len(PAYLOAD_SIZES)} sizes per cell)")
    return 0

# ============================================================================
# Subcommand: full
# ============================================================================

def cmd_full(args: argparse.Namespace) -> int:
    """native + workspace + ros2 (× --reps, round-robin) + compile-csv + plots.

    Uses argparse.Namespace to build per-subcommand arg objects — any field a
    subcommand reads but cmd_full forgets to set raises AttributeError at the
    call site (loud), not a silent default.

    --reps N: the WHOLE matrix runs round-robin per rep — rep 1
    runs native + workspace + ros2 end to end, then rep 2 re-runs the whole
    matrix, and so on — which is the §10 same-window interleaving mechanism
    at rep granularity (every cell's k-th rep is measured in the same
    campaign window as every other cell's k-th rep). Each rep lands in its
    own rep<k>/ subdir; compile-csv aggregates across reps (median-of-rep
    p50s headline + min/max rep spread) and plot.py shades the spread."""
    check_ambient_pacing(args.variant)
    refuse_ambient_sample_overrides("full", args.variant)
    refuse_foreign_cerulion_override("full")
    if args.reps < 1:
        raise SystemExit(f"--reps must be >= 1, got {args.reps}")
    # BEFORE resolve_run_dir, not after. The three sweeps this dispatches
    # into each gate their own posture, but they do so AFTER cmd_full has
    # already resolved (and, on the default path, named) a run directory —
    # so `full` announced a run dir and then refused, which is the exact
    # ordering that lets a directory look like this invocation's when it
    # holds an earlier one's rows. Gating here means a refused `full`
    # touches nothing and names nothing.
    set_dma_basis("host")
    require_dma_posture_deliverable("full sweep")
    # ...and the env contract, EXPLICITLY. My first attempt at this leaned on
    # require_dma_posture_deliverable above to validate CER_BENCH_DMA_LOCK on
    # its way through dma_lock_enabled. Measured, it does not: that function
    # opens with `if not IS_LINUX: return`, so on a non-Linux host it never
    # reaches the parse, and `full` printed its "=== full bench ... -> <dir>"
    # banner and only then exited 1 from the first sweep it dispatched into.
    # Every other command validates via announce_posture; cmd_full is the one
    # that does not call it, which is why the gap was here and only here.
    #
    # Relying on a gate's side effect made the validation platform-dependent
    # and invisible. Calling the parsers directly makes it neither.
    dma_lock_enabled()
    ambient_payload_restriction()
    run_dir = resolve_run_dir(args)
    print(f"=== full bench variant={args.variant} reps={args.reps} on "
          f"{PLATFORM_LABEL} → {_short(run_dir)} ===")

    sweep_rcs: List[int] = []
    for rep in range(1, args.reps + 1):
        if args.reps > 1:
            print(f"\n===== rep {rep}/{args.reps} (round-robin: the whole "
                  f"matrix per rep — §10 interleaving at rep scale) =====")
        rc1 = cmd_native(argparse.Namespace(
            variant=args.variant, chrt=args.chrt, bin=None,
            run_dir=str(run_dir), rep=rep))
        rc2 = cmd_workspace(argparse.Namespace(
            variant=args.variant, chrt=args.chrt, leg="both",
            msg_class="both", sizes=None,
            run_dir=str(run_dir), rep=rep))
        rc3 = cmd_ros2(argparse.Namespace(
            variant=args.variant, distro=args.distro, chrt=args.chrt,
            cells=None, build_image=args.build_image and rep == 1,
            sizes=None, run_dir=str(run_dir), rep=rep))
        sweep_rcs.extend((rc1, rc2, rc3))
        print(f"\n--- rep {rep}/{args.reps} summary: native={rc1} "
              f"workspace={rc2} ros2={rc3} ---")

    # Strict compile (allow_partial=False): a full sweep must yield complete
    # per-cell payload rows in EVERY rep at the schedule's exact measured
    # counts; structural skips produce no .bins at all (no discovered
    # prefix), so they never trip the strictness — only genuine per-payload
    # failures do, and those already made the sweep rcs nonzero.
    rc4 = cmd_compile_csv(argparse.Namespace(
        variant=args.variant, run_dir=str(run_dir), raw_dir=None,
        out_dir=None, allow_partial=False))
    # skip-missing here (and only here): the SAME invocation printed its own
    # '! skip' inventory above, so absent CSVs for skipped cells are already
    # accounted for loudly. The standalone `plots` subcommand stays strict.
    rc5 = cmd_plots(argparse.Namespace(
        variant=args.variant, run_dir=str(run_dir), group=["all"],
        chrt=args.chrt, skip_missing=True))

    print(f"\n=== full bench summary: sweeps({args.reps} rep(s))="
          f"{max(sweep_rcs)} csv={rc4} plots={rc5} ===")
    return max(*sweep_rcs, rc4, rc5)

# ============================================================================
# Subcommand: smoke (ported from the earlier gate, machine-hash keyed)
# ============================================================================

# Per-(cell, payload) sample budget for the smoke gate. EVERY smoke cell
# retains SMOKE_MEASURED measured samples after SMOKE_WARMUP warmup
# iterations, whichever spelling carries the count to it:
#   - paced native + workspace cells receive CER_BENCH_SMOKE_N =
#     SMOKE_TOTAL (a TOTAL — the schedules carve warmup = max(N // 10, 1)
#     out of it, see quiescent_schedule / native smoke_override /
#     run_workspace.sh), so they measure SMOKE_TOTAL − SMOKE_TOTAL // 10;
#   - backtoback + ROS 2 cells receive explicit CER_BENCH_TARGET_SAMPLES =
#     SMOKE_MEASURED / CER_BENCH_WARMUP = SMOKE_WARMUP, which under the G1
#     contract mean MEASURED samples + warmup on top.
# The three constants are derived from one another so the two spellings
# cannot drift apart (pinned by check_percentile_parity.py).
SMOKE_TOTAL = 1000
SMOKE_WARMUP = max(SMOKE_TOTAL // 10, 1)
SMOKE_MEASURED = SMOKE_TOTAL - SMOKE_WARMUP

# --capture-baseline reps: a baseline captured
# from ONE smoke rep inherits the measured ~1.4x rep-to-rep p50 wobble, so
# with the ±2x gate bounds a lucky-rep baseline raises the worst-case
# detection threshold to ~2.7x. The capture runs its cells 3x (each rep in
# its own rep<k>/ dir) and records bounds around the MEDIAN of the three
# p50s. The gate path stays single-rep (it is a catastrophe detector).
SMOKE_BASELINE_REPS = 3

SMOKE_PAYLOAD_SMALL = 64
SMOKE_PAYLOAD_LARGE = 1048576

RANGES_SCHEMA_VERSION = 2   # v2: baselines are keyed by pacing variant

@dataclass(frozen=True)
class SmokeCell:
    """One smoke-gate cell: where it runs + which payloads are gated."""
    kind: str                          # "native" | "workspace" | "ros2"
    cell_name: str                     # raw cell name == .bin filename prefix
    payloads: tuple                    # payload sizes whose p50 is gated
    chrt: int = 0
    native: Optional[NativeBench] = None
    leg: Optional[str] = None
    ros2: Optional[Ros2Cell] = None
    gate_reason: Optional[str] = None  # pre-computed skip reason (or None)
    # TYPE-CLASS axis, WORKSPACE cells only (a ROS 2 cell carries its
    # class in ros2.msg): variable (the incumbent Image legs) | pod (the
    # fixed-PodPayload twin). Threaded into cell_name (workspace_raw_name)
    # AND run_workspace_leg's CER_BENCH_MSG export — the two must agree,
    # or the gate reads one class's .bin under the other's baseline key.
    # None on every other kind: a default of "variable" here put a second
    # spelling of the class on a ros2 image cell, disagreeing with
    # ros2.msg, with this one silently ignored.
    msg: Optional[str] = None

    def __post_init__(self) -> None:
        if (self.msg is not None) != (self.kind == "workspace"):
            raise ValueError(
                f"SmokeCell.msg is the workspace class carrier: kind="
                f"{self.kind!r} must leave it None (a ros2 cell carries "
                f"its class in ros2.msg), got {self.msg!r}")
        if self.msg is not None and self.msg not in WORKSPACE_MSG_CLASSES:
            raise ValueError(
                f"SmokeCell.msg={self.msg!r} is not one of "
                f"{WORKSPACE_MSG_CLASSES}")

def enumerate_smoke_cells(variant: str = "quiescent") -> List[SmokeCell]:
    """Curated smoke subset over the line inventory.

    Every ROS 2 entry is asserted against the REAL enumeration so a future
    matrix change fails loudly instead of gating a phantom cell."""
    both = (SMOKE_PAYLOAD_SMALL, SMOKE_PAYLOAD_LARGE)
    small = (SMOKE_PAYLOAD_SMALL,)

    def native(bin_name: str, chrt: int, payloads: tuple) -> SmokeCell:
        bench = next(b for b in NATIVE_BENCHES if b.bin_name == bin_name)
        return SmokeCell(kind="native", cell_name=f"{bench.raw_prefix}_chrt{chrt}",
                         payloads=payloads, chrt=chrt, native=bench)

    def workspace(leg: str, chrt: int, payloads: tuple,
                  msg: str = "variable") -> SmokeCell:
        if msg not in WORKSPACE_MSG_CLASSES:
            raise RuntimeError(
                f"smoke workspace cell ({leg}, {msg}) names a type class "
                f"outside WORKSPACE_MSG_CLASSES {WORKSPACE_MSG_CLASSES}")
        return SmokeCell(kind="workspace",
                         cell_name=workspace_raw_name(leg, chrt, msg),
                         payloads=payloads, chrt=chrt, leg=leg, msg=msg,
                         gate_reason=workspace_leg_skip_reason(leg, variant))

    def ros2(distro: str, rmw: str, shm: str, recv: str, qos: str, chrt: int,
             payloads: tuple, msg: str = "pod") -> SmokeCell:
        cell = Ros2Cell(distro, rmw, shm, recv, qos, chrt, msg=msg)
        real, _skips = enumerate_ros2_cells(distro, chrt)
        if cell not in real:
            raise RuntimeError(
                f"smoke cell {cell.name} is not in the real ROS 2 "
                f"enumeration — update enumerate_smoke_cells() to match "
                f"enumerate_ros2_cells()")
        return SmokeCell(kind="ros2", cell_name=cell.name, payloads=payloads,
                         chrt=chrt, ros2=cell, gate_reason=None)

    # NOTE (2026-08-13): the curated set changed — the rmw_cerulion cell was
    # dropped (rmw_cerulion is not part of this suite), the
    # 4/16 MiB quiescent schedule was extended (tail-resolved counts), and the
    # workspace headline cell swapped default → split (decided
    # 2026-08-13: split replaces the flagless default while the park-wake bug
    # inflates it). Per-host baselines captured before this date gate a
    # different subset under different counts: re-run
    # `smoke --capture-baseline` per host.
    #
    # NOTE (2026-08-14): the usage-pattern lanes added two jazzy cells
    # (stock + composed ipcon — ros2/memo.md). Hosts with
    # older baselines report them as loud '[skip] ... no range in
    # baseline' lines until --capture-baseline is re-run.
    #
    # NOTE (2026-09-02): the type-class axis (METHODOLOGY §18) added one
    # representative per stack — the headline leg's fixed-POD twin
    # (cerulion_workspace_split_pod_chrt0, both payloads) and the hero
    # rmw's image twin (jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0,
    # small). Without them the full sweep ran both classes but smoke +
    # baseline capture exercised only the incumbent class on each
    # stack, so a regression on a new class passed the gate. Same
    # recapture rule as above: a pre-axis baseline reports the new
    # cells as loud '[skip]' lines (never a silent pass, never a
    # fabricated range) until --capture-baseline is re-run on that host.
    return [
        # Native lines (cerulion_user retired)
        native("raw_iceoryx2_round_trip", 0, both),       # iox2_chrt0 (floor)
        native("zenoh_shm_round_trip", 0, small),         # zenoh_shm_chrt0
        # Workspace legs — split is THE headline row (declared 2-group
        # process_groups mp; the flagless
        # default is retired while the park-wake bug inflates it)
        workspace("split", 0, both),
        workspace("mono", 0, small),
        # Type-class twin of the headline row (METHODOLOGY §18): the
        # fixed-PodPayload split leg, gated at both payloads — 1 MiB is
        # where the class hypothesis bites, 64 B pins the small end.
        workspace("split", 0, both, msg="pod"),
        # ROS 2 hero + zero-copy lane
        ros2("jazzy", "cyclonedds", "shm", "rclcpp", "be1", 0, both),
        ros2("jazzy", "cyclonedds", "shm", "loan", "be1", 0, small),
        # Type-class twin of the hero cell (METHODOLOGY §18): the real
        # sensor_msgs/Image on the same rmw × shm × rclcpp × be1 axis.
        ros2("jazzy", "cyclonedds", "shm", "rclcpp", "be1", 0, small,
             msg="image"),
        # chrt-on coverage
        ros2("jazzy", "cyclonedds", "shm", "rclcpp", "be1", 1, small),
        # Distro coverage
        ros2("humble", "cyclonedds", "shm", "rclcpp", "be1", 0, small),
        ros2("lyrical", "cyclonedds", "shm", "rclcpp", "be1", 0, small),
        # Usage-pattern lanes (jazzy first-class): the
        # zero-config stock lane + the composed lane's IPC-on shade
        ros2("jazzy", "stock", "stock", "rclcpp", "stock", 0, small),
        ros2("jazzy", "composed", "ipcon", "rclcpp", "stock", 0, small),
    ]

# ---------------------------------------------------------------- ranges YAML

# Constrained, zero-dependency YAML reader/writer for expected-ranges.yaml.
# Deliberately NOT a general YAML parser: the schema is fixed (nested string-
# keyed maps, int / quoted-string scalars, inline [int, int] lists, inline {}
# empty maps, full-line comments, 2-space indents). PyYAML is avoided so
# bench.py stays stdlib-only on fresh hosts.

def _expected_ranges_header() -> str:
    return f"""# expected-ranges.yaml — per-host p50 round-trip baselines for the smoke gate
# (`python3 benches/latency/bench.py smoke`).
#
# Schema (schema_version {RANGES_SCHEMA_VERSION}):
#
#   schema_version: {RANGES_SCHEMA_VERSION}
#   hosts:
#     <machine_hash>:                  # 16-char sha256 prefix — see
#                                      # scripts/benchmarks/lib/machine_hash.sh
#                                      # (compute_live_machine_hash)
#       machine_hash: <machine_hash>   # repeated for self-describing entries
#       measured_on_git_sha: <40-char git sha the baseline was captured at>
#       notes: "<free-form provenance string>"
#       rtt_p50_ns:
#         <pacing>:                    # quiescent | fixed100 | backtoback —
#                                      # the CER_BENCH_PACING modes have
#                                      # different latency distributions, so
#                                      # baselines are keyed per pacing variant
#           <cell_name>:               # raw cell name, e.g. iox2_chrt0
#             <payload_bytes>: [<min_ns>, <max_ns>]
#
# Multiple hosts coexist under `hosts:` — the smoke gate looks up its own
# machine_hash + pacing variant and compares measured p50s against
# [min, max]. p50 > max is FAIL_HIGH (regression); p50 < min is FAIL_LOW
# (suspiciously fast — usually a broken measurement, e.g. the responder
# never engaged).
#
# Entries are written ONLY by `bench.py smoke --capture-baseline`, which
# measures real p50s on the current host and records [p50/2, p50*2] safety
# bounds. NEVER hand-write latency numbers into this file (the no-fake-data
# rule in AGENTS.md). The hosts map below is intentionally empty
# until a real machine captures a baseline.
#
# This file is parsed by the constrained YAML reader in
# benches/latency/bench.py (zero-dependency). Stick to the schema above:
# 2-space indents, full-line comments only, inline `[min, max]` integer
# lists, inline `{{}}` for empty maps.
"""

def _parse_ranges_scalar(s: str, where: str):
    """Parse a scalar value in the constrained expected-ranges subset."""
    if s == "{}":
        return {}
    if s.startswith("[") and s.endswith("]"):
        items = [x.strip() for x in s[1:-1].split(",") if x.strip()]
        if not all(re.fullmatch(r"-?\d+", x) for x in items):
            raise ValueError(f"{where}: lists must contain only integers: {s!r}")
        return [int(x) for x in items]
    if len(s) >= 2 and s.startswith('"') and s.endswith('"'):
        return s[1:-1]
    # 16-char lowercase-hex strings are machine hashes — an all-decimal hash
    # (~1 in 1845) must stay a str or validate_expected_ranges rejects a
    # valid file. Checked BEFORE the integer branch.
    if re.fullmatch(r"[0-9a-f]{16}", s):
        return s
    if re.fullmatch(r"-?\d+", s):
        return int(s)
    return s  # bare string (git shas, notes, etc.)

def parse_expected_ranges(text: str, source: str) -> dict:
    """Parse expected-ranges.yaml (constrained subset — see header doc)."""
    entries: List[tuple] = []  # (line_no, depth, key, raw_value)
    for line_no, raw in enumerate(text.splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        if indent % 2 != 0:
            raise ValueError(f"{source}:{line_no}: indentation must be a "
                             f"multiple of 2 spaces")
        body = raw.strip()
        key, sep, value = body.partition(":")
        if not sep or not key.strip():
            raise ValueError(f"{source}:{line_no}: expected 'key:' or "
                             f"'key: value', got {body!r}")
        entries.append((line_no, indent // 2, key.strip(), value.strip()))

    root: dict = {}
    stack: List[tuple] = [(-1, root)]  # (depth, mapping)
    for line_no, depth, key, value in entries:
        while stack and stack[-1][0] >= depth:
            stack.pop()
        if not stack:
            raise ValueError(f"{source}:{line_no}: bad indentation structure")
        parent = stack[-1][1]
        where = f"{source}:{line_no}"
        if key in parent:
            raise ValueError(f"{where}: duplicate key {key!r}")
        if value == "":
            child: dict = {}
            parent[key] = child
            stack.append((depth, child))
        else:
            parent[key] = _parse_ranges_scalar(value, where)
    return root

def validate_expected_ranges(data: dict, source: str) -> None:
    """Loud structural validation of a parsed expected-ranges document."""
    if data.get("schema_version") != RANGES_SCHEMA_VERSION:
        raise ValueError(f"{source}: schema_version must be "
                         f"{RANGES_SCHEMA_VERSION}, got {data.get('schema_version')!r}")
    hosts = data.get("hosts")
    if not isinstance(hosts, dict):
        raise ValueError(f"{source}: 'hosts' must be a map "
                         f"(use 'hosts: {{}}' when empty)")
    for mh, entry in hosts.items():
        ctx = f"{source}: hosts.{mh}"
        if not re.fullmatch(r"[0-9a-f]{16}", str(mh)):
            raise ValueError(f"{ctx}: key must be a 16-char lowercase hex "
                             f"machine hash")
        if not isinstance(entry, dict) or not isinstance(entry.get("rtt_p50_ns"), dict):
            raise ValueError(f"{ctx}: entry must be a map with an "
                             f"'rtt_p50_ns' map")
        if entry.get("machine_hash") != mh:
            raise ValueError(f"{ctx}: machine_hash field must equal the entry key")
        for pacing, cells in entry["rtt_p50_ns"].items():
            if pacing not in PACING_VARIANTS:
                raise ValueError(
                    f"{ctx}: rtt_p50_ns.{pacing}: pacing key must be one of "
                    f"{', '.join(PACING_VARIANTS)}")
            if not isinstance(cells, dict):
                raise ValueError(f"{ctx}: rtt_p50_ns.{pacing} must be a map "
                                 f"of cell -> payload ranges")
            for cell, payloads in cells.items():
                if not isinstance(payloads, dict):
                    raise ValueError(f"{ctx}: rtt_p50_ns.{pacing}.{cell} must "
                                     f"be a map of payload -> [min, max]")
                for payload, rng in payloads.items():
                    pctx = f"{ctx}: rtt_p50_ns.{pacing}.{cell}.{payload}"
                    if not re.fullmatch(r"\d+", str(payload)):
                        raise ValueError(f"{pctx}: payload key must be an "
                                         f"integer byte count")
                    if (not isinstance(rng, list) or len(rng) != 2
                            or not all(isinstance(x, int) for x in rng)
                            or rng[0] < 0 or rng[0] > rng[1]):
                        raise ValueError(f"{pctx}: range must be [min, max] "
                                         f"with 0 <= min <= max")

def emit_expected_ranges(data: dict) -> str:
    """Serialize an expected-ranges document (inverse of parse_expected_ranges).

    Rewrites the documentation header from the template; full-line comments
    outside the header are not preserved. Host entries other than the one
    being updated survive verbatim through the parse → emit round-trip."""
    out: List[str] = [_expected_ranges_header().rstrip("\n")]
    out.append(f"schema_version: {data['schema_version']}")
    hosts = data["hosts"]
    if not hosts:
        out.append("hosts: {}")
    else:
        out.append("hosts:")
        for mh in sorted(hosts):
            entry = hosts[mh]
            out.append(f"  {mh}:")
            out.append(f"    machine_hash: {entry['machine_hash']}")
            out.append(f"    measured_on_git_sha: {entry['measured_on_git_sha']}")
            notes = str(entry.get("notes", "")).replace('"', "'")
            out.append(f'    notes: "{notes}"')
            out.append("    rtt_p50_ns:")
            for pacing in sorted(entry["rtt_p50_ns"]):
                out.append(f"      {pacing}:")
                for cell, payloads in entry["rtt_p50_ns"][pacing].items():
                    out.append(f"        {cell}:")
                    for payload, rng in payloads.items():
                        out.append(f"          {payload}: [{rng[0]}, {rng[1]}]")
    return "\n".join(out) + "\n"

# ---------------------------------------------------------------- measurement

def read_p50_ns(bin_path: Path) -> int:
    """p50 over a raw .bin sample dump (u64-LE wall-ns per sample).

    Same linear-interpolated quantile as compile_csv.py::percentile so the
    smoke gate and the full sweep report identical p50s for the same bin —
    the parity is pinned by check_percentile_parity.py."""
    blob = bin_path.read_bytes()
    if len(blob) < 16 or len(blob) % 8 != 0:
        raise SmokeSetupError(f"{bin_path}: malformed sample dump "
                              f"({len(blob)} bytes; need a multiple of 8, >= 16)")
    samples = sorted(int.from_bytes(blob[i:i + 8], "little")
                     for i in range(0, len(blob), 8))
    rank = 0.5 * (len(samples) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(samples) - 1)
    frac = rank - lo
    return int(samples[lo] + frac * (samples[hi] - samples[lo]))

def median_int(values: Sequence[int]) -> int:
    """Linear-interpolated median over a small int list — the same core
    formula as read_p50_ns, for the A4 median-of-rep-p50s baseline."""
    s = sorted(values)
    if not s:
        raise ValueError("median_int over an empty sequence")
    rank = 0.5 * (len(s) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(s) - 1)
    frac = rank - lo
    return int(s[lo] + frac * (s[hi] - s[lo]))

def smoke_sample_env(variant: str) -> Tuple[Dict[str, str], Dict[str, str]]:
    """(ros2_env, native_workspace_overrides) — the sample-count exports
    the smoke dispatcher hands its cells. ONE function so the two
    spellings cannot drift (pinned by check_percentile_parity.py).

    ros2_env — G1 semantics: MEASURED samples (+ warmup on top), so the
    ROS 2 smoke cells measure SMOKE_MEASURED exactly, the same count the
    SMOKE_N-driven native/workspace cells retain (legacy spellings carry
    the same values — see docker_args_for_cell).

    overrides — native + workspace counts travel via os.environ
    (run_native_bench / run_workspace_leg copy it). Paced variants
    (quiescent AND fixed100): CER_BENCH_SMOKE_N=SMOKE_TOTAL reaches the
    Rust schedules (native binaries), the workspace runner, and the
    Python mirrors here — each carves warmup = max(N // 10, 1) out of the
    TOTAL, and the pacing (rates, fixed100's fallback ladder) stays live
    under smoke counts. Backtoback: explicit CER_BENCH_TARGET_SAMPLES /
    CER_BENCH_WARMUP (MEASURED + warmup, G1) — the same retained count."""
    ros2_env = {"CER_BENCH_TARGET_SAMPLES": str(SMOKE_MEASURED),
                "CER_BENCH_WARMUP": str(SMOKE_WARMUP),
                "TARGET_SAMPLES": str(SMOKE_MEASURED),
                "WARMUP": str(SMOKE_WARMUP)}
    overrides = ({"CER_BENCH_SMOKE_N": str(SMOKE_TOTAL)}
                 if variant != "backtoback"
                 else {"CER_BENCH_TARGET_SAMPLES": str(SMOKE_MEASURED),
                       "CER_BENCH_WARMUP": str(SMOKE_WARMUP)})
    return ros2_env, overrides

def _run_smoke_cells(variant: str, runnable: List[SmokeCell],
                     raw_dir: Path) -> None:
    """Run every runnable smoke cell, writing .bins into raw_dir.

    Raises SmokeSetupError on any crash/build/setup failure (exit 3)."""
    log_dir = raw_dir / "_logs"
    ros2_env, overrides = smoke_sample_env(variant)
    # A NON-paced smoke run additionally CLEARS an ambient CER_BENCH_SMOKE_N.
    # It is not merely unused there: the Rust bench_plan's Backtoback arm
    # consults smoke_override() FIRST, and run_workspace.sh applies its
    # SMOKE_N block AFTER the pacing branch, so a stray export would win on
    # those two runners while the explicit counts govern the ROS 2 ones —
    # unequal populations again, by another door. cmd_smoke is the one
    # producing path that cannot use refuse_ambient_sample_overrides (it
    # owns the overrides), so it neutralises the variable itself.
    cleared = () if "CER_BENCH_SMOKE_N" in overrides else ("CER_BENCH_SMOKE_N",)
    saved = {k: os.environ.get(k) for k in (*overrides, *cleared)}
    # Set the flag BEFORE injecting the override into os.environ so
    # refuse_ambient_sample_overrides can never see a window where the override is
    # live but _SMOKE_ACTIVE is still False.
    global _SMOKE_ACTIVE
    _SMOKE_ACTIVE = True
    os.environ.update(overrides)
    for k in cleared:
        os.environ.pop(k, None)
    try:
        if any(c.kind == "native" for c in runnable):
            ensure_built(NATIVE_DIR)
        for cell in runnable:
            print(f"\n[smoke] {cell.cell_name} ({cell.kind})")
            if cell.kind == "native":
                # Native binaries sweep all payload sizes internally; only
                # cell.payloads are gated.
                if not run_native_bench(cell.native, cell.chrt, variant,
                                        raw_dir, log_dir):
                    raise SmokeSetupError(
                        f"native cell {cell.cell_name} failed — see "
                        f"{log_dir / (cell.cell_name + '.log')}")
            elif cell.kind == "workspace":
                outcome = run_workspace_leg(cell.leg, cell.chrt, variant,
                                            raw_dir, log_dir,
                                            list(cell.payloads),
                                            msg=cell.msg)
                if outcome != "ok":
                    # A runner-side rc=77 here means the smoke partition
                    # missed a structural gate — setup bug, so exit 3, not
                    # a quiet skip.
                    raise SmokeSetupError(
                        f"workspace cell {cell.cell_name} {outcome}ed — see "
                        f"{log_dir / (cell.cell_name + '.log')}")
            else:
                ensure_docker_image(cell.ros2.distro)
                for payload in cell.payloads:
                    outcome = run_ros2_cell_payload(
                        cell.ros2, payload, variant, raw_dir, log_dir,
                        extra_env=ros2_env)
                    if outcome != "ok":
                        # A did_not_sustain here is smoke-fatal too: the
                        # gate needs a measured p50 to compare, and the
                        # curated subset was picked to sustain trivially.
                        raise SmokeSetupError(
                            f"ros2 cell {cell.cell_name} payload {payload} "
                            f"ended '{outcome}' — see {log_dir}")
    except SystemExit as e:
        # FIVE sources, resolved from the call graph rather than listed by
        # memory — the list has now been wrong twice, at two and at three:
        #   ensure_built, ensure_docker_image      raise directly
        #   run_workspace_leg                      raises directly (its bash
        #                                          preflight AND its "could
        #                                          not start bash" spawn)
        #   run_native_bench                       via the ENV gates it reads
        #                                          per cell: usage_enabled,
        #                                          native_timeout_s, and the
        #                                          re-checked
        #                                          ambient_payload_restriction
        #                                          / dma_lock_enabled
        #   run_ros2_cell_payload                  via usage_enabled
        # The last two matter out of proportion to their obscurity: they are
        # why a bad CER_BENCH_USAGE or CER_BENCH_NATIVE_TIMEOUT_S surfaces as
        # 3 and never as 1, which both published exit tables got wrong.
        raise SmokeSetupError(str(e)) from e
    finally:
        _SMOKE_ACTIVE = False
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

# "This host cannot be gated yet", distinct from "something is wrong".
# A baseline is per (machine_hash, variant) and is captured on the machine,
# so a fresh clone CANNOT have one — that is the normal first-run state, not
# a failure, and it shared exit 3 with genuine setup failures. A caller that
# wanted to run its other gates anyway had to either tolerate 3 (which also
# tolerates a build failure) or re-derive the baseline question for itself.
# Neither is acceptable, so the gate reports the distinction instead.
SMOKE_RC_NO_BASELINE = 4

def cmd_smoke(args: argparse.Namespace) -> int:
    """Low-n smoke gate: run the curated cell subset, compare p50s to the
    current host's entry (for this pacing variant) in expected-ranges.yaml.

    Exit codes: 0 = all gated cells in range;
    1 = an env-contract violation — a malformed CER_BENCH_PACING /
    CER_BENCH_DMA_LOCK / CER_BENCH_PAYLOAD_SIZES, or an EMPTY
    CARGO_TARGET_DIR (cargo refuses that configuration outright, so
    nothing here can be built or dated under it; refused at dispatch,
    ahead of every one of these). Python's code for an
    uncaught SystemExit(<message>), so it sits outside the deliberate 2/3/4
    series. Their VALIDATION runs ahead of the classification below while
    their consequences run after it, which is what keeps a malformed value
    at 1 on every path: a well-formed-but-restrictive one is 4 here and 3
    once a baseline exists. Answering a typo with 4 would be silently
    swallowed by run_benchmarks.sh as "no baseline, skip the gate";
    2 = any FAIL_HIGH/FAIL_LOW;
    3 = crash/build/setup failure, OR a cell that RAN with no range in the
    baseline (a hole in the gate — see the `ungated` arm);
    4 (`SMOKE_RC_NO_BASELINE`) = this host has no baseline for this variant,
    so there is nothing to gate against. FOUR shapes reach it: the file
    absent; the shipped `hosts: {}`; a file holding only OTHER hosts'
    entries (the designed steady state of a shared ranges file); and this
    host's own entry carrying no ranges for this variant. They share the
    code but NOT the remedy, so the caller is told which: only the third can
    be identity drift (the only shape where this host's key is absent), and
    only it warns about drift and prints the CPU posture. Distinct from 3 so
    a caller can run its other gates without also tolerating a build
    failure. The no-baseline check runs
    BEFORE any build or bench, so a missing baseline costs seconds, not
    minutes."""
    check_ambient_pacing(args.variant)
    # VALIDATE the rest of the env contract here too -- parsing only, no
    # consequence. Both parsers raise SystemExit(<message>) on a malformed
    # value, which Python reports as exit 1.
    #
    # This is the earlier reorder's own regression, and it is worth stating
    # exactly: moving `announce_posture` and `ambient_payload_restriction`
    # below the classification moved their PARSING with them, so on any host
    # with no recorded baseline a malformed CER_BENCH_DMA_LOCK or
    # CER_BENCH_PAYLOAD_SIZES stopped being an exit-1 contract violation and
    # became a 4. And 4 is the code `run_benchmarks.sh` deliberately
    # SWALLOWS ("no baseline, skip the gate, keep going"), so a typo in an
    # exported variable silently continued a run instead of aborting it --
    # strictly worse than the 3 that reorder set out to fix.
    #
    # The split that fixes it is the same one that reorder drew for
    # check_ambient_pacing: a malformed value is a malformed ASK, answered
    # the same way on every path, while the CONSEQUENCES of a well-formed
    # value (the posture banner, the deliverable refusal, the curated-subset
    # refusal) are about a measurement and stay below the classification.
    #
    # So the three cases are now distinct, which neither the old order nor
    # the reorder's produced:
    #   CER_BENCH_PAYLOAD_SIZES=notanint  -> 1 on every path (malformed ask)
    #   CER_BENCH_PAYLOAD_SIZES=64, no baseline
    #                                     -> 4 (nothing to gate, as the reorder intended)
    #   CER_BENCH_PAYLOAD_SIZES=64, baseline on file
    #                                     -> 3 (a curated subset would thin)
    dma_lock_enabled()
    ambient_payload_restriction()
    # An exported CERULION= is a malformed ASK, not a property of the
    # measurement, so it is answered here — the same exit 1 on every path,
    # above the no-baseline classification that would otherwise swallow it
    # as a 4 (`run_benchmarks.sh` deliberately continues past a 4).
    refuse_foreign_cerulion_override("smoke")
    variant = args.variant

    # IDENTITY ONLY, ahead of every data-producing preflight. Everything
    # between here and the classification below either validates the ASK
    # (check_ambient_pacing, above) or answers "which machine is this" —
    # nothing that presumes a cell will run, and nothing with a side effect.
    #
    # The order matters and used to be wrong. The posture banner, the
    # payload-restriction refusal and the memlock refusal all ran FIRST and
    # all return 3, so on a host with a low memlock limit a fresh clone with
    # `hosts: {}` got 3 — a setup failure — where the README, the wrapper and
    # this function's own docstring all promise 4. `run_benchmarks.sh` then
    # ABORTED the whole run instead of skipping a gate it has no baseline
    # for. Those three are about the quality of a measurement that, in these
    # shapes, is never taken: there is no row to mislabel, no curated subset
    # to thin, and no zenoh cell to wedge.
    #
    # They keep their exit 3 for every run that DOES proceed — a capture
    # (`--capture-baseline` skips the classification by design) and any host
    # whose baseline is on file. That is the whole point of the split: a
    # memlock limit is a real setup failure when a cell is about to run, and
    # irrelevant when none is.
    try:
        mh = compute_machine_hash()
    except SmokeSetupError as e:
        print(f"smoke: {e}", file=sys.stderr)
        return 3

    data = {"schema_version": RANGES_SCHEMA_VERSION, "hosts": {}}
    if RANGES_PATH.exists():
        try:
            data = parse_expected_ranges(RANGES_PATH.read_text(), str(RANGES_PATH))
            validate_expected_ranges(data, str(RANGES_PATH))
        except ValueError as e:
            print(f"smoke: {e}", file=sys.stderr)
            return 3
    elif not args.capture_baseline:
        # The posture rides this exit too. It is the shape most likely to be
        # followed immediately by a --capture-baseline, and a capture taken
        # on a powersave governor bakes the wrong posture into the baseline
        # every later run is gated against. Printed BEFORE the instruction
        # that would act on it, for the same reason.
        print_governor_state_loudly()
        print(f"smoke: {_short(RANGES_PATH)} not found — nothing has been "
              f"captured on this machine or any other. Run "
              f"`python3 benches/latency/bench.py smoke --variant {variant} "
              f"--capture-baseline` on this machine first", file=sys.stderr)
        return SMOKE_RC_NO_BASELINE

    host_entry = data["hosts"].get(mh)
    host_ranges = ((host_entry or {}).get("rtt_p50_ns", {}) or {}).get(variant)
    # Falsy, not `is None`: an entry whose variant map is present but EMPTY
    # carries no range for any cell, so there is nothing to gate against —
    # the same fact, and it must take the same early return, or the
    # docstring's "costs seconds, not minutes" promise breaks and a full
    # cargo build runs before the gate can say so.
    # `reproduce.sh::baseline_present` mirrors this predicate exactly; the
    # two are pinned together by check_smoke_exit_contract.
    if not host_ranges and not args.capture_baseline:
        print(f"smoke: no {variant} baseline for this host (machine_hash {mh}) "
              f"in {_short(RANGES_PATH)} — run "
              f"`python3 benches/latency/bench.py smoke --variant {variant} "
              f"--capture-baseline` on this machine first", file=sys.stderr)
        # A fresh clone is not the only way here. machine_hash covers CPU
        # model, thread count, KERNEL VERSION, GOVERNOR and PREEMPT_RT, so a
        # kernel upgrade or a governor that reverted to powersave on reboot
        # flips the identity and orphans a perfectly good baseline. Saying
        # only "no baseline" would send the operator to re-capture against
        # the drifted posture — anchoring the gate to the thing that changed.
        #
        # But say it ONLY when drift is actually possible. `host_entry` is
        # this machine's own key: when it is present, identity drift is
        # excluded by construction and the problem is in the YAML — an entry
        # carrying no ranges for this variant, a half-written capture, a
        # hand-edited file. Gating the drift text on `others` alone told
        # those operators their identity had changed when it demonstrably
        # had not, and sent them to check a governor that was not the fault.
        others = [h for h in (data.get("hosts") or {}) if h != mh]
        # The posture is printed on EVERY no-baseline exit, not just the drift
        # one. It was briefly gated inside the drift branch and that was
        # backwards: the shapes it was withheld from are exactly the ones
        # whose remedy is "re-capture this variant", and a capture taken on a
        # powersave governor bakes the wrong posture into the baseline every
        # later run is gated against. The drift shape is the one told NOT to
        # re-capture yet. `cmd_smoke` builds no RunManifest and
        # --capture-baseline prints no posture of its own, so this line is the
        # only posture signal on the whole path.
        #
        # ONE read, not two: print_governor_state_loudly returns the pair it
        # printed, so the sentence below interpolates the same values the
        # `[env]` line shows. Reading twice let a governor that changed
        # between the calls disagree with itself in the same paragraph.
        governor, boost = print_governor_state_loudly()
        if host_entry is not None:
            # This machine's own key is present, so identity drift is
            # excluded by construction and the fault is in the YAML. Note that
            # `others` is deliberately NOT consulted: on a shared bench fleet
            # this host's entry and other hosts' entries coexist, and an
            # `and not others` here sends exactly those operators down the
            # drift path. Pinned by the "own entry AND other hosts" arm.
            # Do not call this a corrupt file. Baselines are keyed PER
            # PACING VARIANT, so the ordinary cause is a complete, valid
            # capture taken for a different one — the fixture's own
            # "own entry, other variant only" arm is exactly that shape.
            # Telling that operator their YAML is "partial or hand-edited"
            # sends them to inspect a file that is perfectly well-formed.
            have = sorted(((host_entry or {}).get("rtt_p50_ns") or {}))
            print(f"       this host's entry IS on file but carries no "
                  f"{variant} ranges"
                  + (f" — it has {', '.join(have)}, and baselines are keyed "
                     f"per pacing variant" if have else
                     " — its rtt_p50_ns map is empty, so this is a partial "
                     "or hand-edited capture")
                  + f". Not identity drift: the machine_hash matched. "
                  f"Re-capture this variant.", file=sys.stderr)
        elif others:
            # The governor value is INTERPOLATED rather than left to the
            # `[env]` line alone. That line goes to STDOUT while every line
            # here is stderr, so "check the posture below" was true only on a
            # line-buffered TTY and inverted or vanished under `tee`, CI
            # capture, or a stderr-only log.
            print(f"       {len(others)} other host(s) ARE on file "
                  f"({', '.join(sorted(others)[:3])}"
                  f"{', …' if len(others) > 3 else ''}) but this one is not. "
                  f"If this machine captured a baseline before, its identity "
                  f"changed — machine_hash covers kernel version, cpu "
                  f"governor and PREEMPT_RT, so a kernel upgrade or a "
                  f"governor reverting to powersave will orphan it. This "
                  f"host now reads governor={governor}, turbo/boost={boost}; "
                  f"prefer confirming that posture over re-capturing against "
                  f"a drifted one.", file=sys.stderr)
        else:
            # No entry for this host and no other hosts either — the shipped
            # `hosts: {}`, i.e. what a fresh clone hits. This arm exists
            # because the docs and the wrapper both tell the operator the
            # gate names which shape it hit, and without it the SHIPPED shape
            # was the one that got no second line at all: the claim was false
            # for the default case and true only for the rarer ones.
            print(f"       the file carries no host entries at all — nothing "
                  f"has been captured on any machine yet, so this is a fresh "
                  f"clone rather than a drifted identity or a partial "
                  f"capture.", file=sys.stderr)
        # ...and STILL the skippable code. A previous pass returned 3 for the
        # `others` case, reasoning that it is ambiguous — a new machine, or
        # this one after drift — and that ambiguity should resolve toward the
        # gate. The asymmetry argument was right; the premise was not. This
        # file's own header says "Multiple hosts coexist under `hosts:` — the
        # smoke gate looks up its own machine_hash", so a populated map is
        # the DESIGNED steady state, not a signal. Returning 3 therefore made
        # the documented default command (`scripts/run_benchmarks.sh`, which
        # skips only on 4) fail on every machine except the one that
        # captured — the moment the file is used as intended.
        #
        # Drift and a new machine are indistinguishable from here, so the
        # exit code cannot carry that distinction and must not pretend to.
        # What replaces the block is ONE signal, stated at its real strength
        # rather than the two I first claimed: the lines above, on stderr,
        # plus print_governor_state_loudly. cmd_smoke builds no RunManifest
        # (only the native/workspace/ros2 sweeps do), so nothing here reaches
        # a run.json, and in the wrapper flow this is about (exit 4 -> skip
        # the gate -> run the latency test) no later bench.py invocation
        # writes one either. A printed warning, not a recorded one — which is
        # why check_smoke_exit_contract pins it.
        return SMOKE_RC_NO_BASELINE

    # ---- from here on a cell WILL run: the data-producing preflights ----
    # Smoke PRODUCES measurements (and, with --capture-baseline, a
    # checked-in artifact), so it owes the same loud posture banner as the
    # four sweep subcommands. It calls the leg helpers directly rather than
    # through cmd_native/cmd_workspace/cmd_ros2, so it never inherited one:
    # a stock smoke run gated silently against ranges captured under an
    # unknown posture.
    set_dma_basis("host")
    announce_posture("smoke cells")
    require_dma_posture_deliverable("smoke cells")

    if ambient_payload_restriction() is not None:
        print("smoke: CER_BENCH_PAYLOAD_SIZES is exported — the smoke gate "
              "runs a CURATED payload set and a restricted sweep would gate "
              "less than it claims. Unset it first.", file=sys.stderr)
        return 3

    # The smoke subset includes the zenoh-SHM cell, which WEDGES under the
    # default 8 MiB memlock (see ensure_memlock_for_zenoh) — raise or fail
    # fast BEFORE building anything.
    if not ensure_memlock_for_zenoh():
        print("smoke: zenoh-SHM smoke cell would wedge under this memlock "
              "limit — fix the limit rather than skipping (the smoke subset "
              "is curated; a silently thinner subset would gate less than "
              "it claims).", file=sys.stderr)
        return 3

    # Partition cells into runnable vs skipped (with reasons) BEFORE running.
    docker_ok = docker_available()
    chrt_ok = find_chrt_prefix() is not None
    runnable: List[SmokeCell] = []
    skips: List[tuple] = []  # (cell_name, payload, reason)
    for cell in enumerate_smoke_cells(variant):
        reason = cell.gate_reason
        if reason is None and cell.kind == "ros2" and not docker_ok:
            reason = "docker unavailable — ROS 2 cells skipped"
        if reason is None and cell.chrt == 1 and not chrt_ok:
            reason = (f"chrt -f 80 unavailable on {PLATFORM_LABEL}" if IS_LINUX
                      else f"chrt is Linux-only ({PLATFORM_LABEL} host)")
        if reason is not None:
            skips.extend((cell.cell_name, p, reason) for p in cell.payloads)
        else:
            runnable.append(cell)

    # Capture mode runs the whole subset SMOKE_BASELINE_REPS times (each rep
    # in its own rep<k>/raw so reps never overwrite — A4/A1); the gate path
    # stays single-rep.
    smoke_root = RESULTS_ROOT / "smoke"
    if smoke_root.exists():
        shutil.rmtree(smoke_root)
    n_reps = SMOKE_BASELINE_REPS if args.capture_baseline else 1
    raw_dirs = ([smoke_root / f"rep{k}" / "raw" for k in range(1, n_reps + 1)]
                if args.capture_baseline else [smoke_root / "raw"])

    try:
        # per_rep[k] = [(cell_name, payload, p50_ns)] in a FIXED cell order.
        per_rep: List[List[tuple]] = []
        for rep_i, smoke_raw in enumerate(raw_dirs, 1):
            smoke_raw.mkdir(parents=True)
            if n_reps > 1:
                print(f"\n[smoke] baseline rep {rep_i}/{n_reps}")
            _run_smoke_cells(variant, runnable, smoke_raw)
            rep_measured: List[tuple] = []
            for cell in runnable:
                for payload in cell.payloads:
                    bin_path = smoke_raw / f"{cell.cell_name}_{payload}.bin"
                    if not bin_path.exists():
                        raise SmokeSetupError(
                            f"{cell.cell_name} produced no samples for payload "
                            f"{payload} ({bin_path} missing)")
                    rep_measured.append((cell.cell_name, payload,
                                         read_p50_ns(bin_path)))
            per_rep.append(rep_measured)
        measured: List[tuple] = per_rep[0]  # gate path: the single rep
    except SmokeSetupError as e:
        print(f"smoke: {e}", file=sys.stderr)
        return 3

    print()
    if args.capture_baseline:
        # Merge measured ranges into this host's entry (per pacing, cell,
        # payload) so a partial capture — e.g. docker down, ROS 2 cells
        # skipped — never silently drops previously captured ranges. Only
        # THIS host's entry is touched; every other host survives verbatim
        # through the parse → emit round-trip. Bounds are anchored on the
        # MEDIAN of the per-rep p50s (A4): a single lucky/unlucky rep can no
        # longer set the gate's anchor.
        all_pacing = dict((host_entry or {}).get("rtt_p50_ns", {}))
        ranges = dict(all_pacing.get(variant, {}))
        n_preserved = sum(len(v) for v in ranges.values())
        n_captured = 0
        for idx, (cell_name, payload, _p50) in enumerate(per_rep[0]):
            rep_p50s = [rep[idx][2] for rep in per_rep]
            med = median_int(rep_p50s)
            lo, hi = max(med // 2, 1), med * 2
            cell_ranges = dict(ranges.get(cell_name, {}))
            if str(payload) in cell_ranges:
                n_preserved -= 1
            cell_ranges[str(payload)] = [lo, hi]
            ranges[cell_name] = cell_ranges
            n_captured += 1
            print(f"[ ok ] {cell_name}  {payload}  rep p50s="
                  f"{'/'.join(str(p) for p in rep_p50s)}ns  median={med}ns "
                  f"(captured range {lo}-{hi})")
        for cell_name, payload, reason in skips:
            print(f"[skip] {cell_name}  {payload}  ({reason})")
        all_pacing[variant] = ranges
        data["hosts"][mh] = {
            "machine_hash": mh,
            "measured_on_git_sha": _git_sha(),
            # The posture belongs in the note because the SCHEMA has no
            # posture axis: entries are keyed (machine_hash, variant, cell,
            # payload), so a stock capture and a tuned one overwrite each
            # other and the file looks posture-neutral either way. A stock
            # capture is a legitimate baseline; an UNLABELLED one silently
            # gates tuned runs against stock tails, which is the suite's own
            # "never mix postures without saying so" rule (METHODOLOGY §11)
            # reached through a checked-in artifact that outlives the run.
            "notes": f"captured by bench.py smoke --capture-baseline "
                     f"({variant}, posture={resolved_dma_posture()}); ranges "
                     f"are +/-2x bounds around the MEDIAN of {n_reps} "
                     f"smoke-rep p50s (A4); partial captures merge per cell "
                     f"and the git sha reflects the latest capture",
            "rtt_p50_ns": all_pacing,
        }
        RANGES_PATH.write_text(emit_expected_ranges(data))
        if skips:
            # A partial capture is allowed (it merges, never drops), but it
            # is NOT a complete baseline: the gate path refuses to run
            # against a range-less cell, so say so here rather than letting
            # the next `smoke` invocation look like the problem.
            print(f"\nsmoke: WARNING — {len(skips)} cell/payload(s) were "
                  f"skipped, so this baseline does NOT cover the whole "
                  f"curated subset. A later gate run REFUSES (exit 3) any "
                  f"cell it measures without a range; re-capture with those "
                  f"cells runnable.", file=sys.stderr)
        print(f"\nsmoke: captured {n_captured} baseline ranges for "
              f"machine_hash {mh} ({variant}; median of {n_reps} reps) "
              f"({n_preserved} pre-existing ranges preserved, {len(skips)} "
              f"cells skipped) -> {_short(RANGES_PATH)} — exit 0")
        return 0

    n_ok = n_high = n_low = 0
    # A cell that RAN but has no baseline range is a hole in the gate, not
    # a skip: `--capture-baseline` writes whatever it could measure, so a
    # capture taken with (say) docker down leaves the curated ROS 2 cells
    # range-less, and every later run would report green off the native
    # cells alone while gating nothing there. Skips are reserved for
    # structurally unavailable cells (they never reach `measured`).
    ungated: List[tuple] = []
    for cell_name, payload, p50 in measured:
        rng = host_ranges.get(cell_name, {}).get(str(payload))
        if rng is None:
            ungated.append((cell_name, payload))
            print(f"[MISS] {cell_name}  {payload}  p50={p50}ns — no range "
                  f"in this host's baseline")
            continue
        lo, hi = rng
        if p50 > hi:
            tag = "FAIL_HIGH"; n_high += 1
        elif p50 < lo:
            tag = "FAIL_LOW"; n_low += 1
        else:
            tag = "[ ok ]"; n_ok += 1
        print(f"{tag} {cell_name}  {payload}  p50={p50}ns (range {lo}-{hi})")
    for cell_name, payload, reason in skips:
        print(f"[skip] {cell_name}  {payload}  ({reason})")

    if ungated:
        names = ", ".join(f"{c}@{p}" for c, p in ungated)
        print(f"\nsmoke: {len(ungated)} measured cell/payload(s) have no "
              f"baseline range for this host ({names}) — the gate would "
              f"report a result it did not gate. Re-run `bench.py smoke "
              f"--variant {variant} --capture-baseline` on this machine "
              f"with every curated cell runnable (docker up, chrt "
              f"available) — exit 3", file=sys.stderr)
        return 3

    n_measured = n_ok + n_high + n_low
    if n_measured == 0 and skips:
        print(f"smoke: all {len(skips)} measured cells were skipped — no "
              f"baseline entries matched this host (machine_hash {mh}, "
              f"{variant}). Re-run `bench.py smoke --variant {variant} "
              f"--capture-baseline` on this machine.", file=sys.stderr)
        return 3
    exit_code = 2 if (n_high or n_low) else 0
    print(f"\nsmoke: {n_ok} ok, {n_high} fail_high, {n_low} fail_low, "
          f"{len(skips)} skipped — exit {exit_code}")
    return exit_code

# ============================================================================
# Entry point
# ============================================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="bench.py",
        description="Cerulion public latency-benchmark orchestrator "
                    "(benches/latency/)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    VARIANT_HELP = ("Pacing variant, mapped to CER_BENCH_PACING for every "
                    "spawned bench process: quiescent (sensor-rate "
                    "realism, default), fixed100 (ONE uniform 100 Hz "
                    "target at every payload size, with the "
                    "100→50→20→sensor-floor fallback ladder + achieved-"
                    "rate sidecars — METHODOLOGY § 'The rate axis'), or "
                    "backtoback (saturation, transport upper-bound)")
    RUN_DIR_HELP = ("Override the run directory (default: "
                    "results/<machine-hash>-<date>-<variant>/)")
    REP_HELP = ("Rep index k (>= 1, default 1): raw samples land in rep<k>/ "
                "under the run dir so repeated sweeps ACCUMULATE instead of "
                "overwriting; compile-csv aggregates across reps")

    def add_common(sp, chrt_choices=("0", "1", "both")):
        sp.add_argument("--variant", choices=PACING_VARIANTS,
                        default="quiescent", help=VARIANT_HELP)
        sp.add_argument("--chrt", choices=chrt_choices, default="0",
                        help="Run chrt-off (0), chrt-on (1), or both (default: 0)")
        sp.add_argument("--run-dir", help=RUN_DIR_HELP)

    p_native = sub.add_parser("native", help="Build + run native bench binaries")
    add_common(p_native)
    p_native.add_argument("--rep", type=int, default=1, help=REP_HELP)
    p_native.add_argument("--bin", help="Run a single binary by name "
                          "(e.g. raw_iceoryx2_round_trip) or raw prefix (e.g. iox2)")
    p_native.set_defaults(func=cmd_native)

    p_ws = sub.add_parser("workspace",
                          help="Run the `cerulion graph run` workspace legs")
    add_common(p_ws)
    p_ws.add_argument("--rep", type=int, default=1, help=REP_HELP)
    p_ws.add_argument("--leg", choices=WORKSPACE_LEGS + ("both",), default="both",
                      help="Workspace leg: split (graph run rtt_bench_split — "
                           "the declared 2-group process_groups multi-process "
                           "headline row), mono (--single-process), or both")
    p_ws.add_argument("--msg-class", dest="msg_class",
                      choices=WORKSPACE_MSG_CLASSES + ("both",),
                      default="both",
                      help="Type-class axis (METHODOLOGY § 'The type-class "
                           "axis'): variable (the incumbent sensor_msgs/Image "
                           "legs — unbounded data, loaned per tick; pinned "
                           "token-less prefixes), pod (the fixed-PodPayload "
                           "twin chain — `_pod` prefixes), or both (default)")
    p_ws.set_defaults(func=cmd_workspace, sizes=None)

    p_ros2 = sub.add_parser("ros2", help="Build + run dockerized ROS 2 cells")
    add_common(p_ros2)
    p_ros2.add_argument("--rep", type=int, default=1, help=REP_HELP)
    p_ros2.add_argument("--distro", choices=ROS2_DISTROS,
                        help="Restrict to one distro (default: all three)")
    p_ros2.add_argument("--cells", help="Substring filter on cell name")
    p_ros2.add_argument("--build-image", action="store_true",
                        help="Force `docker build` even if the image exists")
    p_ros2.set_defaults(func=cmd_ros2, sizes=None)

    p_csv = sub.add_parser("compile-csv",
                           help="Convert raw .bin samples to per-cell CSVs "
                                "(aggregating across rep<k>/ subdirs)")
    p_csv.add_argument("--variant", choices=PACING_VARIANTS,
                       default=None,
                       help=f"Which variant's default run dir to compile "
                            f"(the run dir is variant-keyed; pass the value "
                            f"the sweep ran with, or --run-dir explicitly). "
                            f"Default: {PACING_VARIANTS[0]} for the run-dir "
                            f"NAME, and ignored entirely with --run-dir. "
                            f"With --raw-dir the variant comes from the raw "
                            f"dir's parent's run.json manifest, and an "
                            f"explicit value contradicting it is refused.")
    p_csv.add_argument("--run-dir", help=RUN_DIR_HELP)
    p_csv.add_argument("--raw-dir", help="Override: compile ONE raw .bin dir "
                       "(legacy single-rep interface; default is run-dir "
                       "rep discovery)")
    p_csv.add_argument("--out-dir", help="Override CSV output dir "
                       "(default: <run-dir>, or the raw dir's PARENT under "
                       "--raw-dir — that directory IS the run dir for the "
                       "legacy single-rep interface, and is where "
                       "compile_csv.py reads the manifest variant from)")
    p_csv.add_argument("--allow-partial", action="store_true",
                       help="Demote missing/0-sample/wrong-count payload "
                            "rows from a hard error to loud stderr warnings "
                            "(deliberately partial sweeps)")
    p_csv.set_defaults(func=cmd_compile_csv)

    p_plots = sub.add_parser("plots", help="Regenerate plot PNGs from CSVs")
    p_plots.add_argument("--variant", choices=PACING_VARIANTS,
                         default=None,
                         help=f"Which variant's default run dir to plot "
                              f"(the run dir is variant-keyed; pass the "
                              f"value the sweep ran with, or --run-dir). "
                              f"Default: {PACING_VARIANTS[0]} for the run-dir "
                              f"NAME; with --run-dir the variant comes from "
                              f"that dir's run.json manifest, and an "
                              f"explicit value contradicting it is refused.")
    p_plots.add_argument("--run-dir", help=RUN_DIR_HELP)
    p_plots.add_argument("--group", action="append",
                         help="Plot group (repeatable; default: all). "
                              "See plot.py --help for the group list.")
    p_plots.add_argument("--chrt", choices=("0", "1", "both"), default="0",
                         help="Which chrt lines the plots expect (default: 0)")
    p_plots.add_argument("--skip-missing", action="store_true",
                         help="Warn (loudly) instead of failing when an "
                              "expected CSV is missing")
    p_plots.set_defaults(func=cmd_plots)

    p_smoke = sub.add_parser(
        "smoke",
        help="Low-n smoke gate: compare curated-cell p50s against "
             "expected-ranges.yaml",
        description=f"Runs a curated cell subset at low sample counts "
                    f"({SMOKE_MEASURED} measured samples per cell/payload) and "
                    f"compares each measured p50 against this host's "
                    f"[min, max] entry (per pacing variant) in "
                    f"expected-ranges.yaml. Exit codes: 0 = all in range; "
                    f"2 = any FAIL_HIGH/FAIL_LOW; "
                    f"3 = crash/build/setup failure, or a cell that ran "
                    f"with no range in it; {SMOKE_RC_NO_BASELINE} = no "
                    f"baseline for this host+variant (nothing to gate "
                    f"against — the file absent, the shipped `hosts: {{}}`, "
                    f"a shared file holding only other hosts (its designed "
                    f"steady state), or this host's own entry with no "
                    f"ranges for this variant). One code, four shapes, "
                    f"different remedies — the gate names which.")
    p_smoke.add_argument("--variant", choices=PACING_VARIANTS,
                         default="quiescent", help=VARIANT_HELP)
    p_smoke.add_argument("--capture-baseline", action="store_true",
                         help=f"Run the subset {SMOKE_BASELINE_REPS}x and "
                              f"write/update only THIS host's entry in "
                              f"expected-ranges.yaml (range = ±2x bounds "
                              f"around the MEDIAN of the "
                              f"{SMOKE_BASELINE_REPS} rep p50s); "
                              f"other hosts' entries are preserved")
    p_smoke.set_defaults(func=cmd_smoke)

    p_full = sub.add_parser("full",
                            help="native + workspace + ros2 (x --reps, "
                                 "round-robin) + compile-csv + plots")
    add_common(p_full)
    p_full.add_argument("--reps", type=int, default=1,
                        help="Run the WHOLE matrix N times round-robin "
                             "(rep 1 end-to-end, then rep 2, ...) — the §10 "
                             "same-window interleaving mechanism at rep "
                             "scale (campaign rule: k >= 5 for "
                             "close-comparison claims). Default 1.")
    p_full.add_argument("--distro", choices=ROS2_DISTROS,
                        help="Restrict ROS 2 cells to one distro")
    p_full.add_argument("--build-image", action="store_true")
    p_full.set_defaults(func=cmd_full)

    p_list = sub.add_parser("list-cells",
                            help="Print the full cell inventory (incl. "
                                 "'! skip' lines) without running anything")
    p_list.add_argument("--variant", choices=PACING_VARIANTS,
                        default="quiescent", help=VARIANT_HELP)
    p_list.add_argument("--chrt", choices=("0", "1", "both"), default="both")
    p_list.add_argument("--distro", choices=ROS2_DISTROS,
                        help="Restrict to one distro (default: all three)")
    p_list.set_defaults(func=cmd_list_cells)

    return p

def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    # Park the parsed args so the posture refusal can report what an
    # explicitly-targeted --run-dir already holds (see
    # _refusal_leftovers_note). Read-only, via getattr with defaults.
    global _REFUSAL_ARGS, _DMA_BASIS, _DMA_PROBE
    _REFUSAL_ARGS = args
    # An empty CARGO_TARGET_DIR is refused AHEAD of dispatch for every
    # subcommand that builds or runs. The workspace-driving four already
    # reach it through `refuse_foreign_cerulion_override`'s preflight, but
    # `native` and `ros2` BUILD without resolving a CLI, so under an empty
    # value they used to reach cargo and die with "cargo build failed in
    # <dir>" — which names neither the variable nor the remedy and sends
    # the operator hunting a compile error that does not exist. The
    # post-processing verbs touch no cargo-built artifact and are listed
    # out rather than in, so a NEW build-driving subcommand is covered the
    # day it lands instead of being forgotten.
    try:
        # INSIDE the try, so the refusal it raises still runs the reset
        # below. Above it, the SystemExit left `_REFUSAL_ARGS` holding the
        # FAILED invocation's args and `_DMA_BASIS`/`_DMA_PROBE` holding a
        # PREVIOUS in-process drive's posture — and the next imported call
        # would then name the wrong run directory in its refusal, which is
        # the exact confusion the reset exists to prevent (the basis is a
        # one-way ratchet, so a leftover "host" also locks out a later
        # sweep's own "container" declaration).
        if args.func not in (cmd_compile_csv, cmd_plots, cmd_list_cells):
            cargo_target_dir_setting()
        return args.func(args)
    finally:
        # Reset, so a SECOND in-process drive cannot read the previous
        # invocation's --run-dir and name the wrong directory in a refusal
        # — exactly the confusion the note exists to prevent. The posture
        # slots go with it: the basis is now a one-way ratchet, so a
        # leftover "host" from a previous drive would silently lock out a
        # standalone ros2 sweep's own "container" declaration.
        _REFUSAL_ARGS = argparse.Namespace()
        _DMA_BASIS = None
        _DMA_PROBE = None

if __name__ == "__main__":
    sys.exit(main())
