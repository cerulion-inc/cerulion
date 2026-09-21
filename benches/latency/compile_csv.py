#!/usr/bin/env python3
"""compile_csv.py — turn raw .bin sample dumps into per-cell summary CSVs.

Reads raw `.bin` files (each a u64-LE nanosecond round-trip sample stream
from one cell at one payload size) and writes ONE summary CSV per cell into
`<out-dir>`.

Input layouts (the rep dimension):

    --run-dir <run>     discovers rep<k>/raw/ subdirs (bench.py --rep k /
                        `full --reps N`) and AGGREGATES across reps; a
                        legacy rep-less dir (raw/ directly under the run
                        dir) is read as a single rep. The pacing variant is
                        read from <run>/run.json (the bench.py manifest)
                        for the exact-sample-count gate.
    --raw-dir <dir>     legacy explicit single raw dir (single rep).

Filename convention (CER_BENCH_RAW_NAME + payload, set by bench.py):

    <raw_prefix>_<payload-bytes>.bin

where <raw_prefix> always ends in `_chrt<0|1>` and is one of the pinned
line-inventory names, e.g.:

    iox2_chrt0                                         (native floor)
    zenoh_shm_chrt1                                    (native comparison)
    cerulion_workspace_split_chrt0                     (workspace headline leg)
    cerulion_workspace_mono_chrt0                      (workspace leg)
    cerulion_workspace_split_pod_chrt0                 (type-class pod twin)
    jazzy_cyclonedds_shm_rclcpp_be1_chrt0              (ROS 2 cell)
    jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0        (type-class image cell)
    jazzy_fastdds_zc_loan_be1_chrt0                    (FastDDS DataSharing lane)

Output CSV path — the raw_prefix → CSV coupling is deliberately trivial
(the old tree's special-cased names silently dropped lines from plots when
a prefix drifted):

    <out-dir>/results_<raw_prefix>.csv

CSV schema — leading `#` comment lines document the derived columns (plot.py
and any csv reader must skip them); the first 21 columns keep the historical
layout; the rep columns and the fixed100 achieved-rate column are appended
on the right:

    payload_bytes, iterations, round_trip_p50_ns, round_trip_p99_ns,
    round_trip_mean_ns, one_way_p50_ns, chrt, loaned,
    floor_ns, p1_ns, p10_ns, p25_ns, p75_ns, p90_ns, p95_ns, p99_9_ns, max_ns,
    rep_count, rep_p50_min_ns, rep_p50_max_ns, achieved_rate_hz,
    unstamped, nonpositive_rtt

unstamped / nonpositive_rtt (ROS 2 cells only; empty elsewhere): the echoes
that COMPLETED the round trip and yielded no sample, one column per reason,
read from the runner's per-cell delivery receipt
(`<raw>/_logs/<cell>_<size>_delivery.txt`, `DELIVERY role=latency`) and
POOLED over exactly the reps `iterations` is pooled over — the ones whose
samples reached this row, not every rep discovered on disk. `unstamped` = the publisher
sent no stamp (a wiring fault); `nonpositive_rtt` = the receive instant was
not after the send instant (a duplicate or non-monotone stamp), whose
unsigned subtraction would otherwise record ~1.8e19 ns as a latency. Both
were counted at the sink and printed on its receipt but stopped there, so a
cell with one sporadic drop published a row whose `max_ns` could carry the
first log-once WARN's cost with nothing on the row saying a drop occurred.
EMPTY means "no receipt was OWED here" — a native or workspace leg, which
writes none; `unknown` means one WAS owed and could not be totalled: a rep
carried no receipt although the ROS 2 driver ran it, or carried one with no
parseable count, or only some reps answered (a sum over those is an
under-count wearing a total's clothes). Absence alone is never blank on a
cell the ROS 2 driver ran: a cell that lost its accounting must not publish
the value that means the question does not apply to it. A did_not_sustain row leaves both empty: the receipt that exists
there describes the fallback ladder's last rung, not a measured run, and
every other cell on that row is blank for the same reason.

These two columns are APPENDED, so the historical column order is
unchanged and every reader that resolves columns by NAME (plot.py's
csv.DictReader) is unaffected by their arrival.

achieved_rate_hz (fixed100 pacing variant only): the publish rate the row
ACTUALLY ran at, read from the
runner-written `<cell>_<payload>.rate` sidecar beside the `.bin` (first
line: integer Hz). Empty on quiescent/backtoback rows (no sidecar). A
value below the 100 Hz target means the cell stepped down the fallback
ladder (100→50→20→sensor floor) — plot.py annotates those points, so a
mixed-rate line is never silent. `mixed` = the reps disagree (loud warn).
A cell that exhausted the ladder at a payload carries a `did_not_sustain`
sidecar and NO `.bin`: the row is written EMPTY (payload + chrt +
achieved_rate_hz=did_not_sustain, every stat cell blank, iterations 0)
with a loud stderr note — a recorded no-latency outcome, never a fake
number (Principle #13) and never a silent hole.

Cross-rep semantics:

  - round_trip_p50_ns (the HEADLINE) is the MEDIAN of the per-rep p50s —
    identical to the plain p50 when rep_count == 1. rep_p50_min_ns /
    rep_p50_max_ns are the per-rep p50 spread (the error band plot.py
    shades when rep_count > 1; a two-line comparison claim must exceed
    BOTH lines' spread).
  - every OTHER statistic pools the samples of all reps; `iterations` is
    the pooled sample count (per-rep count = iterations / rep_count).
  - one_way_p50_ns = round_trip_p50_ns // 2 — RTT p50 / 2, a DERIVED
    approximation valid only for path-symmetric lines; it is not a
    measurement.

PERCENTILE SUPPRESSION: any percentile cell backed by fewer than
MIN_TAIL_EXCEEDANCES (20) pooled samples in its estimating tail
(n * min(q, 1-q)) is written EMPTY — at the pinned tail-resolved schedule
(measured n >= 2000 at every size) p50/p95/p99/p1 are citable
at every size single-rep, and the suppression bites on p99.9 from 64 KB up
(reps raise the pooled n and un-suppress by the same rule). Suppressions are
counted on stderr, loudly.

STRICT BY DEFAULT: every discovered cell prefix must carry the full pinned
payload sweep (bench.PAYLOAD_SIZES — imported from the sibling bench.py,
the single source of truth) in EVERY discovered rep, and every
present .bin must contain EXACTLY the schedule's measured sample count for
the run's pacing variant (bench.samples_for) — a truncated .bin from a
killed run must never summarize into a full row, and `>= 1 sample` is not
strict. When the variant is unrecoverable (legacy dir, no run.json) the
count check degrades to a LOUD warning when n matches NEITHER schedule.
A bad (prefix, rep, payload) row exits nonzero listing exactly what is
wrong (CSVs for the good rows are still written, so re-running only the
failed cells heals the artifact). `--allow-partial` demotes the bad rows
to loud stderr warnings for deliberately partial sweeps. Cells that were
structurally skipped upstream produce NO .bins at all — no discovered
prefix, so they never trip the strictness.

Percentiles are computed OFFLINE from the raw dumps here — bench.py's smoke
gate reuses the identical linear-interpolated quantile (parity pinned by
check_percentile_parity.py, which also pins the cross-rep headline rule).

Usage:

    compile_csv.py --run-dir <run> [--out-dir <run>] [--quiet]
                   [--allow-partial]
    compile_csv.py --raw-dir <run>/raw [...]          # legacy single rep
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import struct
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# bench.py lives in the same directory and owns the pinned payload sweep
# (PAYLOAD_SIZES) and the per-variant schedules (samples_for) — the strict
# row-completeness + exact-count checks derive their expectations from
# there, never from a second copy.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench  # noqa: E402


# ---------------------------------------------------------------- parse

# The cell name is everything up to the LAST `_<digits>.bin`; it must end in
# `_chrt<0|1>` (the pinned naming rule). Anything else is loudly skipped.
CELL_RE = re.compile(r"^(?P<cell>.+_chrt(?P<chrt>[01]))_(?P<size>\d+)\.bin$")

# fixed100 achieved-rate sidecars (same stem as the .bin): first line is
# the achieved integer Hz, or `did_not_sustain` (then no .bin exists —
# the fallback ladder was exhausted and no latency was minted).
RATE_RE = re.compile(r"^(?P<cell>.+_chrt(?P<chrt>[01]))_(?P<size>\d+)\.rate$")

DID_NOT_SUSTAIN = "did_not_sustain"

# Per-cell delivery receipts live one directory down from the .bins —
# ros2/run_bench.sh greps the node log into
# `<raw>/_logs/<cell>_<size>_delivery.txt`.
DELIVERY_LOG_SUBDIR = "_logs"

# The stamp-gate counters carried onto the row, in COLUMN ORDER. One
# table, so the reader, the aggregator and the header cannot disagree
# about which counters exist or what they are called.
DELIVERY_DROP_KEYS = ("unstamped", "nonpositive_rtt")

# A receipt was expected and could not be TOTALLED. Distinct from empty
# (no receipt exists at all, which is every native and workspace leg): a
# sum over the reps that happened to answer is an under-count presented
# as a total, and an under-count of DROPS is the one direction that
# flatters the row.
DROPS_UNKNOWN = "unknown"

# rep<k> subdirs under a run dir. Legacy rep-less run dirs keep
# raw/ directly under the run dir.
REP_DIR_RE = re.compile(r"^rep(\d+)$")


@dataclasses.dataclass(frozen=True)
class CellKey:
    cell: str        # full raw_prefix including the _chrtN suffix
    chrt: int        # 0 or 1 (parsed out of the suffix)


def parse_filename(path: Path) -> Optional[Tuple[CellKey, int]]:
    m = CELL_RE.match(path.name)
    if not m:
        return None
    return (
        CellKey(cell=m.group("cell"), chrt=int(m.group("chrt"))),
        int(m.group("size")),
    )


def parse_rate_filename(path: Path) -> Optional[Tuple[CellKey, int]]:
    m = RATE_RE.match(path.name)
    if not m:
        return None
    return (
        CellKey(cell=m.group("cell"), chrt=int(m.group("chrt"))),
        int(m.group("size")),
    )


def read_rate_sidecar(path: Path) -> Optional[str]:
    """First line of a fixed100 `.rate` sidecar (stripped) — an integer
    achieved Hz or `did_not_sustain`. None when unreadable. Anything else
    is warned about and treated as absent (a malformed sidecar must not
    silently label a row)."""
    try:
        with path.open() as f:
            first = f.readline().strip()
    except OSError:
        return None
    if first == DID_NOT_SUSTAIN or re.fullmatch(r"\d+", first):
        return first
    sys.stderr.write(f"warn: {path.name}: malformed .rate sidecar first "
                     f"line {first!r} (expected an integer Hz or "
                     f"'{DID_NOT_SUSTAIN}') — ignoring it\n")
    return None


def read_samples_ns(path: Path) -> List[int]:
    """Read a u64-LE binary stream of nanosecond samples.

    A length that is not a multiple of 8 is a torn / partial u64 dump (a
    killed writer, a truncated copy) — REFUSED with ValueError, never
    silently truncated: a torn dump whose floor(len / 8) happens to equal
    the schedule count would otherwise pass the strict sample-count gate
    below (bench.py::read_p50_ns refuses the same shape)."""
    raw = path.read_bytes()
    if len(raw) % 8:
        raise ValueError(
            f"{path.name}: {len(raw)} bytes is not a multiple of 8 — "
            f"torn/partial u64 dump")
    n = len(raw) // 8
    if n == 0:
        return []
    return list(struct.unpack(f"<{n}Q", raw))


def discover_rep_raw_dirs(run_dir: Path) -> List[Tuple[str, Path]]:
    """[(rep_label, raw_dir)] for a run dir: rep<k>/raw/ subdirs sorted by
    rep index, or the legacy rep-less <run-dir>/raw/ as a single 'rep'.

    A dir carrying BOTH layouts is refused loudly — silently merging a
    legacy raw/ with rep dirs would aggregate unlabeled data."""
    reps: List[Tuple[int, Path]] = []
    for d in run_dir.iterdir():
        if not d.is_dir():
            continue
        m = REP_DIR_RE.match(d.name)
        if m and (d / "raw").is_dir():
            reps.append((int(m.group(1)), d / "raw"))
    legacy = run_dir / "raw"
    if reps and legacy.is_dir():
        sys.stderr.write(
            f"error: {run_dir} carries BOTH rep<k>/raw/ subdirs AND a "
            f"legacy top-level raw/ — refusing to guess which is current. "
            f"Move the legacy raw/ into rep1/ (or aside) and re-run.\n")
        raise SystemExit(2)
    if reps:
        return [(f"rep{k}", p) for k, p in sorted(reps)]
    if legacy.is_dir():
        return [("raw", legacy)]
    return []


def read_manifest_variant(run_dir: Path) -> Optional[str]:
    """The pacing variant from <run-dir>/run.json (the bench.py manifest);
    None when absent/unreadable (legacy dirs) — the exact-count gate then
    degrades to warn-if-matches-neither-schedule."""
    path = run_dir / "run.json"
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        sys.stderr.write(f"warn: {path} is unreadable ({e}) — the pacing "
                         f"variant is unrecoverable; sample-count checks "
                         f"degrade to warn-if-matches-neither-schedule\n")
        return None
    variant = data.get("variant") if isinstance(data, dict) else None
    if variant in bench.PACING_VARIANTS:
        return variant
    sys.stderr.write(f"warn: {path} carries no valid pacing variant — "
                     f"sample-count checks degrade to "
                     f"warn-if-matches-neither-schedule\n")
    return None


# ---------------------------------------------------------------- stats


def percentile(sorted_samples: List[int], p: float) -> int:
    """Linear-interpolated quantile, p ∈ [0, 1].

    MUST stay byte-equivalent to bench.py::read_p50_ns's core formula — the
    smoke gate and the sweep must report identical p50s for the same bin
    (check_percentile_parity.py pins this)."""
    n = len(sorted_samples)
    if n == 0:
        return 0
    if n == 1:
        return sorted_samples[0]
    rank = p * (n - 1)
    lo = int(rank)
    hi = min(lo + 1, n - 1)
    if lo == hi:
        return sorted_samples[lo]
    frac = rank - lo
    return int(sorted_samples[lo] + frac * (sorted_samples[hi] - sorted_samples[lo]))


def summarize(samples: List[int]) -> Dict[str, int]:
    if not samples:
        return {}
    s = sorted(samples)
    return {
        "iterations": len(s),
        "floor": s[0],
        "p1": percentile(s, 0.01),
        "p10": percentile(s, 0.10),
        "p25": percentile(s, 0.25),
        "p50": percentile(s, 0.50),
        "p75": percentile(s, 0.75),
        "p90": percentile(s, 0.90),
        "p95": percentile(s, 0.95),
        "p99": percentile(s, 0.99),
        "p99_9": percentile(s, 0.999),
        "mean": sum(s) // len(s),
        "max": s[-1],
        "one_way_p50": percentile(s, 0.50) // 2,
    }


def aggregate_reps(rep_samples: List[List[int]]) -> Dict[str, int]:
    """Cross-rep aggregation.

    HEADLINE p50 = the median of the per-rep p50s (between-run state, not
    sample count, is the dominant error term — a pooled p50 would let one
    outlier rep drag the headline). Every other statistic pools the samples
    of all reps (equal per-rep n, so pooling is weight-fair, and the pooled
    n is what the A2 tail-suppression rule gates on). For a single
    rep the two definitions coincide, so legacy single-rep CSVs are
    byte-identical."""
    if not rep_samples or any(not s for s in rep_samples):
        raise ValueError("aggregate_reps requires >= 1 rep, all non-empty")
    rep_p50s = [percentile(sorted(s), 0.50) for s in rep_samples]
    pooled = [x for s in rep_samples for x in s]
    stats = summarize(pooled)
    stats["p50"] = percentile(sorted(rep_p50s), 0.50)
    stats["one_way_p50"] = stats["p50"] // 2
    stats["rep_count"] = len(rep_samples)
    stats["rep_p50_min"] = min(rep_p50s)
    stats["rep_p50_max"] = max(rep_p50s)
    return stats


# ------------------------------------------------------- tail suppression

# Audit A2: a percentile cell whose estimating tail holds fewer than this
# many pooled samples is written EMPTY — at n=2000 (4/16 MB, single rep,
# the tail-resolved schedule) the type-7 p99.9 rests on ~2 tail
# samples and literally interpolates the top order statistics ("the max
# with extra steps"), and the old docs claimed reliability n cannot
# support. 20 exceedances is the citability floor the audit pinned — the
# extended 4/16 MB windows were sized to land p99 exactly AT it.
MIN_TAIL_EXCEEDANCES = 20

# CSV column key -> quantile, for every percentile column the suppression
# rule covers. floor/mean/max are NOT percentile estimates (per-run extreme
# / moment statistics) and are never suppressed.
PERCENTILE_QUANTILES: Dict[str, float] = {
    "p1": 0.01, "p10": 0.10, "p25": 0.25, "p50": 0.50, "p75": 0.75,
    "p90": 0.90, "p95": 0.95, "p99": 0.99, "p99_9": 0.999,
}


def tail_exceedances(n: int, q: float) -> float:
    """Samples in the tail a q-quantile estimate rests on: n*(1-q) above an
    upper-tail percentile, n*q below a lower-tail one (the audit stated the
    rule for upper tails; it is applied symmetrically — a p1 at a given n
    rests on the same tail-sample count a p99 does)."""
    return n * min(q, 1.0 - q)


def percentile_is_citable(n: int, q: float) -> bool:
    return tail_exceedances(n, q) >= MIN_TAIL_EXCEEDANCES


# ---------------------------------------------------------------- output


CSV_COMMENT = (
    "# per-cell summary (compile_csv.py). Lines starting with '#' "
    "are schema notes — skip them when parsing.\n"
    "# one_way_p50_ns = round_trip_p50_ns // 2 — RTT p50 / 2, a DERIVED "
    "approximation valid only for path-symmetric lines; not a measurement.\n"
    "# round_trip_p50_ns = median of per-rep p50s (equals the plain p50 at "
    "rep_count=1); rep_p50_{min,max}_ns = per-rep p50 spread; all other "
    "stats pool samples across reps (iterations = pooled count).\n"
    "# empty percentile cells were SUPPRESSED: fewer than 20 pooled samples "
    "back that tail at this n (see MIN_TAIL_EXCEEDANCES; add reps or extend "
    "the schedule to make them citable).\n"
    "# achieved_rate_hz (fixed100 variant only; empty elsewhere): the "
    "publish rate the row ACTUALLY ran at, from the runner's .rate sidecar "
    "— below the 100 Hz target = fallback-ladder rung (plots annotate it); "
    "'mixed' = reps disagree; 'did_not_sustain' = the ladder was exhausted "
    "and the row is EMPTY (no latency was minted).\n"
    "# unstamped / nonpositive_rtt (ROS 2 cells only): echoes that completed "
    "the round trip and yielded NO sample, pooled across reps like "
    "iterations — the terms that close received == samples + warmup + "
    "unstamped + nonpositive_rtt. Empty = no receipt was OWED here (a "
    "native or workspace leg writes none); 'unknown' = one WAS owed and "
    "could not be totalled (a rep the ROS 2 driver ran carried none, or "
    "carried no parseable count, or only some reps answered), never a sum "
    "over the reps that answered.\n"
)

CSV_HEADER = (
    "payload_bytes,iterations,round_trip_p50_ns,round_trip_p99_ns,"
    "round_trip_mean_ns,one_way_p50_ns,chrt,loaned,"
    "floor_ns,p1_ns,p10_ns,p25_ns,p75_ns,p90_ns,p95_ns,p99_9_ns,max_ns,"
    "rep_count,rep_p50_min_ns,rep_p50_max_ns,achieved_rate_hz,"
    + ",".join(DELIVERY_DROP_KEYS) + "\n"
)


def node_log_path(raw_dir: Path, cell: str, size: int) -> Path:
    """The per-binary log `ros2/run_bench.sh` writes for one cell.

    ONLY the ROS 2 driver writes it (`BIN_LOG=...` in run_bench.sh); the
    native binaries and `workspace/run_workspace.sh` write none. So its
    presence is per-cell evidence that the driver which produces delivery
    receipts ran here — which is what lets an ABSENT receipt be told apart
    from a leg that never writes one."""
    return raw_dir / f"{cell}_{size}_node.log"


def loaned_from_log(raw_dir: Path, cell: str, size: int) -> str:
    """Backfill the `loaned` column from the per-binary log written by the
    ROS 2 driver (<cell>_<size>_node.log). Empty string if missing."""
    log_path = node_log_path(raw_dir, cell, size)
    if not log_path.exists():
        return ""
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        return ""
    m = re.search(r"loaned=([01])", text)
    return m.group(1) if m else ""


def loaned_across_reps(raw_dirs: List[Tuple[str, Path]], cell: str,
                       size: int) -> str:
    """The `loaned` column across reps: all reps must agree. Disagreement
    (loan engagement flapping between reps) is a real signal — warned
    loudly and written as 'mixed', never silently collapsed."""
    values = {}
    for rep_label, raw_dir in raw_dirs:
        v = loaned_from_log(raw_dir, cell, size)
        if v:
            values[rep_label] = v
    distinct = set(values.values())
    if not distinct:
        return ""
    if len(distinct) == 1:
        return distinct.pop()
    sys.stderr.write(
        f"warn: {cell} payload {size}: loaned= disagrees across reps "
        f"({', '.join(f'{k}={v}' for k, v in sorted(values.items()))}) — "
        f"loan engagement is flapping; writing 'mixed'\n")
    return "mixed"


def delivery_log_path(raw_dir: Path, cell: str, size: int) -> Path:
    """The per-cell delivery receipt ros2/run_bench.sh greps out of the
    node log. One definition, so the presence test and the read cannot
    disagree about where it lives."""
    return raw_dir / DELIVERY_LOG_SUBDIR / f"{cell}_{size}_delivery.txt"


def _receipt_was_expected(raw_dir: Path, cell: str, size: int) -> bool:
    """Did the driver that WRITES delivery receipts run for this cell?

    Evidence, not a name: only `ros2/run_bench.sh` writes the per-cell
    node log, and it writes one for every cell it runs. Keying on the raw
    prefix instead would be the same name-matching this change removes
    elsewhere, and it would have to be kept in step with the cell
    inventory by hand.

    Anything other than "the file is definitely not there" counts as
    EXPECTED: an unreadable directory is not evidence that no receipt was
    owed, and the conservative answer here is the loud one."""
    try:
        node_log_path(raw_dir, cell, size).stat()
    except FileNotFoundError:
        return False
    except OSError:
        return True
    return True


def delivery_drops_from_log(
        raw_dir: Path, cell: str,
        size: int) -> Tuple[bool, Optional[Dict[str, int]]]:
    """ONE rep's stamp-gate drop counts, as (receipt_expected, counts).

    THREE outcomes, kept apart because they mean different things and the
    caller acts differently on each:

      (False, None)  no receipt, and none was owed — this leg writes none
                     (every native and workspace cell). The column does
                     not apply.
      (True,  None)  a receipt was OWED and could not be read: it is
                     missing although the ROS 2 driver ran this cell, or
                     it exists and is unreadable, or it carries no
                     `DELIVERY role=latency` line (the nodes died before
                     printing one — run_bench.sh writes the file anyway,
                     saying so), or a key is missing or not a whole
                     number. NOT zero, and NOT blank: "the sink never
                     reported" and "the sink reported no drops" are
                     opposite claims, and so are "no receipt was owed" and
                     "one was owed and is gone".
      (True,  {...}) every key in DELIVERY_DROP_KEYS, parsed.

    The first flag is EXPECTATION, not file presence, and that is the
    whole of it: a ROS 2 cell whose receipts vanished would otherwise
    publish the EMPTY column, whose published meaning is "this leg writes
    none" — so a cell that lost its accounting would read as a valid row
    with the question marked inapplicable. Reproduced with an executed
    harness on review: two contributing ROS 2 reps with samples and no
    receipts compiled to blank cells and exit 0.

    Extraction is WHOLE-TOKEN, the same discipline run_bench.sh's
    `receipt_count` had to learn: the receipt is whitespace-separated
    `key=value` tokens, the key is compared whole (so `retries_unstamped=7`
    can never answer for `unstamped`) and the value is everything after the
    FIRST `=` and must be entirely digits, so corrupted text fails CLOSED
    to "could not read" rather than to a plausible number."""
    path = delivery_log_path(raw_dir, cell, size)
    expected = _receipt_was_expected(raw_dir, cell, size)
    # stat EXPLICITLY rather than `path.exists()`. `exists()` answers False
    # for ENOENT *and* for ENOTDIR / ELOOP / EBADF — so a symlink loop or a
    # `_logs` that is a regular file would publish the EMPTY column, whose
    # documented meaning is "this leg writes no receipt", when the truth is
    # "I could not look". It also RE-RAISES EACCES, and `/raw` is a bind
    # mount written by a root container: a root-owned 0700 `_logs` would
    # kill the whole compile with a traceback rather than mark one row
    # unknown.
    try:
        path.stat()
    except FileNotFoundError:
        # Absent. Whether that is "does not apply" or "owed and gone" is
        # decided by whether the receipt-writing driver ran here.
        return (expected, None)
    except OSError:
        return (True, None)
    try:
        text = path.read_text(errors="replace")
    except OSError:
        return (True, None)
    line = next((ln for ln in text.splitlines()
                 if ln.startswith("DELIVERY role=latency ")
                 or ln == "DELIVERY role=latency"), None)
    if line is None:
        return (True, None)
    counts: Dict[str, int] = {}
    for token in line.split():
        key, sep, value = token.partition("=")
        # `isascii()` as well as `isdigit()`: `str.isdigit()` is true for
        # Unicode digit forms `int()` REFUSES (a superscript `²`, say), so
        # a corrupted receipt carrying one raised ValueError out of this
        # function and took the whole compile down with a traceback —
        # while this function's contract, two paragraphs up, is that junk
        # fails CLOSED to "could not read".
        if (sep and key in DELIVERY_DROP_KEYS
                and value.isascii() and value.isdigit()):
            counts[key] = int(value)
    if len(counts) != len(DELIVERY_DROP_KEYS):
        return (True, None)
    return (True, counts)


def delivery_drops_across_reps(
        per_rep: List[Tuple[str, bool, Optional[Dict[str, int]]]],
        cell: str, size: int) -> Dict[str, str]:
    """The `unstamped` / `nonpositive_rtt` cells across reps — POOLED, the
    way `iterations` is, because these counts and the sample count are
    terms of one identity (received == samples + warmup + unstamped +
    nonpositive_rtt) and a row that pools one term and averages another
    describes no run.

    PURE over the per-rep verdicts (`(rep_label, present, counts)`) so the
    decision can be driven with hand vectors; the reading is the one line
    above it.

    The whole row's verdict is ONE decision, not one per column, because
    both counters are printed by one receipt line: a rep that cannot
    answer for `unstamped` cannot answer for `nonpositive_rtt` either, and
    letting the two columns disagree would invent a distinction the data
    does not carry."""
    if not per_rep or not any(present for _lbl, present, _c in per_rep):
        # No rep has a receipt at all: the column does not apply here.
        return {key: "" for key in DELIVERY_DROP_KEYS}
    silent = sorted(lbl for lbl, present, _c in per_rep if not present)
    unreadable = sorted(lbl for lbl, present, counts in per_rep
                        if present and counts is None)
    if silent or unreadable:
        why = []
        if silent:
            why.append(f"no receipt in {', '.join(silent)}")
        if unreadable:
            why.append(f"no parseable count in {', '.join(unreadable)}")
        sys.stderr.write(
            f"warn: {cell} payload {size}: the stamp-gate drop counts "
            f"cannot be totalled ({'; '.join(why)}) — some rep(s) DID "
            f"report, so writing '{DROPS_UNKNOWN}' rather than a sum over "
            f"the reps that answered, which would under-report drops in "
            f"the one direction that flatters the row\n")
        return {key: DROPS_UNKNOWN for key in DELIVERY_DROP_KEYS}
    return {key: str(sum(counts[key] for _lbl, _p, counts in per_rep))
            for key in DELIVERY_DROP_KEYS}


def achieved_rate_across_reps(rates_by_rep: Dict[str, str], cell: str,
                              size: int) -> str:
    """The achieved_rate_hz column value across reps (fixed100 sidecars;
    same discipline as the loaned column): all reps must agree — a
    disagreement (one rep sustained 100 Hz, another fell to a ladder
    rung) is a real signal, warned loudly and written as 'mixed', never
    silently collapsed to either value."""
    distinct = set(rates_by_rep.values())
    if not distinct:
        return ""
    if len(distinct) == 1:
        return distinct.pop()
    sys.stderr.write(
        f"warn: {cell} payload {size}: achieved rate disagrees across reps "
        f"({', '.join(f'{k}={v}' for k, v in sorted(rates_by_rep.items()))})"
        f" — the fixed100 ladder settled differently per rep; writing "
        f"'mixed' (plots annotate it)\n")
    return "mixed"


def csv_path_for(out_dir: Path, cell: str) -> Path:
    """raw_prefix → results_<raw_prefix>.csv. No special cases: the coupling
    with plot.py's expected-CSV derivation is name-identity."""
    return out_dir / f"results_{cell}.csv"


# ---------------------------------------------------------------- main


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    p.add_argument("--run-dir", type=Path, default=None,
                   help="Run directory: discovers rep<k>/raw/ subdirs and "
                        "aggregates across reps (legacy rep-less dirs fall "
                        "back to <run-dir>/raw/); reads run.json for the "
                        "pacing variant")
    p.add_argument("--raw-dir", type=Path, default=None,
                   help="Explicit single raw .bin dir (legacy single-rep "
                        "interface)")
    p.add_argument("--out-dir", type=Path, default=None,
                   help="CSV output dir (default: the run dir, or the raw "
                        "dir's parent under --raw-dir)")
    p.add_argument("--quiet", action="store_true",
                   help="suppress per-file progress")
    p.add_argument("--allow-partial", action="store_true",
                   help="Demote missing/0-sample/wrong-sample-count payload "
                        "rows from a hard nonzero exit to loud stderr "
                        "warnings (deliberately partial sweeps)")
    args = p.parse_args(argv)

    if (args.run_dir is None) == (args.raw_dir is None):
        sys.stderr.write("error: pass exactly one of --run-dir / --raw-dir\n")
        return 2

    variant: Optional[str] = None
    if args.raw_dir is not None:
        if not args.raw_dir.is_dir():
            sys.stderr.write(f"error: {args.raw_dir} is not a directory\n")
            return 2
        raw_dirs: List[Tuple[str, Path]] = [("raw", args.raw_dir)]
        out_dir = (args.out_dir if args.out_dir is not None
                   else args.raw_dir.parent)
        variant = read_manifest_variant(args.raw_dir.parent)
    else:
        if not args.run_dir.is_dir():
            sys.stderr.write(f"error: {args.run_dir} is not a directory\n")
            return 2
        raw_dirs = discover_rep_raw_dirs(args.run_dir)
        if not raw_dirs:
            sys.stderr.write(
                f"error: {args.run_dir} contains neither rep<k>/raw/ "
                f"subdirs nor a legacy raw/ dir\n")
            return 2
        out_dir = args.out_dir if args.out_dir is not None else args.run_dir
        variant = read_manifest_variant(args.run_dir)

    # The expected sample counts must describe the RUN, never the shell
    # that post-processes it: `bench.samples_for` reads CER_BENCH_SMOKE_N /
    # CER_BENCH_TARGET_SAMPLES / CER_BENCH_WARMUP from the environment, so
    # an exported bench variable would otherwise reject a valid run (or
    # wave a bad one through under --allow-partial). bench.py refuses the
    # same variables on every producing subcommand; refuse them here too,
    # from whichever entry point — the counts then follow only from
    # (variant, payload). An unrecoverable variant takes the strictest
    # reading.
    try:
        bench.refuse_ambient_sample_overrides("compile-csv", variant,
                                              produces=False)
    except SystemExit as e:
        # Exit 2 = "you invoked this wrong", which is what a dirty shell is.
        # A bare SystemExit(str) would exit 1, and 1 already means "the run
        # has bad rows" here — two very different problems for the caller
        # (bench.py returns this code verbatim) to tell apart.
        sys.stderr.write(f"error: {e}\n")
        return 2

    rep_labels = [label for label, _ in raw_dirs]
    if not args.quiet and len(raw_dirs) > 1:
        sys.stdout.write(f"compile_csv: aggregating {len(raw_dirs)} reps "
                         f"({', '.join(rep_labels)})\n")

    # Group .bin files by cell → size → rep_label.
    grouped: Dict[CellKey, Dict[int, Dict[str, Path]]] = defaultdict(dict)
    for rep_label, raw_dir in raw_dirs:
        for path in sorted(raw_dir.glob("*.bin")):
            parsed = parse_filename(path)
            if not parsed:
                sys.stderr.write(
                    f"warn: skipping unparseable filename {path.name} "
                    f"(expected <cell>_chrt<0|1>_<size>.bin)\n")
                continue
            key, size = parsed
            grouped[key].setdefault(size, {})[rep_label] = path

    # fixed100 achieved-rate sidecars, same grouping. A did_not_sustain
    # payload has a sidecar and NO .bin, so it can introduce a (cell,
    # size) — even a cell — that the .bin walk never saw; the row loop
    # walks the UNION so the correct empty row is still written.
    rates: Dict[CellKey, Dict[int, Dict[str, str]]] = defaultdict(dict)
    for rep_label, raw_dir in raw_dirs:
        for path in sorted(raw_dir.glob("*.rate")):
            parsed = parse_rate_filename(path)
            if not parsed:
                sys.stderr.write(
                    f"warn: skipping unparseable sidecar {path.name} "
                    f"(expected <cell>_chrt<0|1>_<size>.rate)\n")
                continue
            value = read_rate_sidecar(path)
            if value is None:
                continue
            key, size = parsed
            rates[key].setdefault(size, {})[rep_label] = value

    if not grouped and not rates:
        sys.stderr.write(
            f"error: no .bin files matched the naming pattern under "
            f"{args.run_dir or args.raw_dir}\n")
        return 1

    # Strict row accounting: every discovered prefix is expected to carry
    # the full pinned sweep in EVERY discovered rep, every present .bin
    # must have samples, and (A3) the sample count must be EXACTLY the
    # schedule's measured count for the run's variant.
    expected_sizes = tuple(bench.PAYLOAD_SIZES)
    bad_rows: List[Tuple[str, int, str]] = []   # (cell, payload, why)
    n_suppressed_cells = 0
    rows_with_suppression = 0

    def expected_count(size: int) -> Optional[int]:
        if variant is None:
            return None
        return bench.samples_for(variant, size)[1]

    written = 0
    all_keys = sorted(set(grouped) | set(rates), key=lambda k: k.cell)
    n_dns_rows = 0
    for key in all_keys:
        by_size = grouped.get(key, {})
        rates_by_size = rates.get(key, {})
        for want in expected_sizes:
            for rep_label in rep_labels:
                if rep_label in by_size.get(want, {}):
                    continue
                # A did_not_sustain sidecar ACCOUNTS for the missing .bin
                # (fixed100: the ladder was exhausted; the correct empty
                # row is written below) — a rate-valued sidecar with no
                # .bin stays a bad row (it claims a sustained run that
                # left no samples).
                if rates_by_size.get(want, {}).get(rep_label) \
                        == DID_NOT_SUSTAIN:
                    continue
                where = (f"missing .bin in {rep_label}"
                         if len(rep_labels) > 1 else "missing .bin")
                bad_rows.append((key.cell, want,
                                 f"{where} (payload never produced, or "
                                 f"the cell failed there)"))
        out_path = csv_path_for(out_dir, key.cell)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        rows_written = 0
        with out_path.open("w") as f:
            f.write(CSV_COMMENT)
            f.write(CSV_HEADER)
            for size in sorted(set(by_size) | set(rates_by_size)):
                rep_samples: List[List[int]] = []
                # The (label, dir) of every rep whose samples reached this
                # row — the SAME set `iterations` and `rep_count` are
                # pooled over. The drop counts must use it too: they are
                # terms of the identity the row publishes, and a rep that
                # contributed no `received` cannot make THIS row's total
                # unobtainable. Pooling over every DISCOVERED rep instead
                # made one crashed rep of a `--allow-partial` campaign
                # print `unknown` on every row of every ROS 2 cell, which
                # is how a real `unknown` stops being read.
                contributing: List[Tuple[str, Path]] = []
                for rep_label in rep_labels:
                    bin_path = by_size.get(size, {}).get(rep_label)
                    if bin_path is None:
                        continue   # already accounted above
                    try:
                        samples = read_samples_ns(bin_path)
                    except ValueError as e:
                        # Torn dump: a bad row (strict → nonzero exit;
                        # --allow-partial → the loud warn path), never a
                        # truncated-then-accepted percentile.
                        bad_rows.append((key.cell, size, str(e)))
                        continue
                    rel = (f"{rep_label}/{bin_path.name}"
                           if len(rep_labels) > 1 else bin_path.name)
                    if not samples:
                        bad_rows.append((key.cell, size,
                                         f"0 samples in {rel}"))
                        continue
                    want_n = expected_count(size)
                    if want_n is not None and len(samples) != want_n:
                        bad_rows.append(
                            (key.cell, size,
                             f"{len(samples)} samples in {rel} — the "
                             f"{variant} schedule pins {want_n} measured "
                             f"samples (an under-sampled cell is a failed "
                             f"cell, never a quietly thinner percentile)"))
                        # EXCLUDE it: a wrong-count file must not reach
                        # rep_samples. Aggregating it would write a
                        # percentile row computed from data this function
                        # just declared invalid — and under --allow-partial
                        # that misleading CSV would ship with exit 0.
                        continue
                    if want_n is None:
                        known = {bench.samples_for(v, size)[1]
                                 for v in bench.PACING_VARIANTS}
                        if len(samples) not in known:
                            sys.stderr.write(
                                f"warn: {key.cell} payload {size}: "
                                f"{len(samples)} samples in {rel} matches "
                                f"NO variant's schedule "
                                f"({', '.join(map(str, sorted(known)))}) "
                                f"and the variant is unrecoverable (no "
                                f"run.json) — treat this row with "
                                f"suspicion\n")
                    if rates_by_size.get(size, {}).get(rep_label) \
                            == DID_NOT_SUSTAIN:
                        # THIS rep declared the ladder exhausted and still
                        # left samples. Checked per REP, not on the
                        # aggregated verdict: with one rep exhausted and
                        # another sustaining, `achieved` collapses to
                        # "mixed" and the aggregate check below never
                        # fires, so a stale `.bin` would be pooled into a
                        # real percentile row. An exhausted rep mints no
                        # latency by definition — the runner deletes its
                        # `.bin` — so samples beside that verdict are the
                        # contradiction regardless of what the other reps
                        # did.
                        bad_rows.append(
                            (key.cell, size,
                             f"{rel} carries samples while its own rep "
                             f"declares did_not_sustain — contradictory "
                             f"fixed100 artifacts; re-run this cell "
                             f"(rep excluded)"))
                        continue
                    rep_samples.append(samples)
                    # The rep's own raw dir, taken from the .bin that was
                    # just read — NOT from a `raw_dir` name, which in this
                    # scope is whatever the enclosing discovery loop left
                    # bound and is the SAME directory for every rep. That
                    # bug published rep N's receipt N times (measured:
                    # two reps reporting 7 and 5 published 10).
                    contributing.append((rep_label, bin_path.parent))
                achieved = achieved_rate_across_reps(
                    rates_by_size.get(size, {}), key.cell, size)
                if achieved == DID_NOT_SUSTAIN and rep_samples:
                    # Every rep that HAS a sidecar says the ladder was
                    # exhausted, yet samples survived the per-rep filter
                    # above — so they came from a rep with no sidecar at
                    # all, which a fixed100 run always writes. Contradictory
                    # at the cell level. Drop the samples rather than
                    # aggregate them, and fall through to the correct empty
                    # row below: excluding the payload ENTIRELY would make
                    # it indistinguishable from one that never ran, and
                    # would leave a silent gap in the figure where the
                    # did_not_sustain annotation belongs.
                    bad_rows.append(
                        (key.cell, size,
                         "every rate sidecar says did_not_sustain but a "
                         "rep with no sidecar left samples — contradictory "
                         "fixed100 artifacts; re-run this cell (no latency "
                         "minted)"))
                    rep_samples = []
                if not rep_samples:
                    if achieved == DID_NOT_SUSTAIN:
                        # fixed100 ladder exhausted in EVERY rep: the
                        # correct EMPTY row — payload + chrt +
                        # did_not_sustain, every stat cell blank. Loud,
                        # and never a fabricated latency (Principle #13).
                        # The two drop columns are blank here for the same
                        # reason `loaned` is: this row describes no
                        # measured run, and the receipt that may exist
                        # describes the fallback ladder's LAST RUNG (each
                        # rung is its own container writing over the same
                        # path), which is not what the row is about.
                        f.write(f"{size},0,,,,,{key.chrt},,,,,,,,,,,0,,,"
                                f"{DID_NOT_SUSTAIN}"
                                + "," * len(DELIVERY_DROP_KEYS) + "\n")
                        rows_written += 1
                        n_dns_rows += 1
                        sys.stderr.write(
                            f"note: {key.cell} payload {size}: DID NOT "
                            f"SUSTAIN the fixed100 fallback ladder — empty "
                            f"row written (no latency minted)\n")
                    continue
                stats = aggregate_reps(rep_samples)
                pooled_n = stats["iterations"]

                def cell_value(col: str) -> str:
                    q = PERCENTILE_QUANTILES.get(col)
                    if q is not None and not percentile_is_citable(pooled_n, q):
                        return ""
                    return str(stats[col])

                cols = {c: cell_value(c) for c in PERCENTILE_QUANTILES}
                suppressed_here = sum(1 for v in cols.values() if v == "")
                if suppressed_here:
                    n_suppressed_cells += suppressed_here
                    rows_with_suppression += 1
                # one_way is derived FROM the headline p50 — suppressed with it.
                one_way = (str(stats["one_way_p50"]) if cols["p50"] != ""
                           else "")
                loaned = loaned_across_reps(raw_dirs, key.cell, size)
                drops = delivery_drops_across_reps(
                    [(rep_label,) + delivery_drops_from_log(raw_dir,
                                                            key.cell, size)
                     for rep_label, raw_dir in contributing],
                    key.cell, size)
                f.write(
                    f"{size},{pooled_n},{cols['p50']},{cols['p99']},"
                    f"{stats['mean']},{one_way},{key.chrt},{loaned},"
                    f"{stats['floor']},{cols['p1']},{cols['p10']},"
                    f"{cols['p25']},{cols['p75']},{cols['p90']},"
                    f"{cols['p95']},{cols['p99_9']},{stats['max']},"
                    f"{stats['rep_count']},{stats['rep_p50_min']},"
                    f"{stats['rep_p50_max']},{achieved},"
                    + ",".join(drops[k] for k in DELIVERY_DROP_KEYS) + "\n"
                )
                rows_written += 1

        if not args.quiet:
            try:
                display = out_path.relative_to(out_dir.parent)
            except ValueError:
                display = out_path
            sys.stdout.write(f"{display}: {rows_written} rows\n")
        written += 1

    if n_suppressed_cells:
        sys.stderr.write(
            f"note: SUPPRESSED {n_suppressed_cells} percentile cell(s) "
            f"across {rows_with_suppression} row(s) — fewer than "
            f"{MIN_TAIL_EXCEEDANCES} pooled samples back those tails at "
            f"this n, so the cells are written empty rather than citable-"
            f"looking. Add reps (bench.py full --reps N) or "
            f"extend the schedule to make them citable.\n")

    if n_dns_rows:
        sys.stderr.write(
            f"note: {n_dns_rows} row(s) DID NOT SUSTAIN the fixed100 "
            f"fallback ladder — written as empty did_not_sustain rows "
            f"(no latency minted; plots render the gap with an in-figure "
            f"note).\n")

    if bad_rows:
        sev = "warn" if args.allow_partial else "error"
        for cell, size, why in sorted(bad_rows):
            sys.stderr.write(f"{sev}: {cell} payload {size}: {why}\n")
        if not args.allow_partial:
            n_cells = len({c for c, _, _ in bad_rows})
            sys.stderr.write(
                f"error: {len(bad_rows)} bad (prefix, payload) row(s) "
                f"across {n_cells} cell(s) — every discovered cell must "
                f"carry the full {len(expected_sizes)}-size sweep in every "
                f"rep at the schedule's exact measured sample count "
                f"(strict by default; CSVs for the good rows were "
                f"written). Re-run the failed cells via bench.py, or pass "
                f"--allow-partial to accept a partial artifact loudly.\n")
            return 1

    if not args.quiet:
        sys.stdout.write(f"compile_csv: wrote {written} CSV files\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
