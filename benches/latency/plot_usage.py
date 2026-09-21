#!/usr/bin/env python3
"""Render CPU + memory usage companion figures from .usage.csv sidecars.

Standalone on purpose: plot.py's strict group machinery
(posture verification, pinned six-line hero sets, expected-CSV gates)
assumes every cell of a group was measured; usage sidecars land sparsely
(CER_BENCH_USAGE=1 is opt-in and Linux-only), so this renderer draws
exactly what exists and says what is missing, instead of refusing.

Input:  a results run dir (results/<hash>-<date>-<variant>/) — every
        `<cell>_<size>.usage.csv` under it (rep dirs + legacy layouts,
        found recursively) written by usage_sampler.py via
        bench.py / run_workspace.sh.
Output: <out-dir>/usage_cpu.png     steady-window mean of the cell's
                                    SUMMED per-process CPU (% of one
                                    core) vs payload
        <out-dir>/usage_mem.png     steady-window mean summed PSS (MiB)
                                    vs payload — the accurate cross-
                                    process number for SHM-heavy cells
                                    — with summed RSS as a lighter
                                    dashed twin (labeled; RSS double-
                                    counts shared mappings, shown so
                                    the double-count is visible, not
                                    quoted)
        <out-dir>/usage_summary.csv one row per (cell, size)
        stdout                      the same table, human-readable

Steady window: samples inside [t0 + 20% span, t1 − 5% span] — bring-up
(graph build, discovery, container start) and teardown are excluded
from the mean. Per-tick totals sum across the cell's sampled processes.

PSS is ALL-OR-NOTHING (Principle #13): a summed PSS is correct only when
it covers the WHOLE process tree, so a cell's PSS is UNMEASURED — not a
partial sum — unless (a) the sidecar's sampler_self footer is present
with pss_denied=0 (a torn/killed sampler proves nothing about coverage)
AND (b) every PSS-bearing steady tick carried a PSS value for every
sampled process. A partial-denial tree (e.g. root-owned chrt/sudo bench
processes under an unprivileged sampler) summed only its readable
minority would fabricate a LOW memory number; instead the mem figure
draws that cell's RSS with a loud "(PSS unavailable — permission
denial; showing RSS)" label. Docker cells whose whole tree is denied
carry `scope=cgroup` fallback rows (pid=0, comm=cgroup — the container
TOTAL anon+file from memory.stat, see usage_sampler.py); those render
as their own labeled series and are never summed with per-proc rows —
a cgroup total and per-process PSS are not like-for-like.

Rep dirs are ordered NUMERICALLY (rep10 after rep2), and the newest
rep's sidecar wins for a (cell, size); a cross-rep aggregation can come
later if usage ever gates anything.

No number is fabricated: a cell×size with no sidecar is simply not a
point; the summary row says pss=UNMEASURED where it is.

Self-check: `python3 plot_usage.py --self-test` drives parse+summarize
over synthetic sidecars, including the partial-denial shape that must
read UNMEASURED (the adversarial-review reproduction).
"""

import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

NAME_RE = re.compile(r"^(?P<cell>.+_chrt[01])_(?P<size>\d+)\.usage\.csv$")
REP_RE = re.compile(r"^rep(\d+)$")
PSS_DENIED_RE = re.compile(r"\bpss_denied=(\d+)\b")

# Okabe-Ito (CVD-safe) fixed categorical order — assigned to cells by
# first-seen sorted order. Past 8 cells the hues CYCLE with a distinct
# per-lap dash pattern (loudly noted on stderr) — a 12-cell full-
# campaign render must degrade to hue+dash disambiguation, never refuse.
PALETTE = ["#0072B2", "#E69F00", "#009E73", "#D55E00",
           "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
CYCLE_DASHES = [(None, None), (6, 2), (2, 2), (6, 2, 2, 2)]


def style_for(i: int):
    """(color, dashes) for series index i: fixed hues, then hue reuse
    disambiguated by dash pattern per palette lap."""
    color = PALETTE[i % len(PALETTE)]
    dashes = CYCLE_DASHES[(i // len(PALETTE)) % len(CYCLE_DASHES)]
    return color, dashes


def rep_sort_key(path: Path) -> tuple:
    """Numeric rep ordering: rep10 sorts AFTER rep2 (lexicographic rglob
    order put rep10 first, silently letting rep2's sidecar win)."""
    rep = 0
    for part in path.parts:
        m = REP_RE.match(part)
        if m:
            rep = int(m.group(1))
    return (rep, str(path))


def human_size(n: int) -> str:
    if n >= 1 << 20:
        return f"{n // (1 << 20)}M"
    if n >= 1 << 10:
        return f"{n // (1 << 10)}K"
    return str(n)


def parse_sidecar(path: Path) -> Optional[dict]:
    """-> {meta, rows, cgroup_rows, self_line, pss_denied}.

    rows are per-process samples (ts_ns,pid,comm,cpu,rss_kb,pss_kb);
    cgroup_rows are the docker whole-tree-denial fallback rows
    (comm == 'cgroup', pid == 0 — container-total anon+file in the
    pss_kb column), kept SEPARATE so a cgroup total can never be summed
    into per-proc totals. pss_denied is the sampler_self footer count,
    or None when the footer is absent (torn/killed sampler — PSS
    coverage unprovable)."""
    rows: List[Tuple[int, int, str, Optional[float], Optional[int],
                     Optional[int]]] = []
    cgroup_rows: List[Tuple[int, int]] = []   # (ts_ns, anon+file kB)
    meta: List[str] = []
    self_line = None
    pss_denied: Optional[int] = None
    try:
        text = path.read_text()
    except OSError as e:
        print(f"warn: {path}: unreadable ({e}) — skipped", file=sys.stderr)
        return None
    for line in text.splitlines():
        if line.startswith("#"):
            if line.startswith("# sampler_self:"):
                self_line = line
                m = PSS_DENIED_RE.search(line)
                if m:
                    pss_denied = int(m.group(1))
            else:
                meta.append(line)
            continue
        if line.startswith("ts_ns") or not line.strip():
            continue
        p = line.split(",")
        if len(p) != 6:
            continue   # torn tail from a killed sampler — fine
        try:
            if p[2] == "cgroup" and p[1] == "0":
                if p[5]:
                    cgroup_rows.append((int(p[0]), int(p[5])))
                continue
            rows.append((int(p[0]), int(p[1]), p[2],
                         float(p[3]) if p[3] else None,
                         int(p[4]) if p[4] else None,
                         int(p[5]) if p[5] else None))
        except ValueError:
            continue
    if not rows:
        print(f"warn: {path.name}: no data rows — skipped", file=sys.stderr)
        return None
    return {"meta": meta, "rows": rows, "cgroup_rows": cgroup_rows,
            "self": self_line, "pss_denied": pss_denied}


def summarize(parsed: dict) -> dict:
    """Steady-window per-tick totals -> means. See module docstring.

    PSS is served ONLY under provable full coverage: the sampler_self
    footer must be present with pss_denied=0, AND every PSS-bearing
    steady tick must carry a PSS value for every process it sampled.
    Anything less — a torn footer, any denial, a partial tick — is
    pss UNMEASURED (None) with the reason recorded, never a partial
    sum (a root-owned chrt tree summed only its readable bash/timeout
    minority would fabricate a LOW memory number)."""
    rows = parsed["rows"]
    pss_denied = parsed["pss_denied"]
    t0, t1 = rows[0][0], rows[-1][0]
    span = t1 - t0
    lo, hi = t0 + span * 0.20, t1 - span * 0.05
    by_tick: Dict[int, list] = defaultdict(list)
    for r in rows:
        if lo <= r[0] <= hi:
            by_tick[r[0]].append(r)
    cpu_totals, rss_totals, pss_totals = [], [], []
    pss_partial_ticks = 0
    for ts in sorted(by_tick):
        tick = by_tick[ts]
        cpus = [r[3] for r in tick if r[3] is not None]
        if cpus:
            cpu_totals.append(sum(cpus))
        rsss = [r[4] for r in tick if r[4] is not None]
        if rsss:
            rss_totals.append(sum(rsss))
        psss = [r[5] for r in tick if r[5] is not None]
        if psss:
            if len(psss) < len(tick):
                pss_partial_ticks += 1   # a partial sum is not a sum
            else:
                pss_totals.append(sum(psss))
    if pss_denied is None:
        pss_reason = "footer absent — coverage unprovable"
        pss_totals = []
    elif pss_denied > 0:
        pss_reason = f"permission denial (pss_denied={pss_denied})"
        pss_totals = []
    elif pss_partial_ticks:
        pss_reason = (f"{pss_partial_ticks} steady tick(s) with partial "
                      f"PSS coverage")
        pss_totals = []
    else:
        pss_reason = None
    # cgroup fallback rows (docker whole-tree denial): container-total
    # anon+file, its OWN series — never summed with per-proc rows.
    cg = [kb for ts, kb in parsed.get("cgroup_rows", ())
          if lo <= ts <= hi] or [kb for _, kb in parsed.get(
              "cgroup_rows", ())]   # tiny sidecars: all cgroup rows
    mean = lambda xs: (sum(xs) / len(xs)) if xs else None  # noqa: E731
    nproc = len({r[1] for rs in by_tick.values() for r in rs})
    return {
        "cpu_pct_mean": mean(cpu_totals),
        "rss_mib_mean": (mean(rss_totals) or 0) / 1024 if rss_totals else None,
        "pss_mib_mean": (mean(pss_totals) or 0) / 1024 if pss_totals else None,
        "pss_unmeasured_reason": pss_reason,
        "cgroup_mib_mean": (mean(cg) or 0) / 1024 if cg else None,
        "ticks": len(by_tick),
        "procs_seen": nproc,
    }


def _synthetic_sidecar(path: Path, procs, *, pss_denied: int,
                       footer: bool = True, cgroup_kb: Optional[int] = None,
                       ticks: int = 30) -> None:
    """Write a synthetic sidecar. procs = [(pid, comm, cpu, rss_kb,
    pss_kb_or_None)]; every 5th tick is a PSS tick (a proc with pss None
    on a PSS tick models a denied/unreadable read)."""
    with path.open("w") as f:
        f.write("# usage_sampler v1 scope=procs\n")
        f.write("# target: self-test fixture\n")
        f.write("ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb\n")
        for t in range(ticks):
            ts = 1_000_000_000 + t * 200_000_000
            do_pss = (t % 5) == 0
            for pid, comm, cpu, rss, pss in procs:
                pss_s = str(pss) if (do_pss and pss is not None) else ""
                f.write(f"{ts},{pid},{comm},{cpu:.1f},{rss},{pss_s}\n")
            if do_pss and cgroup_kb is not None:
                f.write(f"{ts},0,cgroup,,,{cgroup_kb}\n")
        if footer:
            f.write(f"# sampler_self: cpu_s=0.1 wall_s=6.0 "
                    f"cpu_pct_of_one_core=1.5 rows={ticks * len(procs)} "
                    f"ticks={ticks} pss_denied={pss_denied} cgroup_rows=0 "
                    f"discovery=static\n")


def self_test() -> int:
    """Pin the PSS labeling gate against the adversarial-review shapes.

    Shape A: a chrt1-shaped tree — the
    root-owned cerulion processes PSS-denied, the bash/timeout wrappers
    readable. A partial sum (the pre-fix bug) reported ~1 MiB for a
    tree whose real memory was unreadable; the gate must say UNMEASURED.
    Shape B: clean full coverage — PSS is served. Shape C: footer says
    pss_denied=0 but one PSS tick has partial coverage (death mid-tick)
    — UNMEASURED. Shape D: footer torn off (killed sampler) —
    UNMEASURED. Shape E: docker whole-tree denial with cgroup fallback
    rows — pss UNMEASURED, cgroup served as its own scope."""
    import tempfile
    errors = 0
    with tempfile.TemporaryDirectory(prefix="usage_st_") as td:
        d = Path(td)

        def run(name, procs, **kw):
            p = d / name
            _synthetic_sidecar(p, procs, **kw)
            parsed = parse_sidecar(p)
            assert parsed is not None, name
            return summarize(parsed)

        # A: denied cerulion tree, readable wrappers.
        sA = run("a_chrt1_1024.usage.csv",
                 [(10, "cerulion", 99.0, 40_000, None),
                  (11, "cerulion", 50.0, 30_000, None),
                  (12, "bash", 0.1, 800, 700),
                  (13, "timeout", 0.1, 400, 380)],
                 pss_denied=12)
        if sA["pss_mib_mean"] is not None:
            print(f"FAIL  [partial-denial tree served a PSS: "
                  f"{sA['pss_mib_mean']:.2f} MiB — the fabricated-low-sum "
                  f"bug]")
            errors += 1
        elif "denial" not in (sA["pss_unmeasured_reason"] or ""):
            print(f"FAIL  [denial shape: wrong reason "
                  f"{sA['pss_unmeasured_reason']!r}]")
            errors += 1
        else:
            print("ok    [partial permission denial => pss UNMEASURED, "
                  "never a partial sum]")
        if sA["rss_mib_mean"] is None:
            print("FAIL  [denial shape lost its RSS fallback]")
            errors += 1

        # B: clean coverage — PSS served.
        sB = run("b_clean_1024.usage.csv",
                 [(20, "cerulion", 99.0, 40_000, 20_000),
                  (21, "cerulion", 50.0, 30_000, 15_000)],
                 pss_denied=0)
        if sB["pss_mib_mean"] is None:
            print(f"FAIL  [clean shape should serve PSS: "
                  f"{sB['pss_unmeasured_reason']!r}]")
            errors += 1
        elif abs(sB["pss_mib_mean"] - 35_000 / 1024) > 0.01:
            print(f"FAIL  [clean PSS wrong: {sB['pss_mib_mean']:.2f}]")
            errors += 1
        else:
            print("ok    [full coverage => PSS served (35000 kB summed)]")

        # C: footer clean but one proc's PSS missing on PSS ticks.
        sC = run("c_partial_1024.usage.csv",
                 [(30, "cerulion", 99.0, 40_000, 20_000),
                  (31, "cerulion", 50.0, 30_000, None)],
                 pss_denied=0)
        if sC["pss_mib_mean"] is not None:
            print("FAIL  [partial-coverage tick served a PSS]")
            errors += 1
        else:
            print("ok    [partial per-tick coverage => pss UNMEASURED]")

        # D: torn footer — coverage unprovable.
        sD = run("d_torn_1024.usage.csv",
                 [(40, "cerulion", 99.0, 40_000, 20_000)],
                 pss_denied=0, footer=False)
        if sD["pss_mib_mean"] is not None:
            print("FAIL  [torn-footer sidecar served a PSS]")
            errors += 1
        else:
            print("ok    [absent sampler_self footer => pss UNMEASURED]")

        # E: docker whole-tree denial + cgroup fallback rows.
        sE = run("e_docker_1024.usage.csv",
                 [(50, "ros2_bench", 99.0, 500_000, None)],
                 pss_denied=6, cgroup_kb=480_000)
        if sE["pss_mib_mean"] is not None:
            print("FAIL  [cgroup shape served a per-proc PSS]")
            errors += 1
        elif sE["cgroup_mib_mean"] is None:
            print("FAIL  [cgroup fallback rows not picked up]")
            errors += 1
        elif abs(sE["cgroup_mib_mean"] - 480_000 / 1024) > 0.01:
            print(f"FAIL  [cgroup value wrong: {sE['cgroup_mib_mean']:.2f}]")
            errors += 1
        else:
            print("ok    [docker whole-tree denial => cgroup total served "
                  "as its own scope, pss UNMEASURED]")

    if errors:
        print(f"self-test FAIL: {errors} error(s)", file=sys.stderr)
        return 1
    print("self-test PASS")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run-dir", type=Path,
                    help="results/<hash>-<date>-<variant>/ (searched "
                         "recursively for *.usage.csv)")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="default: <run-dir>/plots")
    ap.add_argument("--self-test", action="store_true",
                    help="drive parse+summarize over synthetic sidecars "
                         "(incl. the partial-denial shape that must read "
                         "UNMEASURED); no run dir needed")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.run_dir is None:
        ap.error("--run-dir is required (or use --self-test)")
    run_dir: Path = args.run_dir
    if not run_dir.is_dir():
        raise SystemExit(f"--run-dir {run_dir} is not a directory")
    out_dir = args.out_dir or (run_dir / "plots")

    # Numeric rep order (rep10 AFTER rep2 — lexicographic rglob order let
    # rep2 win over rep10): the newest rep's sidecar wins for a
    # (cell, size); a cross-rep aggregation can come later if usage ever
    # gates anything.
    sidecars = sorted(run_dir.rglob("*.usage.csv"), key=rep_sort_key)
    if not sidecars:
        raise SystemExit(f"no *.usage.csv sidecars under {run_dir} — run "
                         f"with CER_BENCH_USAGE=1 first (nothing is "
                         f"fabricated from their absence)")

    data: Dict[str, Dict[int, dict]] = defaultdict(dict)
    self_lines: List[str] = []
    for path in sidecars:
        m = NAME_RE.match(path.name)
        if not m:
            print(f"warn: {path.name}: unrecognized sidecar name "
                  f"(expected <cell>_chrt<0|1>_<size>.usage.csv) — skipped",
                  file=sys.stderr)
            continue
        parsed = parse_sidecar(path)
        if parsed is None:
            continue
        s = summarize(parsed)
        data[m["cell"]][int(m["size"])] = s
        if parsed["self"]:
            self_lines.append(f"{path.name}: {parsed['self']}")

    if not data:
        raise SystemExit("no parseable sidecars — nothing to plot")
    cells = sorted(data)
    if len(cells) > len(PALETTE):
        print(f"note: {len(cells)} cells > {len(PALETTE)} fixed hues — "
              f"hues CYCLE with distinct dash patterns per lap (legend "
              f"disambiguates)", file=sys.stderr)

    # ---- summary table (stdout + CSV) ------------------------------------
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = out_dir / "usage_summary.csv"
    hdr = ("cell", "payload_bytes", "cpu_pct_mean", "pss_mib_mean",
           "pss_unmeasured_reason", "cgroup_mib_mean", "rss_mib_mean",
           "procs_seen", "steady_ticks")
    print(f"{'cell':<40} {'payload':>8} {'cpu%':>7} {'PSS MiB':>10} "
          f"{'cgrp MiB':>10} {'RSS MiB':>8} {'procs':>5}")
    with summary_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        for cell in cells:
            for size in sorted(data[cell]):
                s = data[cell][size]
                fmt = lambda v, d=1: ("UNMEASURED" if v is None  # noqa: E731
                                      else f"{v:.{d}f}")
                print(f"{cell:<40} {human_size(size):>8} "
                      f"{fmt(s['cpu_pct_mean']):>7} "
                      f"{fmt(s['pss_mib_mean']):>10} "
                      f"{fmt(s['cgroup_mib_mean']):>10} "
                      f"{fmt(s['rss_mib_mean']):>8} {s['procs_seen']:>5}")
                if s["pss_unmeasured_reason"]:
                    print(f"{'':<40} {'':>8} pss UNMEASURED: "
                          f"{s['pss_unmeasured_reason']}")
                w.writerow([cell, size,
                            "" if s["cpu_pct_mean"] is None
                            else f"{s['cpu_pct_mean']:.2f}",
                            "" if s["pss_mib_mean"] is None
                            else f"{s['pss_mib_mean']:.2f}",
                            s["pss_unmeasured_reason"] or "",
                            "" if s["cgroup_mib_mean"] is None
                            else f"{s['cgroup_mib_mean']:.2f}",
                            "" if s["rss_mib_mean"] is None
                            else f"{s['rss_mib_mean']:.2f}",
                            s["procs_seen"], s["ticks"]])
    if self_lines:
        print("\nobserver cost (sampler self-accounting):")
        for line in self_lines:
            print(f"  {line}")

    # ---- figures ---------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable — summary CSV written, figures "
              "skipped", file=sys.stderr)
        return 0

    def new_fig(title: str, ylabel: str):
        fig, ax = plt.subplots(figsize=(9, 5.5))
        ax.set_xscale("log", base=2)
        all_sizes = sorted({s for c in data.values() for s in c})
        ax.set_xticks(all_sizes)
        ax.set_xticklabels([human_size(s) for s in all_sizes])
        ax.set_xlabel("payload (bytes)")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11)
        ax.grid(True, which="major", alpha=0.25, linewidth=0.6)
        ax.spines[["top", "right"]].set_visible(False)
        return fig, ax

    def plot_line(ax, pts, i, *, solid: bool, label: str, lw=2.0,
                  alpha=1.0, markersize=5):
        color, dashes = style_for(i)
        kw = dict(color=color, linewidth=lw, alpha=alpha, label=label)
        if solid:
            line, = ax.plot(*zip(*pts), "-o", markersize=markersize, **kw)
            if dashes != (None, None):
                line.set_dashes(dashes)     # hue-cycle lap disambiguator
        else:
            line, = ax.plot(*zip(*pts), "--", **kw)
        return line

    run_label = run_dir.name
    fig, ax = new_fig(f"Per-cell CPU (steady-window mean, summed across the "
                      f"cell's processes) — {run_label}",
                      "CPU (% of one core)")
    for i, cell in enumerate(cells):
        pts = [(s, d["cpu_pct_mean"]) for s, d in sorted(data[cell].items())
               if d["cpu_pct_mean"] is not None]
        if pts:
            plot_line(ax, pts, i, solid=True, label=cell)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out_dir / "usage_cpu.png", dpi=150)
    plt.close(fig)

    fig, ax = new_fig(f"Per-cell memory (steady-window mean, summed across "
                      f"processes) — {run_label}\nPSS solid (accurate for "
                      f"shared/SHM pages); RSS dashed (double-counts shared "
                      f"mappings); cgroup = container total (docker "
                      f"fallback — not comparable to PSS)", "memory (MiB)")
    drew_any = False
    for i, cell in enumerate(cells):
        by_size = sorted(data[cell].items())
        pss = [(s, d["pss_mib_mean"]) for s, d in by_size
               if d["pss_mib_mean"] is not None]
        rss = [(s, d["rss_mib_mean"]) for s, d in by_size
               if d["rss_mib_mean"] is not None]
        cgrp = [(s, d["cgroup_mib_mean"]) for s, d in by_size
                if d["cgroup_mib_mean"] is not None]
        pss_unmeasured = any(d["pss_unmeasured_reason"] for _, d in by_size)
        if pss:
            plot_line(ax, pss, i, solid=True, label=f"{cell} (PSS)")
            drew_any = True
        elif cgrp:
            # Docker whole-tree denial: the cgroup container-total is the
            # correct series — labeled as its own scope, never a PSS.
            plot_line(ax, cgrp, i, solid=True,
                      label=f"{cell} (cgroup total — container scope, "
                            f"not PSS)")
            drew_any = True
        elif pss_unmeasured and rss:
            # Partial permission denial: a partial PSS sum would be a
            # fabricated LOW number — show RSS, loudly labeled, instead.
            plot_line(ax, rss, i, solid=True,
                      label=f"{cell} (RSS — PSS unavailable: permission "
                            f"denial)")
            drew_any = True
            rss = []   # no dashed twin: the solid line IS the RSS
        if rss:
            # The labeled dashed twin (beside a PSS line), or — with no
            # PSS/cgroup/denial at all (e.g. a run too short for a PSS
            # tick) — the only memory series there is.
            plot_line(ax, rss, i, solid=False, lw=1.2, alpha=0.55,
                      label=f"{cell} (RSS)")
            drew_any = True
    if not drew_any:
        print("warn: no memory points at all — usage_mem.png not written",
              file=sys.stderr)
    else:
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=8, frameon=False)
        fig.tight_layout()
        fig.savefig(out_dir / "usage_mem.png", dpi=150)
    plt.close(fig)

    print(f"\nwrote {summary_path}")
    print(f"wrote {out_dir / 'usage_cpu.png'}")
    if drew_any:
        print(f"wrote {out_dir / 'usage_mem.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
