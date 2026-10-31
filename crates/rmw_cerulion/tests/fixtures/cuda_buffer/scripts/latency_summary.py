#!/usr/bin/env python3
"""Summarize s2_lat CSVs: per rmw x mode x size, p50/p90/p99/p99.9/max in
microseconds over the post-warmup samples, with received, lost and invalid
counts and the backend(s) the subscriber saw.

usage: latency_summary.py <latency_dir> <count> <warmup>
Writes summary.tsv and summary.md beside the CSVs.
"""
import csv
import glob
import math
import os
import sys

d, count, warmup = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])


def pct(xs, p):
    if not xs:
        return float("nan")
    k = max(0, min(len(xs) - 1, math.ceil(p / 100.0 * len(xs)) - 1))
    return xs[k]


rows = []
for path in sorted(glob.glob(os.path.join(d, "*__*__*.csv"))):
    rmw, mode, size = os.path.basename(path)[:-4].split("__")
    recs = list(csv.DictReader(open(path)))
    meas = [r for r in recs if int(r["seq"]) >= warmup]
    lat = sorted(int(r["lat_ns"]) / 1000.0 for r in meas)
    rows.append({
        "rmw": rmw, "mode": mode, "size": int(size), "n": len(lat),
        "lost": count - len(meas), "invalid": sum(1 for r in meas if r["ok"] != "1"),
        "backends": "+".join(sorted({r["backend"] for r in meas})) or "-",
        "p50_us": pct(lat, 50), "p90_us": pct(lat, 90), "p99_us": pct(lat, 99),
        "p999_us": pct(lat, 99.9), "max_us": lat[-1] if lat else float("nan"),
    })
rows.sort(key=lambda r: (r["rmw"], r["mode"], r["size"]))
cols = ["rmw", "mode", "size", "n", "lost", "invalid", "backends",
        "p50_us", "p90_us", "p99_us", "p999_us", "max_us"]
with open(os.path.join(d, "summary.tsv"), "w") as f:
    f.write("\t".join(cols) + "\n")
    for r in rows:
        f.write("\t".join(f"{r[c]:.1f}" if isinstance(r[c], float) else str(r[c])
                          for c in cols) + "\n")


def human(n):
    for unit, s in (("MiB", 1 << 20), ("KiB", 1 << 10)):
        if n >= s:
            return f"{n // s} {unit}"
    return f"{n} B"


with open(os.path.join(d, "summary.md"), "w") as f:
    f.write("| rmw | mode | size | n | lost | invalid | backend | p50 µs | p99 µs | p99.9 µs | max µs |\n")
    f.write("|---|---|---:|---:|---:|---:|---|---:|---:|---:|---:|\n")
    for r in rows:
        f.write(f'| {r["rmw"]} | {r["mode"]} | {human(r["size"])} | {r["n"]} | {r["lost"]} | '
                f'{r["invalid"]} | {r["backends"]} | {r["p50_us"]:.1f} | {r["p99_us"]:.1f} | '
                f'{r["p999_us"]:.1f} | {r["max_us"]:.1f} |\n')
print(open(os.path.join(d, "summary.md")).read())
