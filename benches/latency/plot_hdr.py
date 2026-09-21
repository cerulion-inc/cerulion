#!/usr/bin/env python3
"""plot_hdr.py: HdrHistogram-style percentile-percentile plot.

Reads a `.bin` raw-sample dump (u64-LE nanoseconds, written by the ROS2
or Cerulion bench binaries when CER_BENCH_RAW_DUMP_DIR is set) and
produces a percentile-vs-latency chart following the HdrHistogram
convention:

  - x-axis: percentile, linear from 0.0 → 0.9 then logarithmic from
            0.9 → 0.99999 (so the long tail expands)
  - y-axis: latency in microseconds, log scale
  - one curve per `(label, .bin)` pair

The plot is the standard tool for reading a latency distribution's
*shape* — flat curves are good (all percentiles within tight bound),
hockey-stick curves at p99+ flag a long tail that the percentile
summaries hide.

Usage:

    plot_hdr.py output.png \\
        "rclcpp callback @ 1MB"=results/raw/humble_cyclonedds_shm_rclcpp_chrt0_1048576.bin \\
        "rcl_loan_recv @ 1MB"=results/raw/humble_cyclonedds_shm_rcl_loan_recv_chrt0_1048576.bin \\
        "intra-naive @ 1MB"=results/raw/humble_cyclonedds_shm_intra_naive_chrt0_1048576.bin \\
        "intra-forward @ 1MB"=results/raw/humble_cyclonedds_shm_intra_forward_chrt0_1048576.bin

Pass any number of `<label>=<path>` pairs. The plot title can be set via
`PLOT_TITLE` env var.

Inspired by HdrHistogram's percentile output:
https://github.com/HdrHistogram/HdrHistogram
"""

from __future__ import annotations

import math
import os
import struct
import sys
from pathlib import Path
from typing import List, Tuple

try:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker
except ImportError as e:
    sys.stderr.write(f"plot_hdr.py: missing dependency — {e}\n")
    sys.stderr.write("install: pip install numpy matplotlib\n")
    sys.exit(2)


# ---- HdrHistogram x-axis transform --------------------------------------
# The HdrHistogram percentile axis: linear from 0 → 90, log thereafter so the
# tail (99, 99.9, 99.99) gets equal visual weight to the first 90%.
# Implementation: x = -log10(1 - p) so p=0.9 → x=1, p=0.99 → x=2,
# p=0.999 → x=3. Below p=0.9 just use p directly scaled to match.

def percentile_to_x(p: float) -> float:
    """Map percentile p ∈ [0, 1) to a chart x-coordinate.

    The combination of the linear segment [0, 0.9] and the log-tail
    segment [0.9, 1) means the chart treats (0 → 0.9), (0.9 → 0.99),
    (0.99 → 0.999), and (0.999 → 0.9999) as equal-width bands."""
    if p < 0.9:
        # 0 → 0 ; 0.9 → 1.0
        return p / 0.9
    # 0.9 → 1.0 ; 0.99 → 2.0 ; 0.999 → 3.0 ; …
    return -math.log10(max(1.0 - p, 1e-12))


# ---- IO -----------------------------------------------------------------

def read_samples_us(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n = len(raw) // 8
    if n == 0:
        return np.array([], dtype=np.uint64)
    arr = np.frombuffer(raw[: n * 8], dtype="<u8").copy()
    arr.sort()
    return arr.astype(np.float64) / 1000.0  # ns → µs


# ---- plot ---------------------------------------------------------------

def plot_curves(curves: List[Tuple[str, np.ndarray]], out_path: Path) -> None:
    if not curves:
        sys.stderr.write("plot_hdr: no curves to plot\n")
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(11, 6.5))

    # Tick stops at standard percentile multiples for readability.
    tick_pcts = [
        0.0, 0.5, 0.9,
        0.99, 0.999, 0.9999, 0.99999
    ]
    tick_labels = [
        "0%", "50%", "90%",
        "99%", "99.9%", "99.99%", "99.999%"
    ]

    for label, sorted_us in curves:
        if sorted_us.size == 0:
            sys.stderr.write(f"plot_hdr: warn — {label} has 0 samples\n")
            continue
        n = sorted_us.size
        # Sample at every percentile mark from 1 / n upward in equally-
        # spaced HdrHistogram space (i.e. equal x-coordinate steps).
        # To keep the curve smooth we generate ~400 points with x evenly
        # spaced from 0 → percentile_to_x(1 - 1/n).
        x_max = percentile_to_x((n - 1) / n)
        xs = np.linspace(0.0, x_max, 400)
        # Invert percentile_to_x to get the percentile back, then index
        # into the sorted array.
        ps = np.array([
            x * 0.9 if x <= 1.0 else 1.0 - 10 ** (-x)
            for x in xs
        ])
        idx = np.clip((ps * (n - 1)).astype(int), 0, n - 1)
        ys = sorted_us[idx]
        ax.plot(xs, ys, label=label, linewidth=1.6)

    # X-axis ticks at standard percentile stops.
    ax.set_xticks([percentile_to_x(p) for p in tick_pcts])
    ax.set_xticklabels(tick_labels, rotation=0)
    ax.set_xlabel("percentile (HdrHistogram convention)")

    ax.set_ylabel("round-trip latency (µs)")
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(mticker.LogFormatter(labelOnlyBase=False))

    ax.grid(True, which="both", alpha=0.3, linestyle="--")
    ax.legend(loc="upper left", fontsize=9, framealpha=0.9)

    title = os.environ.get(
        "PLOT_TITLE",
        "Round-trip latency distribution — percentile-percentile chart"
    )
    ax.set_title(title, fontsize=11)

    note = os.environ.get(
        "PLOT_NOTE",
        "HdrHistogram convention: x-axis linear to p90, log to p99.999.\n"
        "Flat curve = consistent latency. Hockey-stick at p99+ = tail."
    )
    if note:
        ax.text(
            0.02, 0.98, note,
            transform=ax.transAxes,
            fontsize=8,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.85,
                      edgecolor="0.7"),
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close(fig)
    sys.stdout.write(f"wrote {out_path}\n")


# ---- main ---------------------------------------------------------------

def main(argv: List[str]) -> int:
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2

    out_path = Path(argv[0])
    pairs = []
    for spec in argv[1:]:
        if "=" not in spec:
            sys.stderr.write(f"plot_hdr: missing '=' in arg: {spec}\n")
            return 2
        label, path = spec.split("=", 1)
        bin_path = Path(path)
        if not bin_path.is_file():
            sys.stderr.write(f"plot_hdr: not a file: {bin_path}\n")
            return 2
        pairs.append((label, read_samples_us(bin_path)))

    plot_curves(pairs, out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
