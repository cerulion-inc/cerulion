#!/usr/bin/env python3
"""Compare real RMW round-trip percentiles in interleaved release runs."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import tempfile

MEASUREMENT = re.compile(
    r"discriminator\[(spin=(?:0|50) unpinned)\] RTT "
    r"p50=([0-9.]+)(ns|µs|us|ms|s) p90=[0-9.]+(?:ns|µs|us|ms|s) "
    r"p99=([0-9.]+)(ns|µs|us|ms|s)"
)
SCALE_US = {"ns": 0.001, "µs": 1, "us": 1, "ms": 1000, "s": 1_000_000}
TEST = "rmw_wait_pingpong_discriminator_test"
FILTER = "ping_pong_counters_with_the_spin_on_and_off"


def measurements(output):
    result = {}
    for match in MEASUREMENT.finditer(output):
        label, p50, unit50, p99, unit99 = match.groups()
        if label in result:
            raise ValueError("duplicate RMW measurement phase")
        values = {"p50_us": float(p50) * SCALE_US[unit50],
                  "p99_us": float(p99) * SCALE_US[unit99]}
        if values["p50_us"] <= 0 or values["p99_us"] < values["p50_us"]:
            raise ValueError("invalid RMW percentile measurement")
        result[label] = values
    if set(result) != {"spin=0 unpinned", "spin=50 unpinned"}:
        raise ValueError("missing RMW measurement phase")
    return result


def command(args, cwd, **kwargs):
    return subprocess.run(args, cwd=cwd, check=True, **kwargs)


def build(checkout, target, destination, logs):
    env = dict(os.environ, CARGO_TARGET_DIR=str(target), CARGO_INCREMENTAL="0")
    with logs.open("w") as log:
        built = command(["cargo", "test", "--locked", "--release", "-p",
                         "rmw_cerulion", "--test", TEST, "--no-run",
                         "--message-format=json"], checkout, env=env,
                        stdout=subprocess.PIPE, stderr=log, text=True)
    executable = None
    for line in built.stdout.splitlines():
        event = json.loads(line)
        if (event.get("reason") == "compiler-artifact"
                and event.get("target", {}).get("name") == TEST
                and event.get("executable")):
            executable = Path(event["executable"])
    if executable is None:
        raise ValueError("RMW release test executable was not produced")
    shutil.copy2(executable, destination)
    return hashlib.sha256(destination.read_bytes()).hexdigest()


def summarize(pairs):
    rows = []
    for phase in ("spin=0 unpinned", "spin=50 unpinned"):
        row = {"path": phase, "size_bytes": 24}
        for percentile in ("p50_us", "p99_us"):
            before = [pair["baseline"][phase][percentile] for pair in pairs]
            after = [pair["candidate"][phase][percentile] for pair in pairs]
            deltas = [(a / b - 1) * 100 for b, a in zip(before, after)]
            row[percentile] = {
                "before_median": statistics.median(before),
                "after_median": statistics.median(after),
                "paired_delta_percent": statistics.median(deltas),
                "paired_delta_min_percent": min(deltas),
                "paired_delta_max_percent": max(deltas),
            }
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = Path(command(["git", "rev-parse", "--show-toplevel"], Path.cwd(),
                        stdout=subprocess.PIPE, text=True).stdout.strip())
    baseline_sha = command(["git", "rev-parse", args.baseline + "^{commit}"],
                           root, stdout=subprocess.PIPE, text=True).stdout.strip()
    candidate_sha = command(["git", "rev-parse", "HEAD"], root,
                            stdout=subprocess.PIPE, text=True).stdout.strip()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="rmw-comparison-") as temporary:
        scratch = Path(temporary)
        baseline = scratch / "baseline"
        command(["git", "clone", "--quiet", "--no-checkout", "--shared",
                 str(root), str(baseline)], root)
        command(["git", "checkout", "--quiet", "--detach", baseline_sha], baseline)
        target = root / "target"
        hashes = {}
        for label, checkout in (("baseline", baseline), ("candidate", root)):
            binary = baseline / "rmw-oracle" if label == "baseline" else scratch / "rmw-candidate"
            hashes[label] = build(checkout, target, binary, output / (label + "-build.log"))
        binaries = {"baseline": baseline / "rmw-oracle",
                    "candidate": scratch / "rmw-candidate"}
        pairs = []
        for index in range(1, 6):
            pair = {}
            for label in ("baseline", "candidate"):
                run = subprocess.run([str(binaries[label]), FILTER, "--exact",
                                      "--nocapture", "--test-threads=1"], cwd=root,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, check=False)
                (output / f"pair-{index:02}-{label}.log").write_text(run.stdout)
                run.check_returncode()
                if "test result: ok. 1 passed;" not in run.stdout:
                    raise ValueError("RMW oracle did not run exactly one passing test")
                pair[label] = measurements(run.stdout)
            pairs.append(pair)
        report = {"baseline_sha": baseline_sha, "candidate_sha": candidate_sha,
                  "binary_sha256": hashes, "pairs": pairs,
                  "summary": summarize(pairs)}
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
