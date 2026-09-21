# Benchmark evidence

Every number in the root README and in `docs/PERFORMANCE.md` traces to a
package under `results/`. Packages contain the retained per-cell summary CSVs, achieved-rate sidecars,
run manifests (`run.json`: git sha, machine hash, host specifications, governor
state and per-cell timestamps), and a `HEADLINE.md` that renders the cited
tables. Machine identifiers and paths use portable labels.
The native campaigns' `.bin` samples remain on the benchmark hosts. The
rmw_cerulion native harness package keeps its raw `.bin` samples, node logs and
delivery receipts, because its caveats are counted from them.

## Packages

| package | what it is | cited by |
|---|---|---|
| [Native fixed100 campaign](results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/) | the six STOCK hero cells (Cerulion multi-process and single-process, raw iceoryx2, zenoh SHM, ROS 2 defaults, ROS 2 composed) on an x86-64 Linux desktop, k=5, fixed 100 Hz, recorder on | the README latency table and five of the six series of its round-trip chart; `docs/PERFORMANCE.md` |
| [ROS 2 over rmw_cerulion in the native harness](results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/) | ROS 2 Jazzy over `rmw_cerulion` with loaned messages both ways, measured by `benches/latency` itself in the chart's stock posture on the same desktop: ten sizes, k=5, fixed 100 Hz; with a stock ROS 2 parity spot check and the acceptance smoke | the `rmw_cerulion` series of the README round-trip chart and the figures under the README's "ROS 2 over Cerulion"; `docs/PERFORMANCE.md` "ROS 2 over `rmw_cerulion`, in the same harness"; `docs/ros2_compatibility.md` |
| [Jetson Orin NX campaign](results/9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx/) | the two Cerulion rows on an NVIDIA Jetson Orin NX, k=5, fixed 100 Hz, recorder on | the README three-platform chart |
| [Apple M4 campaign](results/0b7bc3994f78e232-2026-09-17-fixed100-apple-m4/) | the two Cerulion rows on an Apple M4 laptop, k=5, fixed 100 Hz, recorder on | the README three-platform chart |

Package directories are named `<machine_hash>-<date>-<campaign>`. The machine
hash is a 16-character sha256 prefix of the host's plaintext identity fields,
computed by `tools/scripts/benchmarks/lib/machine_hash.sh` and recorded with
those fields in each `run.json`, so two packages from one machine share the
prefix.

## Publishing a run

`benches/latency/bench.py` writes a run directory under
`benches/latency/results/` in this layout, and
`tools/scripts/reproduce_benchmarks.sh` drives it from a fresh clone.
Committing a run directory here is a separate, deliberate act: a package lands
with its `HEADLINE.md` and provenance, and the docs that cite it change in the
same change. Nothing here is generated from a plot or edited by hand.

A baseline report (`baselines/`) is optional extra context for a host: the
package's own `run.json` is the identity the docs rely on.
