# Cerulion benchmark suites

**`benches/latency/` is THE public benchmark suite**. It measures
round-trip latency of Cerulion's zero-copy shared-memory transport
side-by-side with the raw-transport floor (iceoryx2), an alternative SHM
middleware (zenoh), and ROS 2 (CycloneDDS / FastDDS / zenoh RMWs across
humble / jazzy / lyrical). Its `README.md`, `METHODOLOGY.md` and
`PITFALLS.md` carry what it measures, how to reproduce a run, and the
pitfalls that produce wrong numbers.

Start here:

- [`latency/README.md`](latency/README.md): what is measured, the pinned
  line inventory, the topology matrix + canonical pairing table, how to
  reproduce, where results land.
- [`latency/METHODOLOGY.md`](latency/METHODOLOGY.md): pacing modes
  (quiescent = primary, backtoback = saturation bound), schedules, timing
  rules, QoS pins, run-shape declarations, A/B discipline.
- [`latency/PITFALLS.md`](latency/PITFALLS.md): symptom → cause → fix
  catalog of the RMW + operational pitfalls that produce wrong or missing
  numbers.

Quickstart (see `latency/README.md` for the full matrix):

```bash
python3 benches/latency/bench.py list-cells          # what would run
python3 benches/latency/bench.py native --chrt both  # native lines
python3 benches/latency/bench.py full --chrt both --build-image  # everything
python3 benches/latency/bench.py compile-csv && python3 benches/latency/bench.py plots
```

## Directory map

| Directory | Status | What it is |
|---|---|---|
| `latency/` | **ACTIVE: the public suite** | Complete, self-contained latency campaign: native floor + comparison lines, real-CLI workspace lines (`cerulion graph run`, mono + split legs), ROS 2 cell matrix in Docker, driver (`bench.py`), strict offline aggregation, smoke gate, plot system. Raw `.bin` samples → offline percentiles; no number ships without its harness (Principle #13). |

## Conventions (all suites)

- Standalone workspaces (NOT root-workspace members); each has its own
  `Cargo.toml` + `Cargo.lock`. Always build `--release`.
- No fake data, ever (Principle #13): no latency number appears anywhere
  until a real run produces it, and every published number is
  recomputable from committed raw samples + the committed harness + the
  host spec: no number leaves the repo without its harness.
- Prove delivery per run, never infer it; report floor/p50/p99/max.
  Build invariants: the iceoryx2 exact pin `=0.9.1`, subprocess-only
  `graph run`, and isolated SHM configs for hermetic benches.
  [`latency/METHODOLOGY.md`](latency/METHODOLOGY.md) carries the run
  rules those numbers have to satisfy.
