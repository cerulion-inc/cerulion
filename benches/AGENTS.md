# benches - agent notes

Standalone benchmark workspaces - NOT root-workspace members; each has its own
`Cargo.toml` + `Cargo.lock` and builds/runs from its own directory. A green build proves
nothing about delivery: a misconfigured bench runs to completion flowing ZERO data.

## Invariants

- Prove delivery per run, never infer it: every latency/throughput leg reports producer
  publish/fire count, consumer received count, and per-input backpressure drop counters,
  so `received == published && dropped == 0` is a measured invariant - not a guess from
  `n ~= window x rate` (worked example: `latency/METHODOLOGY.md` §6).
- Report window arithmetic (setup/drain inside the wall window; Period catch-up ticks)
  separately from loss - a fires-vs-naive-count gap is not drops. Frame policies exactly:
  `drop_oldest` is a declared latest-wins contract with counters; `block` is lossless.
- Report floor (min), p50, p99, and max per row - floors and tails move independently.
- Flatness gates: the per-size floor is a robust LOW percentile (p10 today), never
  the raw min - a lucky-fast sample false-FAILs the ratio. Drop-one-highest
  requires >= 2 large copy-sensitive swept sizes so a real O(n) copy inflates at least two
  floors and survives the drop (a copy isolated to ONLY the largest size is the documented
  residual). Pin both the "a copy must FAIL" property and the >=2-large-sizes config
  check with a test before touching either gate.
- Tune gates by principle + synthetic injection on captured sample distributions (inject a
  lucky-low sample; inject a per-byte copy; confirm the gate flips) - never by tuning on
  one machine; a quiet machine cannot reproduce a noisy shared-CI flake.
- Bench graphs are REAL Cerulion workspaces: one cdylib crate per node under
  `nodes/<type>/`, topology in `graphs/*.yaml`, trigger policy in the macro - scaffolded
  and run through the real `cerulion` CLI. No fake data, ever (root rule).

## Runner conventions

- THE public latency suite: `python3 benches/latency/bench.py` (`native` / `workspace` /
  `ros2` / `compile-csv` / `plots` / `smoke` / `full` / `list-cells`; `--variant
  quiescent|fixed100|backtoback`). Legs: Cerulion workspace (real `cerulion graph run`,
  split + mono), raw iceoryx2 floor, zenoh-SHM, ROS 2 cells. `quiescent` = sensor-rate
  realism (default); `fixed100` = uniform 100 Hz at every size, fallback ladder - THE
  payload-flatness variant; `backtoback` = saturation. `smoke` = regression gate (per-host
  baseline in `expected-ranges.yaml`; see `latency/README.md`).
- The older trees (`cerulion_round_trip{,_quiescent}`, `mp_latency`, `shm_*`, the root
  `bench.py`) are DELETED and their result packages are not published; the packages under
  `docs/benchmarks/results/` are the campaigns the README cites. Build `--release`.

## Gotchas

- iceoryx2 version skew HANGS, it does not error: exact-pin iceoryx2 (`=0.10.0`) in every
  bench workspace - a loose `"0.9"` floats, and a binary<->cdylib skew breaks the SHM
  event protocol ("Unable to establish connection" flood / 0 samples). Rebuild binary +
  cdylibs under ONE lock; give scripts a per-size watchdog + flood guard so a skew
  aborts loudly.
- Run `cerulion graph run` as a SUBPROCESS - host binary and cdylib each carry their own
  statically-linked transport singleton and never rendezvous in-process.
- Hermetic benches share `iceoryx2::testing::generate_isolated_config()` across all
  peers (two iceoryx2 versions collide on the host-global management SHM segment).
- `latency/results/` is deliberately NOT gitignored: a reviewed run's raw `.bin`s, CSVs
  and evidence are committed so every published percentile stays recomputable. Stage a run
  explicitly, NEVER with a tree-wide `git add`; only run logs + the smoke scratch dir are
  ignored (`latency/.gitignore`).

Deep reference: `benches/README.md` + `benches/latency/{METHODOLOGY,PITFALLS}.md` - read
before adding a suite or changing a gate (the deleted trees' methodology lives there).
