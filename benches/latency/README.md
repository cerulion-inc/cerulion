# Cerulion latency benchmark suite

Round-trip latency of Cerulion's zero-copy shared-memory transport, measured
side-by-side with the raw-transport floor it sits on (iceoryx2), an
alternative SHM middleware (zenoh), and ROS 2 (CycloneDDS / FastDDS / zenoh
RMWs across multiple distros).

> `rmw_cerulion` (ROS 2 running *over* the Cerulion transport) was removed
> from this suite once and re-admitted for one reason: the README round-trip
> chart draws an `rmw_cerulion` line beside the stock and composed ROS 2
> lines, and that line has to come from the same harness. The published cell
> is `jazzy_cerulion_shm_loan_be1_chrt0`, loaned publishing and loaned takes
> (`METHODOLOGY.md` §19 to §21).

This folder is the complete, self-contained campaign: harness code, cell
matrix, methodology, pitfalls catalog, the `bench.py` driver and
post-processing. Published result packages live under
`docs/benchmarks/results/`.

> **Status.** Packages published from this suite are committed under
> `docs/benchmarks/results/` (see `docs/benchmarks/README.md`). The working
> directory `results/` here is empty and `expected-ranges.yaml` ships with
> `hosts: {}`: a host captures its own baseline before the smoke gate means
> anything. This repo's Principle #13 (never fabricate data) applies to
> benchmarks with full force: no latency number appears anywhere in this
> suite until a real run produces it, and every published number is
> recomputable from its committed package.

## Reading order

| File | What it covers |
|---|---|
| `README.md` (this file) | What is measured, the line inventory, how to reproduce, where results land |
| `METHODOLOGY.md` | Pacing modes, schedules, timing rules, QoS pins, run-shape declarations, A/B discipline |
| `PITFALLS.md` | Symptom → cause → fix catalog: 22 RMW + operational pitfalls that produced wrong or missing numbers in prior campaigns |

## What this measures

One **cell** = one (line, payload size, scheduling mode) tuple. Each cell's
binary emits raw per-sample round-trip times as little-endian `u64`
nanoseconds to a `.bin` file; percentiles are computed **offline** by
`compile_csv.py` (never inside the measurement binary), so any re-analysis
(different percentiles, ECDFs, HDR histograms) is one Python call away, not
a re-run.

Three pacing modes cover three different questions (see `METHODOLOGY.md`
§1 + §17 "The rate axis"):

- **quiescent** (default, the sensor-rate realism suite): publish at a
  fixed, payload-appropriate sensor rate (1 kHz IMU-class down to 10 Hz
  dense-frame-class), sleep between iterations. This is what production
  robotics looks like: a robot's topic mix really does tick at these
  rates.
- **fixed100** (uniform-rate): ONE target
  rate, 100 Hz, the field-modal choice for a payload sweep (the ROS 2 `performance_test` school; the Fast DDS vendor's published 1 KB→4 MB campaign),
  at EVERY size, 2 000 measured samples per size. Holding the rate
  constant makes the per-wake idleness tax a constant across sizes, so
  payload-flatness is measurable without the rate axis riding along
  (`METHODOLOGY.md` §17).
  A cell that cannot sustain 100 Hz
  at a size steps down a fallback ladder (100→50→20→sensor floor) and
  re-runs; the ACHIEVED rate is recorded (`achieved_rate_hz` CSV
  column) and plots annotate any non-target point `@NHz`; no
  mixed-rate line is ever silent. A cell that exhausts the ladder
  renders **"did not sustain"** (empty row + loud note + in-figure gap
  explanation), never a fake latency.
- **backtoback** (secondary): publish the instant the previous round-trip
  completes. Bounds the transport at saturation; at large payloads it
  measures queueing + memory bandwidth, not latency.

All modes run the same cell matrix; `CER_BENCH_PACING` (or
`bench.py --variant`) selects between them. Payload sweep: **10 sizes**, namely
64 B, 256 B, 1 KB, 4 KB, 16 KB, 64 KB, 256 KB, 1 MB, 4 MB, 16 MB (exact
powers of two in bytes; the 16 MB label means 16,777,216 B).

## Line inventory

Line names are **pinned**: the `raw_prefix` below is the `.bin` filename
stem, the CSV key, and the plot-line key. Renaming one silently drops the
line from plots (see `PITFALLS.md` § "Plot/prefix coupling").

### Native host lines (Rust, no Docker): `native/`

| raw_prefix | Tag | Binary | chrt | What it is |
|---|---|---|---|---|
| `iox2_chrt0` | **floor** | `raw_iceoryx2_round_trip` | off only | Raw iceoryx2 with zero Cerulion involvement, mirroring iceoryx2's upstream canonical publish-subscribe bench. The hardware/OS floor any iceoryx2-based middleware can hit. Spin-bound, so SCHED_FIFO adds nothing; no chrt-on cell. |
| `zenoh_shm_chrt0` / `zenoh_shm_chrt1` | **comparison** | `zenoh_shm_round_trip` (+ `_pong`, two processes) | off + on | Raw zenoh with SHM engaged at every payload size (`message_size_threshold: 0`: zenoh's shipped default sends small payloads inline; forcing SHM keeps the `shm` label true at every size and mirrors zenoh's own `z_ping_shm`/`z_pong` two-process methodology). The zenoh crate is exact-pinned `=1.7.2`: the SHM config keys + allocator floor are 1.7.2-verified, and bumping it is a deliberate change that re-verifies those keys (`native/Cargo.toml`), disclosed here so nobody discovers the compared stack's version from the lockfile. |

### Workspace lines (the real `cerulion` CLI): `workspace/`

A real Cerulion workspace (per-node cdylib crates, YAML graphs) run via
`cerulion graph run`, exactly as a user deploys. Driven by
`workspace/run_workspace.sh`, which follows the quiescent schedule by
rewriting the ping node's `period_ms` per payload size (`period_ms` is an
integer millisecond, so the 60 Hz and 30 Hz sizes run slightly fast: the
runner prints the effective rate; see `METHODOLOGY.md` § "integer-period
quantization").

| raw_prefix | Tag | Leg | What it is |
|---|---|---|---|
| `cerulion_workspace_split_chrt{0,1}` | **user-facing: THE HEADLINE ROW** | `cerulion graph run rtt_bench_split`, a declared 2-group `process_groups:` graph, **zero flags, zero env overrides** | Real-clock multi-process: g1={ping} / g2={pong, latency}, so the ping → pong edge is a real cross-process iceoryx2 hop under barrier lockstep and the network gateway spawns by default. This is the multi-process pairing against ROS 2's own 3-process shape. **Type class: variable**; these legs ride `sensor_msgs/Image` with the unbounded `data` field loaned per tick (they always have; the axis just names it; see `METHODOLOGY.md` §18). |
| `cerulion_workspace_mono_chrt{0,1}` | **user-facing** | `graph run rtt_bench --single-process`, one flag, nothing else | The single-process opt-in. The gateway still spawns (it is the product's default network posture); the row label IS the invocation. **Type class: variable** (same note as split). |
| `cerulion_workspace_split_pod_chrt{0,1}` | **type-class twin** | `graph run rtt_bench_pod_split`, the same declared 2-group shape | The FIXED-POD twin of the split leg (type-class axis, `METHODOLOGY.md` §18): identical wiring, the message swapped for `PodPayload`, a purely-fixed workspace schema totaling exactly N bytes per sweep point (mirroring ROS 2's `Pod<N>`), rebuilt per size with the baked array length + an init-time size gate. The split-variable vs split-pod delta isolates what a variable field costs the loaned-slot write path. |
| `cerulion_workspace_mono_pod_chrt{0,1}` | **type-class twin** | `graph run rtt_bench_pod --single-process` | The FIXED-POD twin of the mono leg (same rules). |

> **Where the flagless default is:** the retired `default` leg (`cerulion graph run
> rtt_bench`, zero flags, where the runtime derives a process-per-node partition)
> reads much slower than the declared split on the bench machine, because the derived
> 3-worker shape parks a worker on an un-ringable doorbell.
> The declared 2-group `split` leg
> is the representative multi-process number, so it is the
> headline row.
> `run_workspace.sh` refuses a
> requested `default` leg loudly rather than minting either a misleading
> headline or a duplicate mp row.

The row label is the invocation; a leg never inherits hidden shape flags
or env. See "Run shape and network posture" below.

### ROS 2 cells (Docker, one container per cell): `ros2/`

`raw_prefix = {distro}_{rmw}_{shmmode}[_{msg}]_{recv}_{qos}_chrt{N}`

(the `msg` token appears only on non-pod cells; the incumbent Pod<N>
cell names are pinned and unchanged)

| Axis | Values | Notes |
|---|---|---|
| `distro` | `humble`, `jazzy`, `lyrical` | one Docker image per distro (`ARG ROS_DISTRO`). The prior campaign's `kilted` is dropped; `lyrical` is new and stays marked unverified until a sweep has exercised it. |
| `rmw` | `cyclonedds`, `fastdds`, `zenoh`, `cerulion` | The first three are the stock **comparison** lines. `cerulion` is `rmw_cerulion`, ROS 2 over the Cerulion transport, re-admitted so the README round-trip chart can draw its line from this harness. It is `shm` only: its one data path is iceoryx2 shared memory, so `no_shm` is a structural skip. The published cell is `jazzy_cerulion_shm_loan_be1_chrt0` (`METHODOLOGY.md` §19 to §21). |
| `shmmode` | `shm`, `zc`, `no_shm` | `shm` everywhere. `zc` (**fastdds only**, same distro gating as `shm`) = the vendor-recommended DataSharing zero-copy lane: the shm profile plus publisher/subscriber default profiles carrying `<data_sharing><kind>AUTOMATIC</kind></data_sharing>`, run under `RMW_FASTRTPS_USE_QOS_FROM_XML=1`, the rmw_fastrtps README's own recipe. The plain fastdds `shm` cells are **stock defaults** (rmw_fastrtps forces DataSharing off without that env var; `METHODOLOGY.md` §15); publishing both, labeled, is what closes the "you benched Fast DDS with its zero-copy off" attack. `no_shm` (UDP) only on **jazzy**; it exists to show what SHM buys, one distro is enough. |
| `recv` | `rclcpp`, `loan` | `rclcpp` = standard subscription callback (what every ROS 2 app uses; pays a payload memcpy per receive, upstream's shipped default, since rcl disables subscription-side loaned dispatch unless `ROS_DISABLE_LOANED_MESSAGES=0`; rclcpp#2335 / rcl#1110). `loan` = the `rcl_take_loaned_message` lane (the C-layer zero-copy receive); it exports `ROS_DISABLE_LOANED_MESSAGES=0` so the capability probe and the published `loaned=` lines describe the lane that actually ran, while the `rclcpp` lane leaves the env untouched (`METHODOLOGY.md` §15). Structural skip (rc=77, SKIP): `zenoh × loan` (a NO-OP stub upstream). FastDDS loan keeps the runtime capability probe (see `PITFALLS.md` #11). |
| `msg` | *(none = pod)*, `image` | **Type-class axis** (`METHODOLOGY.md` §18): the incumbent cells are fixed-size `Pod<N>` (pod class: no token, names pinned). `image` = `sensor_msgs/msg/Image`, the real unbounded sensor type, `data` resized to the sweep point at prealloc; enumerated on matrix rmws × `shm` × `rclcpp` × `be1` only. Structural skips (the hypothesis as inventory): `image × loan` (unbounded types cannot loan: `can_loan_messages` gates on `is_plain`), `image × zc` (DataSharing requires plain bounded types), lanes stay pod-only. The `is_plain` smoke gate inverts per class: each class fails when its label lies, and either class fails when its marker is ABSENT on a run that otherwise succeeded; every node logs it from its constructor, so a log without it lost it and the class claim is unbacked. |
| `qos` | `be1`, `rel10` | `be1` = BEST_EFFORT / VOLATILE / KEEP_LAST(1), the default, and the prior-campaign pin: the eligibility-friendliest triple for the RMWs' documented SHM / zero-copy paths (NOT because reliability gates loans: it never did; the corrected upstream record is `METHODOLOGY.md` §8/§15). `rel10` = RELIABLE / VOLATILE / KEEP_LAST(10), the rmw-harness pin; enumerated only on jazzy × `shm` × {cyclonedds, fastdds} × rclcpp. **SHM-only**: the pin was an SHM head-to-head, so a `no_shm` rel10 cell answers no pinned question. See `METHODOLOGY.md` § "The two QoS pins". |
| `chrt` | `0`, `1` | `1` = SCHED_FIFO priority 80 (`chrt -f 80`) for every process in the cell. |

### ROS 2 usage-pattern lanes: `ros2/`, same driver

Two additional lanes from the usage evidence in
`ros2/memo.md` (upstream PR/doc reads + GitHub-wide code
search, quoted there with links). They are **pseudo-rmw lanes, never
crossed with the matrix axes**: both definitionally run `recv=rclcpp`
and `qos=stock` (`rmw_qos_profile_default` = RELIABLE / VOLATILE /
KEEP_LAST(10), numerically rel10-equivalent, labeled `stock` so the
cell name says "the default was requested", not "a pin matched it");
`run_bench.sh` hard-errors on any cross-labeled combination:

| Lane | Cell name | What it is |
|---|---|---|
| **stock** (slowest-common) | `{distro}_stock_rclcpp_chrt{N}` | The **zero-config default**, what `ros2 run` gives you (memo §3): `rmw_fastrtps_cpp` (the default rmw), **no** profiles XML, **no** transport env (Fast DDS's default UDPv4 + builtin copy-based SHM transport; DataSharing OFF at the rmw layer), plain `publish()` + typed-callback receive, 3 separate processes. **No `verify_shm` gate**: the lane claims no transport-engagement label (its claim is "the defaults, whatever they do"), so each cell gets a provenance note (`_logs/<cell>_provenance.txt`: "stock (fastdds defaults: UDP+builtin SHM transport, datasharing off)", citing the memo) instead of an SHM gate. |
| **composed** (fastest-common) | `{distro}_composed_ipc{on,off}_rclcpp_chrt{N}` | The **composition pattern** (memo §2): Nav2 ships composed bringup by default since Humble (PR #2750); rclcpp intra-process comms is the opt-in shade (off by default even composed; Nav2 gained the option in Kilted→Lyrical, PR #5804), so BOTH shades are cells. One process, three manually-composed nodes on one `SingleThreadedExecutor` (`composed_rtt_node`), `CER_BENCH_IPC={on,off}` → `use_intra_process_comms`. Publish gesture is the pattern's own: per-iteration `unique_ptr` publish (the documented 0-copy intra-process gesture) with rosidl `MessageInitialization::SKIP`, so the O(N) payload zero-fill stays out of the timed window (Mode-A intent) while the per-message allocation (a structural cost of the intra-process ownership model) stays in, deliberately (named in the binary's header). Same provenance-note treatment as stock (with ipc=on the data path bypasses the rmw entirely; there is no transport label to verify). |

**Distro coverage:** `jazzy` is first-class; `humble`/`lyrical` lane
cells are enumerated but **loudly unverified** until a sweep runs them
(`bench.py ros2` prints an `! UNVERIFIED lane cell` warning per cell).

**Not in the matrix:** the `kilted` distro. The rclcpp intra-process
axis is covered by the `composed` lane above, as manual composition with
both IPC shades (memo-grounded, `ros2/memo.md` §2).

## Topology of each line (read before comparing any two rows)

All lines measure **2 timed message hops** (ROS 2 cells add one
*untimed* `/kick` hop for pacing), but what sits INSIDE the timed
window (address-space crossings and wakes) differs per line, and at
small payloads that row, not the transport, is the first-order term.
The matrix below is the normalization key; the pairing table in
"Boundary discipline" says which rows may face each other.

| Line | Processes on the data path | In-window crossings | In-window wakes | Receive discipline | Timing boundary |
|---|---|---|---|---|---|
| `iox2_chrt0` (floor) | 1 (two threads) | 0 process / 2 thread | 0: spin-poll both sides | spin | harness bracket |
| `zenoh_shm_*` | 2 | 2 | 2 (blocking `recv()` + RX callback) | block | harness bracket |
| `cerulion_workspace_split_*` | 2 workers (+ supervisor & gateway, off-path) | 1 | 1 cross-process data wake (ping → pong); the pong → latency echo hop is in-process in g2, gated by the level-boundary barrier rendezvous | block (WaitSet/park) | embedded stamp |
| `cerulion_workspace_mono_*` | 1 (+ gateway, off-path) | 0 | 0: all three nodes fire in one step on one thread | in-step dispatch | embedded stamp |
| ROS 2 `*_rclcpp_*` | 3 nodes (+ iox-roudi / rmw_zenohd where applicable, off-path) | 2 | 2 executor wakes (+ per-receive heap alloc + payload memcpy) | block (executor) | embedded stamp |
| ROS 2 `*_loan_*` | 3 nodes | 2 | 2 WaitSet wakes (no receive memcpy on pong/latency) | block (WaitSet) | embedded stamp |
| ROS 2 `*_stock_rclcpp_*` | 3 nodes | 2 | 2 executor wakes (+ per-receive heap alloc + payload memcpy); the `*_rclcpp_*` row at zero config: default transports, `stock` QoS | block (executor) | embedded stamp |
| ROS 2 `*_composed_ipc{on,off}_*` | 1 (all three nodes on one executor) | 0 | 0 process wakes: same-thread executor dispatch (+ per-publish heap alloc, the intra-process ownership model; ipcoff additionally round-trips the rmw in-process with its receive alloc+memcpy) | in-executor dispatch | embedded stamp |

Three facts the matrix encodes:

- **Pacing is untimed uniformly.** Every line takes its send stamp
  AFTER the initiator's wake (harness bracket opens after the pacer
  sleep; ROS 2 ping stamps inside the kick callback after the executor
  woke it; workspace ping stamps inside its period tick), so initiator
  wake cost is excluded on every line, a genuine cross-family parity
  property, stated so it is visible instead of discoverable.
- **The wake-cost mechanism differs by line.** Blocking-receive lines
  pay a real wake/context-switch inside the window; the spin-bound
  floor instead carries post-sleep cold microarchitectural state
  (`METHODOLOGY.md` §1). Quiescent-vs-backtoback deltas must be read
  per line, not as one mechanism.
- **Tails are owned by each line's wake mechanism.** A workspace
  `split` max is barrier/park-wake territory; a ROS 2 max is
  DDS-listener/executor territory; an `iox2` max is scheduler
  preemption of a spin. A cross-family MAX or p99 comparison must name
  the mechanism it is comparing; the `.bin`s preserve chronological
  order precisely so an outlier can be located in time and attributed
  before it is quoted (`METHODOLOGY.md` §5).

Containerization is a further named axis: every ROS 2 cell runs in
Docker on the host kernel while native/workspace lines run bare-host:
in-container head-to-heads are environment-symmetric, cross-boundary
comparisons are bounded and direction-labeled in `METHODOLOGY.md` §14.

## Boundary discipline (read before quoting any number)

Three rules
govern every number this suite ever produces:

1. **No number leaves the repo without its harness.** A benchmark claim
   requires the committed script, the raw samples, and the host spec.
   This directory is that harness; the committed
   `results/<machine-hash>-<date>-<variant>/` raw `.bin`s + CSVs plus
   the run manifest (`run.json`: git sha, machine hash with its
   plaintext input fields, governor/turbo state, per-cell timestamps,
   skip inventory, docker image IDs) are that evidence. `run.json` IS
   the machine-hash-keyed host record this rule cites.
2. **Native numbers and rmw numbers never appear in the same sentence
   without the boundary being named.** rmw_cerulion numbers (from this suite's `cerulion` cells) are
   *ROS 2 over the Cerulion transport*: rclcpp, rosidl typesupport, and
   the rmw C ABI are all in that measured path. They are not
   Cerulion-native numbers and must never be presented as the native
   zero-copy story. The native story is the `cerulion_workspace_*`
   lines.
3. **Every cross-family quote routes through the canonical pairing
   table.** Small-payload rankings are decided by the in-window
   crossing/wake rows of the topology matrix above before they are
   decided by any transport, so which rows may face each other is part
   of the method, not editorial choice:

| Claim you want to make | The pair | Topology status |
|---|---|---|
| "ROS 2 over Cerulion vs stock ROS 2" | `{distro}_cerulion_shm_loan_*` vs `{distro}_stock_rclcpp_*`. This suite carries the `cerulion` cells for one purpose, the `rmw_cerulion` line of the README round-trip chart (`jazzy_cerulion_shm_loan_be1_chrt0`) | Same harness, posture and pacing, three processes on both sides. The `cerulion` cell publishes and takes loaned messages and the stock cell does not, so quote the loan posture with the pair. It is ROS 2 over the Cerulion transport, never a native number (rule 2). |
| "Cerulion multi-process vs ROS 2 default" | `cerulion_workspace_split_*` vs `{distro}_stock_rclcpp_*` (default-vs-default, memo §5: the stock lane IS the unconfigured `ros2 run` experience; the per-rmw `{distro}_{rmw}_shm_rclcpp_*` cells are the configured variants of the same pairing) | **Near-matched, asymmetry named** (2 worker processes / 1 in-window crossing vs 3 processes / 2 in-window crossings): the split's declared g2={pong, latency} keeps the echo hop in-process, so quote the crossing counts with the pair. (The flagless default, 3 workers, 2 crossings, the exact crossing-match, returns as this pairing when the park-wake fix lands; see the flagless-default note above.) Executor models also differ (barrier-lockstep level executor vs 3× rclcpp spin), a real product difference, named not hidden. |
| "What a process boundary costs Cerulion" | `split` vs `mono` | Within-family A/B: the split-vs-mono delta is exactly ONE explicit cross-process hop (the declared 2-group partition vs `--single-process` on the same 3-node chain). |
| "Cerulion collapsed shape vs ROS 2" | `mono` vs `{distro}_composed_ipc{on,off}_rclcpp_*` | **Matched** (1 process vs 1 process, 0 in-window crossings on both): ROS 2's collapsed twin is the `composed` lane (memo §2: composition + the intra-process opt-in). Named asymmetry: with ipc=on the ROS 2 side bypasses its middleware entirely (rclcpp pointer-pass with N−1-copy fan-out cliffs), so this row compares *framework* overhead, not transport; `mono` keeps Cerulion's full SHM transport + observability plane in the path. `mono` vs any OTHER ROS 2 cell stays NOT matched (0 vs 2 crossings), citable only with the crossing row quoted. |
| "… vs the hardware floor" | `iox2_chrt0` vs anything | **Floor reference only**: spin-bound, 0 wakes; never a fairness pair for any wake-driven line. |
| "zenoh-native vs Cerulion-native" | `zenoh_shm_*` vs `cerulion_workspace_split_*` | Data-path process count matched (2 vs 2 workers); crossings differ (2 vs 1; the split's echo hop is in-process) and receive models differ (blocking recv vs barrier-lockstep workers); quote the crossing counts. Comparison lines replicate their upstream canonical harness shape (`z_ping_shm`); that rule, stated once, covers the residual. |

Corollary (same doc): "deterministic" is reserved for replay claims;
per-run tail behavior is "jitter", quantified, never "essentially
deterministic".

## Run shape and network posture

Cerulion's product defaults changed since the prior campaign, and both
defaults would silently change what a bench measures, so every workspace
leg **declares** them instead of inheriting them:

- **Multi-process by default.** An unpartitioned graph on Unix
  under the real clock derives a process partition and runs multi-process.
  The `split` leg measures the multi-process shape through a DECLARED
  2-group `process_groups:` graph (`rtt_bench_split`, zero flags) rather
  than the derived flagless partition, because the derived shape
  currently parks a worker on an un-ringable doorbell (the park-wake
  bug) and reads much slower: see the flagless-default note in the workspace
  table. The flagless `default` leg is retired (refused loudly by the
  runner) until the park-wake fix lands. The `mono` leg passes `--single-process`
  (exactly one flag) to measure the single-process opt-in.
- **Permissive networking by default.** Every real-clock live
  run spawns a network gateway process (the robot's zenoh session). The
  gateway is a separate process that parks at zero demand and is not part
  of the measured SHM chain, and it is what every flagless user runs
  with, so NEITHER leg suppresses it (the
  default row omits ALL tags, and mono differs from it by exactly
  `--single-process`). Suppressing the gateway would measure a shape no
  default user gets.

The `.bin`/CSV/plot labels carry the leg name, so the run shape is never
ambiguous in results.

One host-tuning knob is declared rather than implied: `CER_BENCH_DMA_LOCK`
(default `1`) is the suite-wide C-state posture, honored uniformly by all
three stacks. `CER_BENCH_DMA_LOCK=0` is the **stock** posture, what a
flagless user on an untuned host gets: the workspace runner UNSETS
`CERULION_CPU_DMA_LOCK` for the graph (`env -u`, so an inherited value is
removed too, not merely not-added; bench.py exports one on every leg),
the native bins skip the C-state lock, and bench.py runs ros2 containers
WITHOUT the `/dev/cpu_dma_latency` device. Announced loudly at start
("C-state exits unmanaged, tails not comparable to capped runs"),
recorded as `dma_lock_posture` in the run manifest, and a contradictory
ambient `CERULION_CPU_DMA_LOCK` export is refused outright. Stock rows and
capped rows must never be mixed in one comparison without saying so
(METHODOLOGY env table).

A second opt-in knob, `CER_BENCH_USAGE` (default `0`), records per-cell
**CPU + memory usage sidecars** (`<cell>_<size>.usage.csv` beside each
`.bin`; `usage_sampler.py`, Linux-only host-side `/proc` sampling:
both PSS and RSS are recorded because RSS double-counts SHM mappings
across a cell's processes; METHODOLOGY § "Usage sidecars"). Under it
the native sweep runs one invocation per payload size (for per-size
attribution; the `.bin` contract and gates are unchanged), so usage
runs are labeled companion runs, not the headline latency campaign.
Render with `python3 plot_usage.py --run-dir results/<run>/`. The
per-size native wiring (invocation split, sidecar naming, stale-sidecar
guard, exact-set gate) is runtime-exercised locally by
`check_percentile_parity.py` against stub binaries, and both
`usage_sampler.py --self-test` and `plot_usage.py --self-test` pin the
sampler and the PSS labeling gate; the REAL native + docker sampling
paths are exercised only by a usage campaign on a Linux host
(the workspace path has run live on a Jetson).

## Reproducing the campaign

Everything goes through one driver script: `bench.py`. Subcommands:
`native`, `workspace`, `ros2`, `compile-csv`, `plots`, `smoke`, `full`,
`list-cells` (run `list-cells` first; it prints the exact cell matrix
and skip inventory this host would run). Every subcommand accepts
`--variant {quiescent, backtoback}` (default `quiescent`), which maps
onto `CER_BENCH_PACING`: one tree, two pacing modes, no
sibling-directory switching. Run `python3 bench.py <subcommand> --help`
for the authoritative flag surface.

### Prerequisites

- Rust toolchain (stable): native + workspace lines build from this repo.
- Python 3 (`bench.py` uses only the stdlib; `matplotlib` for `plots`).
- Docker: ROS 2 cells only.
- Linux: for chrt-on cells and `no_shm` cells. Everything else is
  designed to run on macOS too (with those cells skipped and a printed
  skip inventory).
- **A citable sweep requires the `performance` CPU governor and an
  otherwise-idle machine** (no other tenants, no concurrent sweeps). The
  DMA lock pins C-states only: frequency scaling is a separate axis;
  the runners print the active governor + turbo/boost state and record
  them in `run.json`, and a governor change re-keys the machine hash.
  macOS exposes no governor, so a macOS sweep carries no governor
  receipt and is never mixed into the cross-stack comparison; it is
  published as its own platform package instead (`METHODOLOGY.md` §7).
- `ulimit -n 65536` before workspace runs (the runner sets it; see
  `PITFALLS.md` #14 for why the default fd cap produces a misleading
  `ServiceInCorruptedState`).
- For `no_shm` (UDP) cells at large payloads, host-global socket buffer
  ceilings (kernel-global; not settable per container):

  ```bash
  sudo sysctl -w net.core.rmem_max=33554432 net.core.wmem_max=33554432
  ```

- Optional but recommended: `/dev/cpu_dma_latency` writable by the bench
  user (the DMA latency lock; without it, tail percentiles include
  C-state-exit jitter, and the runner warns) and `rtprio` in
  `/etc/security/limits.conf` so chrt-on cells don't need sudo.

### The full sweep

```bash
cd benches/latency

# 1. Native host lines (iceoryx2 floor + zenoh comparison)
python3 bench.py native --chrt both

# 2. Workspace lines (real `cerulion graph run`; split + mono legs;
#    split is the headline row)
python3 bench.py workspace --chrt both

# 3. ROS 2 cells (builds the per-distro images, tagged latency_bench:{distro},
#    on first run; hours, unattended)
python3 bench.py ros2 --chrt both --build-image

# 4. Post-process: raw .bin → percentile CSVs → plots
#    (--chrt both here too: the plots default is chrt 0, and a
#    `--chrt both` sweep post-processed without it silently drops
#    every chrt1 series from the figures; a non-quiescent sweep needs
#    the same --variant here; the plots enumerate the workspace legs
#    that variant can carry, e.g. no split leg under backtoback, and
#    plot.py refuses a --variant that contradicts the run dir's run.json)
python3 bench.py compile-csv
python3 bench.py plots --chrt both

# Or end-to-end in one call:
python3 bench.py full --chrt both --build-image
```

**A citable campaign runs reps, not one sweep** (`METHODOLOGY.md` §10:
between-run wobble dominates within-run sampling error, so close
comparisons need k ≥ 5 reps, the median-of-reps as the headline, and
the rep spread as the error bar):

```bash
# k reps of the WHOLE matrix, round-robin (rep-granularity
# interleaving, the §10 same-window mechanism); each rep lands in its
# own rep<k>/ subdirectory and never overwrites a previous one:
python3 bench.py full --chrt both --build-image --reps 5
```

Other pacing variants of any step: add `--variant fixed100` (uniform
100 Hz + the fallback ladder) or `--variant backtoback` (saturation).
The variant is part of the run-dir name, and of the run.json manifest,
which refuses a cross-variant `--run-dir` reuse, so artifacts from
different pacing modes can never silently mix.

Individual legs / cells can also be driven directly, but a **direct
`bash workspace/run_workspace.sh <leg>` or `bash ros2/run_bench.sh`
invocation is NOT a supported entry point**.
It is a diagnostic gesture, not a way to produce numbers. `bench.py` owns
the policies neither script can enforce for itself: the `CERULION`
provenance and freshness refusal (below), the run manifest and pacing
variant that make artifacts comparable, and the **one container per
cell** shape (never `--pid host`) that confines `run_bench.sh`'s blanket
bench-binary backstop. Those live in one place deliberately; a second copy
of them in shell would be a second thing to keep true. Each script says so
at the top, refuses what it *can* see when run directly, and says where it
cannot rather than substituting a weaker rule: `run_bench.sh` asserts the
PID-namespace property before using that backstop and declines it, loudly
and by name, when the property is unproven.


```bash
# Workspace legs without the orchestrator (leg = split | mono). The runner
# REFUSES to start without a raw output directory and the leg's raw
# prefix (bench.py normally sets both), so pass them here:
mkdir -p /tmp/cerbench-raw
CER_BENCH_RAW_DUMP_DIR=/tmp/cerbench-raw \
CER_BENCH_RAW_NAME=cerulion_workspace_split_chrt0 \
    bash workspace/run_workspace.sh split   # THE headline row
CER_BENCH_RAW_DUMP_DIR=/tmp/cerbench-raw \
CER_BENCH_RAW_NAME=cerulion_workspace_mono_chrt0 \
    bash workspace/run_workspace.sh mono
# (raw prefix = cerulion_workspace_<leg>_chrt<0|1>; use chrt1 with
#  CER_BENCH_CHRT=1, which is what the chrt-on cells are named)
# Type-class twin (METHODOLOGY §18; default CER_BENCH_MSG=variable =
# the incumbent Image legs). The pod class carries its own token in the
# raw prefix (cerulion_workspace_<leg>_pod_chrt<0|1>), so its rows
# never land under an Image leg's name:
CER_BENCH_MSG=pod \
CER_BENCH_RAW_DUMP_DIR=/tmp/cerbench-raw \
CER_BENCH_RAW_NAME=cerulion_workspace_split_pod_chrt0 \
    bash workspace/run_workspace.sh split

# One ROS 2 cell inside its container (per-cell driver):
#   see ros2/run_bench.sh; bench.py invokes it per cell with the
#   CER_BENCH_* env contract documented in METHODOLOGY.md
```

`bench.py` enumerates the exact cell matrix above and **prints a
skip inventory**: every cell it does not run appears as a one-line skip
with its reason (non-Linux chrt, the structural `zenoh × loan`
take-loan-stub skip, `no_shm` outside jazzy, etc.), so any run leaves a
complete record of what was and wasn't measured.

### The smoke gate

A low-N sanity pass over a curated cell subset, gated against per-host
p50 baselines in `expected-ranges.yaml` (keyed by machine hash):

```bash
python3 bench.py smoke                        # gate against this host's baseline
python3 bench.py smoke --capture-baseline     # once per machine per variant
```

`expected-ranges.yaml` ships with `hosts: {}`; there are **no** committed
baselines until a real machine captures one. Capture runs its cells three
times and records `[p50/2, p50*2]` bounds around the **median** of the
three measured p50s under that host's machine hash (a lucky single rep
cannot set the bounds); entries are written only by `--capture-baseline`,
never by hand (Principle #13). The gate refuses to run when ambient `CER_BENCH_*`
overrides are present in the environment (they would silently change what
the baseline means).

> **Baselines from before the smoke-set and schedule changes need recapture**: the curated
> smoke set dropped its rmw_cerulion cell and
> the 4/16 MiB quiescent schedule was extended (tail-resolved counts), so a
> pre-change baseline gates a different subset under different counts.
>
> **Baselines captured before the type-class axis** lack its additional smoke cells
> (`METHODOLOGY.md` §18): the smoke set gained one representative per
> stack: `cerulion_workspace_split_pod_chrt0` (both payloads) and
> `jazzy_cyclonedds_shm_image_rclcpp_be1_chrt0` (64 B). An older baseline
> reports them as loud `[skip] … no range in baseline` lines (never a
> silent pass) until `--capture-baseline` is re-run on that host.

### The `CERULION` override

`workspace/run_workspace.sh` reads `CERULION` and runs whatever it names,
defaulting to `<repo>/target/release/cerulion`. It rebuilds that DEFAULT
path before running, so the rebuild protects the default and nothing
else, and an exported override survives it untouched. A campaign measured
through a binary built from other code, while the run manifest records
this checkout's SHA, is a mislabeled result rather than a failed one, so
`bench.py` checks the override before any cell runs (`workspace`, `full`
and `smoke`, and again in the leg itself, ahead of the `cleanup_iceoryx()`
sweep that would otherwise destroy SHM state shared with every other
tenant on the way to refusing).

Two questions are asked, in this order. **Provenance:** was this binary
built from THIS checkout? The path answers it for the default
`target/`, but not under a `CARGO_TARGET_DIR`, which is shared across
checkouts by design, so there the binary's cargo depfile decides.
**Freshness:** is it newer than the source that can change it? There is no
build id to compare (`cerulion --version` prints the crate version on every
build of every commit), so the answer is the newest source mtime over
`cerulion_cli`'s workspace-member closure, derived from the manifests,
not from cargo. Members the CLI does not link (the viz tree, `rmw_cerulion`,
the test fixtures) do not move the floor, so editing one no longer declines
a binary it cannot affect. When that closure cannot be derived the walk
widens back to the WHOLE workspace, which is stricter, and a `[note]` on
stderr says why.

A `CARGO_TARGET_DIR` set to the **empty string** is refused outright, the
way cargo refuses it (`cargo build`, `cargo check` and `cargo metadata`
all exit 101 with *the target directory is set to an empty string in the
`CARGO_TARGET_DIR` environment variable*), because nothing in the checkout
can be built or enumerated under that spelling, so no binary sitting beside
it can be vouched for. A **whitespace-only** value is a different case and
is not refused: cargo builds into a directory of that name, and so does
this. (Note the contrast with `CERULION` in the table below, where a
whitespace-only value *is* refused. The two are not inconsistent; they
are refused by different consumers for different reasons: the shell's
`${CERULION:-…}` does not substitute for a value that is non-null but
blank, so the runner would take the spaces literally and abort on its own
`[ ! -x ]` test, whereas cargo accepts `' '` as an ordinary directory
name. Each guard matches the tool that actually reads the variable.)

| `CERULION` is… | outcome |
|---|---|
| unset, or **empty** | ignored: `${CERULION:-…}` substitutes the default for a null value, so an empty export is the same ask as none |
| this checkout's own build, newer than its source | **runs** |
| the runner's **default path** (`<repo>/target/release/cerulion`), however stale, or not built yet | **runs**: that is the file the runner rebuilds, so naming it explicitly is the same ask as not setting it |
| **undatable** (no `git`, a sparse checkout, a source dated in the future) | **runs**, with a loud `proceeding UNVERIFIED` warning on stderr; refusing would block a campaign over a transient failure |
| **stale**, older than the newest source in `cerulion_cli`'s member closure | **refused** (exit 1), naming how far behind |
| **foreign**, built from another tree (its depfile says so, or it is outside every target dir this checkout writes) | **refused** (exit 1) |
| **whitespace-only** | **refused**: the shell does *not* treat it as unset, so the runner would take the spaces literally and abort on `[ ! -x ]` after its rebuild |
| a **relative** path | **refused**: this process and the runner do not share a working directory, so it could name two different files |
| a path ending in a **separator**, not a file, or not **executable** | **refused** |
| under `workspace/target/debug` | **refused**: the runner `rm -rf`s that directory (its freshest-wins guard) before running `$CERULION` |

Every refusal names the remedy: `unset CERULION`, or
`cargo build -p cerulion_cli --release`. A **direct** `bash
workspace/run_workspace.sh <leg>` invocation does not go through
`bench.py` and is not covered, and is **not a supported entry point**
(see above): the policy is not duplicated in shell, it is declined there.

Smoke exit codes: **0** = every gated cell's p50 in range; **1** = an
env-contract violation refused before any cell ran (a contradictory
`CER_BENCH_PACING` or `CER_BENCH_DMA_LOCK` export, a `CERULION` override
this checkout's source does not describe (see **The `CERULION` override**
below), an **empty `CARGO_TARGET_DIR`**, which cargo itself refuses so
nothing here can be built or dated under it, or a malformed
`CER_BENCH_PAYLOAD_SIZES`. Python's own code for an uncaught
`SystemExit(<message>)`, which is why it is not in the deliberate 2/3/4
series. What runs ahead of the baseline lookup is the **validation** of those
three (parsing only). Their *consequences* (the posture banner, the
deliverable refusal, the curated-subset refusal) sit below the no-baseline
classification, because they gate the quality of a measurement that a
no-baseline run never takes. That split makes three cases distinct, and a
malformed value is answered the same way on every path:

| | no baseline on file | baseline on file |
|---|---|---|
| `CER_BENCH_PAYLOAD_SIZES=notanint` (malformed *ask*) | **1** | **1** |
| `CER_BENCH_PAYLOAD_SIZES=64` (well-formed, restrictive) | **4** | **3** |

Getting that wrong is not cosmetic: **4** is the code
`tools/scripts/run_benchmarks.sh` deliberately *swallows* ("no baseline, skip the
gate, keep going"), so a typo in an exported variable answered with 4 would
silently continue a run instead of aborting it. Deliberately NOT exit 1:
`CER_BENCH_USAGE` and `CER_BENCH_NATIVE_TIMEOUT_S`, which are read per-cell
inside `_run_smoke_cells` and become **3** via its `except SystemExit`; on
the no-baseline path they are never consulted at all, so the run is a 4);
**2** = at least one `FAIL_HIGH` /
`FAIL_LOW`; **3** = crash / build / setup failure, or a cell that RAN with no
range in the baseline; **4** = this host has no baseline for this variant, so
there is nothing to gate against. Four shapes reach it: `expected-ranges.yaml`
absent, `hosts: {}`, a file holding only OTHER hosts' entries, or this host's
own entry with no ranges for this variant. The third is the file's designed
steady state (its header: "Multiple hosts coexist under `hosts:`"), so it is 4
and not a failure. **They share a code but not a remedy, so the gate says
which one it hit**: only the third can be identity drift (it is the only
shape where this host's key is absent), so only it warns that a machine which
captured before may have drifted, and prints the live governor/turbo posture
with it, `machine_hash` covering kernel version, cpu governor and PREEMPT_RT.
The fourth is a partial or hand-edited capture and says so instead, since the
hash matched and drift is excluded. 4 is separate from 3 on
purpose: a baseline is captured ON the machine, so a fresh clone cannot have
one, and `tools/scripts/run_benchmarks.sh` skips the smoke gate on 4 (loudly)
while still stopping on 2 or 3. (The per-cell ROS 2 driver has its own
exit-code contract; see `METHODOLOGY.md` § "Exit-code contract".)

### Where results land

- **Raw samples**: each binary writes
  `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_<payloadbytes>.bin`:
  raw little-endian `u64` nanoseconds, one per sample. Binaries fail fast
  if either variable is unset; there is no silent default path.
- **Aggregated CSVs**: `compile_csv.py` reduces `.bin` files to
  floor/p50/p99/max (+ additional percentiles) per cell. Reviewed runs are
  committed under `results/<machine-hash>-<date>-<variant>/`; the
  pacing variant is part of the directory name so quiescent and
  back-to-back artifacts can never silently mix, and each rep of a
  multi-rep campaign lands in its own `rep<k>/` subdirectory (reps
  accumulate, never overwrite). The machine hash comes from
  `tools/scripts/benchmarks/lib/machine_hash.sh`
  (`compute_live_machine_hash`: a 16-char sha256 over CPU model/threads,
  kernel, governor, PREEMPT_RT), so every CSV row ties back to exactly one
  documented host. Percentile cells whose tail is too thin to be citable
  (< 20 expected exceedances) are emitted EMPTY, with a loud stderr
  note (`METHODOLOGY.md` §2); with reps present, the cross-rep
  aggregation reports the median of per-rep p50s as the headline plus
  min/max rep-spread columns (`METHODOLOGY.md` §10).
- **Run manifest**: every run dir carries `run.json`: git sha, machine
  hash + its plaintext input fields, uname, governor + turbo/boost
  state, per-cell start/end timestamps, the skip inventory, and the
  docker image IDs used. This is the machine-hash-keyed host record the
  boundary discipline cites; per ROS 2 cell, the driver additionally
  dumps the container's installed package versions to
  `_logs/<cell>_versions.txt` (`METHODOLOGY.md` §15).
- **Plots**: `plot.py` renders from the CSVs; line keys are the pinned
  `raw_prefix` names above. With reps present it draws the median line
  with the rep spread as a shaded band.
- `check_percentile_parity.py` is this tree's test harness: stdlib-only
  hand-oracle arms that check `bench.py`'s inline percentile math (the
  smoke gate) against `compile_csv.py`'s, and that drive the REAL shell
  runners' input gates, the REAL plot helpers and renders, and
  `bench.py`'s cell-minting guards, on a host with no ROS 2, no docker
  and no Linux. No CI job runs it, so run it before pushing:
  `python3 check_percentile_parity.py` (PASS/FAIL, exit 0/1).

Reporting convention: **floor / p50 / p99 / max, never max-free.** A
percentile table without its max hides exactly the tail robotics cares
about.

## Machine matrix

| Machine | Class | Role |
|---|---|---|
| box-x86 | x86-64 Linux workstation (WAITPKG) | **Primary.** The full sweep runs here; committed baselines and results come from this machine. |
| box-jetson | aarch64 Jetson Orin (WFE) | Cerulion-vs-Cerulion platform lines; published as [`9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx`](../../docs/benchmarks/results/9b0c5fbf0f55dea4-2026-09-17-fixed100-jetson-orin-nx/). |
| mac | Apple Silicon macOS | Cerulion-vs-Cerulion platform lines, published as [`0b7bc3994f78e232-2026-09-17-fixed100-apple-m4`](../../docs/benchmarks/results/0b7bc3994f78e232-2026-09-17-fixed100-apple-m4/), plus dev smoke. chrt-on and `no_shm` cells are skipped here by design, and with no CPU governor to pin a macOS sweep carries no governor receipt, so it stays a platform package and is never mixed into the cross-stack comparison. |

The first column is a short label for the machine, not a host name; each
published package records its own capture alias and machine hash in
`run.json`. Absolute numbers are strongly hardware-dependent; the
committed results directory name carries the machine hash so numbers from
different machines can never be silently mixed. Relative comparisons
(Cerulion vs ROS 2 vs raw floor) and curve *shapes* (flat-with-payload vs
climbing) are the portable story.

## Reporting requirements

- `results/` stays empty until real runs land. No placeholder CSVs, no
  synthetic samples, anywhere (Principle #13).
- Percentiles are computed offline from committed raw `.bin` samples;
  every published number is independently recomputable.
- Every cell that doesn't run is a printed skip with a reason, not a
  silent absence.
- Delivery is accounted, never inferred, and on BOTH stacks the
  accounting is **reported, never gated**: workspace legs print per-node
  `RTT_DELIVERY` lines plus the host's `drop_oldest` telemetry per size;
  ROS 2 cells print `DELIVERY` lines the driver persists per cell (a
  BEST_EFFORT deficit at large payloads is a finding about the QoS, not
  a harness failure). The hard gate on every stack is the exact
  measured-sample-count check on each `.bin`. See `METHODOLOGY.md`
  § "Delivery accounting".
- Tail percentiles are suppressed, not decorated: a percentile cell
  whose expected tail-exceedance count (n × (1 − q)) is below 20 is
  emitted EMPTY by `compile_csv.py` with a loud stderr note; a p99.9
  that is really the max with extra steps cannot be quoted from a
  committed CSV (`METHODOLOGY.md` §2).
- Close comparisons are rep-disciplined: k ≥ 5 round-robin reps,
  median-of-reps headline, rep spread as the error bar, and no two-line
  claim inside either line's spread (`METHODOLOGY.md` §10).
- Binaries fail fast on a missing env contract rather than measuring
  something other than what was asked.
- Rebuild-always: every runner rebuilds the binaries and cdylibs it
  measures before measuring (`PITFALLS.md` #13). A stale binary measures
  code that is no longer in the tree, and nothing in the result says so.
