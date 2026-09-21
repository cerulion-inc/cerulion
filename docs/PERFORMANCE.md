# Cerulion Performance

Cerulion is a zero-copy, deterministic robotics runtime built on
[iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) shared memory. Its
defining performance property is **flat latency**: transport cost is independent
of payload size, because a subscriber reads a message directly out of shared
memory: there is no serialize, no deserialize, and no copy on the receive path.

> **How to read this doc.** Every measured number below traces to a package
> under [`benchmarks/results/`](benchmarks/results/), labelled with the hardware
> it was run on. Where a section cites an in-tree test instead, its numbers are
> the ceilings that test enforces, not run outputs. Absolute microseconds are
> platform-dependent; the **flatness** (the zero-copy claim itself) is not.

## The public fixed100 benchmark

In the fixed100 campaign, the Cerulion multi-process cell, measured with free run
enabled (lockstep is the default execution mode), reads 3.97 to 4.43 µs p50 from 64 B to
16 MiB, a 1.1× growth over the range at
a nominal 100 Hz offered load. Single-process reads 2.66 to 2.80 µs and
is flat. ROS 2 defaults grow 334× over the same range.
The product default is the multi-process column, not the single-process
result.
Round-trip latency under a nominal 100 Hz offered load in a single-host
ping/pong workload. Both ROS 2 lanes are ROS 2 Jazzy.

| Payload | Cerulion (multi-process, free run enabled) | ROS 2 (defaults) | vs. ROS 2 defaults | Cerulion (single process) | ROS 2 (composed + intra-process) |
|---|---|---|---|---|---|
| 64 B | 4.07 µs | 311 µs | 76× | 2.69 µs | 24.6 µs |
| 256 B | 3.97 µs | 310 µs | 78× | 2.68 µs | 28.3 µs |
| 1 KiB | 4.01 µs | 410 µs | 102× | 2.66 µs | 28.5 µs |
| 4 KiB | 4.01 µs | 442 µs | 110× | 2.68 µs | 28.4 µs |
| 16 KiB | 4.02 µs | 464 µs | 116× | 2.70 µs | 28.9 µs |
| 64 KiB | 4.05 µs | 464 µs | 115× | 2.75 µs | 25.9 µs |
| 256 KiB | 4.04 µs | 483 µs | 120× | 2.71 µs | 27.7 µs |
| 1 MiB | 4.08 µs | 11.0 ms | 2,687× | 2.76 µs | 29.8 µs |
| 4 MiB | 4.33 µs | 12.9 ms | 2,983× | 2.77 µs | 33.6 µs |
| 16 MiB | 4.43 µs | 103.9 ms | 23,422× | 2.80 µs | 33.4 µs |

All values are p50. Recording is ON for the native Cerulion rows (the product default); the ROS 2 and zenoh lanes do not run the native graph recorder. The multi-process column runs with free run enabled (CERULION_EXECUTION_MODE=free_run, an opt-in described under "Environment variables" in [`docs/user-api.md`](user-api.md)), the one change from the shipped defaults, whose execution mode is lockstep; every other setting is the default. The `vs. ROS 2
defaults` column is the ratio of the ROS 2 defaults p50 to the Cerulion
multi-process p50 (ROS 2 at its defaults against Cerulion multi-process with free run
enabled), computed from unrounded
p50s; dividing the rounded values shown may not reproduce the ratios exactly.
Each cell is the median of five per-rep p50s (k=5, 2000 samples per size per
rep).

The tail: the multi-process cell's p99 is 6.61 µs at 64 B and
10.45 µs at 16 MiB, within a few multiples of its p50 at every
size; lockstep mode holds p99 at 15.89 µs at 64 B and
45.78 µs at 16 MiB with p50s
5.54 µs and 14.36 µs.
Both series are in the package.

> **Benchmark configuration.** These are **internal benchmarks**,
> single-host ping/pong workloads. They are not guarantees and
> have no error bars. The offered load is nominally 100 Hz; at 16 MiB a stock
> ROS 2 round trip outlasts the 10 ms pacing interval, so its requests overlap
> in flight.
> The `ROS 2 (defaults)` lane is the zero-config `ros2 run` experience on Jazzy:
> default RMW (`rmw_fastrtps_cpp`), no profiles XML, no transport environment,
> plain `publish()` plus typed-callback receive, and three separate processes.
> It makes no claim about which transport path Fast DDS selects: it measures
> the defaults as they ship. The composed lane is a single process with the
> nodes on one executor and intra-process communications enabled (`benches/latency/METHODOLOGY.md`
> §16), which is why its latency is payload-flat; its shape-matched Cerulion
> counterpart is the single-process column, not the multi-process default. The
> process-count asymmetry is explicit: Cerulion multi-process uses 2 worker
> processes with 1 process crossing inside the measured window; ROS 2 defaults
> use 3 processes with 2 crossings. Executor models also differ
> (native graph workers versus independent `rclcpp` executors).

The fixed100 campaign is the source for the native comparison above: [campaign directory](benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/),
[HEADLINE.md](benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/HEADLINE.md), and
[provenance](benchmarks/results/8a84baf25d5d1710-2026-09-16-fixed100-heroes/PROVENANCE-TWO-BUILDS.txt). Raw `.bin` sample
dumps stay on the bench host; the package carries every summary CSV, `.rate`
sidecar, manifest and render needed to audit or re-plot it, and every published
number is recomputable from those CSVs. Provenance: two builds, stated in the package: the single-process, iceoryx2, zenoh and ROS 2 rows are on git sha 68a0d6196; the multi-process row was measured on the build that pins the recorder to a core (commit a68a2a987); ROS 2 image b18318026199 (Jazzy, harness at 68a0d6196; the ROS 2 rows are on the release-candidate harness and are not label-comparable with the earlier package's ROS 2 rows); STOCK posture (no RT tuning, no DMA lock); nominal 100 Hz offered load; k=5 reps x 2000 samples per size; one x86-64 desktop (24-core Intel Core Ultra 9 285K, performance governor); native Cerulion recorder ON, the default a visitor gets (always-on Flashback: rolling window + trace rings). The multi-process default runs with free run enabled (CERULION_EXECUTION_MODE=free_run, the opt-in execution mode); that is the only change from the shipped defaults. The single-process leg has no execution-mode axis.
The tail with the recorder ON: multi-process (free run enabled) p99 is 6.61 µs at 64 B and
10.45 µs at 16 MiB; single-process p99 is 9.41 µs at 64 B and
14.23 µs at 16 MiB. p99 is pooled across the five reps (n=10000 per
size). The full p99 table is below.

The multipliers above compare the Cerulion multi-process result with the
*ROS 2 defaults* lane. The composed lane is listed separately and is not part
of that ratio.

These campaign measurements are not comparable to the figures the in-tree
tests below print. The
fixed100 campaign measures a round trip with the payload fill excluded (the
payload is loaned, never written) under a nominal 100 Hz offered load across
process boundaries. `flat_latency_test` and `cross_thread_rtt_test`
measure fill-excluded floors (the minimum over iterations). A reader who
divides one quantity by another gets a meaningless number.

> **The tail, every size, recorder ON.** p99 pooled across k=5 reps (n=10000 per
> size), from the pooled series (free run enabled, recorder on and pinned to a core,
> registry swept before the run, run scheduled clear of OS timers). The multi-process
> default holds its p99 within a few multiples of its p50 at every size, from
> 6.61 µs at 64 B to
> 10.45 µs at 16 MiB, against a
> p50 of 4.07 µs to
> 4.43 µs. The lockstep series is
> listed beside it in the package HEADLINE as a diagnostic. Every measured
> multi-process p99 above is below 20x its own p50.

| Payload | Cerulion (multi-process, free run enabled) p99 | Cerulion (single process) p99 | ROS 2 (defaults) p99 | ROS 2 (composed + intra-process) p99 |
|---|---|---|---|---|
| 64 B | 6.61 µs | 9.41 µs | 568 µs | 31.5 µs |
| 256 B | 6.62 µs | 9.51 µs | 556 µs | 30.8 µs |
| 1 KiB | 7.77 µs | 9.35 µs | 604 µs | 31.2 µs |
| 4 KiB | 6.75 µs | 9.37 µs | 587 µs | 31.2 µs |
| 16 KiB | 6.45 µs | 9.71 µs | 614 µs | 32.5 µs |
| 64 KiB | 6.76 µs | 9.94 µs | 639 µs | 31.4 µs |
| 256 KiB | 7.63 µs | 10.03 µs | 653 µs | 42.1 µs |
| 1 MiB | 7.38 µs | 10.74 µs | 43.4 ms | 37.0 µs |
| 4 MiB | 8.18 µs | 11.66 µs | 36.0 ms | 36.5 µs |
| 16 MiB | 10.45 µs | 14.23 µs | 156.9 ms | 37.3 µs |

> **RT-tuned posture.** The fixed100 campaign ran the STOCK posture only
> (no RT priority, no DMA lock). No RT-tuned rows are published from it.

### ROS 2 over `rmw_cerulion`, in the same harness

The README round-trip chart draws a sixth line: ROS 2 Jazzy running over
Cerulion's transport through `rmw_cerulion`, measured by `benches/latency` in
the same harness, posture and pacing as the lanes above. This is ROS 2 over the
Cerulion transport, so rclcpp, the typesupport and the rmw C ABI are in the
measured path; it is not a native Cerulion number. Three ROS 2 nodes run as
three processes in one Jazzy container. The cell is
`jazzy_cerulion_shm_loan_be1_chrt0`: every hop publishes a loaned message and
takes a loaned message, so the transport never copies the payload.

| Payload | ROS 2 over `rmw_cerulion` p50 | ROS 2 over `rmw_cerulion` p99 |
|---|---|---|
| 64 B | 10.45 µs | 36.18 µs |
| 256 B | 10.58 µs | 45.06 µs |
| 1 KiB | 10.43 µs | 36.38 µs |
| 4 KiB | 10.52 µs | 26.68 µs |
| 16 KiB | 10.49 µs | 23.65 µs |
| 64 KiB | 10.65 µs | 53.26 µs |
| 256 KiB | 10.58 µs | 37.62 µs |
| 1 MiB | 11.12 µs | 37.21 µs |
| 4 MiB | 15.05 µs | 50.55 µs |
| 16 MiB | 17.47 µs | 54.38 µs |

p50 is the median of five per-rep p50s and p99 is pooled across the five reps
(k=5, 2000 samples per size per rep, n=10000 per size), the definitions of the
tables above. Every row held the full 100 Hz. The p50 stays between 10.43 µs
and 10.65 µs from 64 B through 256 KiB, then reads 11.12 µs at 1 MiB, 15.05 µs at
4 MiB and 17.47 µs at 16 MiB. The ROS 2 defaults column above reads 11.0 ms at
1 MiB and 103.9 ms at 16 MiB.

> **What is outside the timed window.** Payload fill is excluded on every node,
> as on every other line (`benches/latency/METHODOLOGY.md` §3). On
> `rmw_cerulion`, borrowing a loaned message initializes the payload, which is
> work the size of the payload, so the echo node borrows its reply ahead of
> time and refills that loan once nothing is in flight (§20 and §21). That
> work is real: in the package's acceptance smoke run the refill took a median
> of 310 ns at 64 B and 368 µs at 16 MiB. Four of 105,050 echoes found the
> prefetched loan missing and borrowed inside their own timed window; the
> package names them. These processes do not run the native graph recorder.

Package: [campaign directory](benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/),
[HEADLINE.md](benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/HEADLINE.md), and
[provenance](benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/PROVENANCE.txt).
STOCK posture, unpinned, performance governor, harness commit `9f30d2b3b`, on
the x86-64 desktop of the campaign above (same machine hash).

## Latency across payload sizes

The subscriber holds a reference into the publisher's shared-memory slot, so a
1 KiB message and a 16 MiB message cross the transport in the same time. The
measured evidence is the fixed100 table above: the multi-process p50 reads
4.07 µs at 64 B and 4.43 µs at 16 MiB.

Two in-tree tests gate the same property on every change. Both exclude the
payload *fill*, so they isolate transport, and both take the per-size floor
(the minimum over iterations) across five sizes from 1 KiB to 16 MiB:

| Test | Path | CI ceiling on the flatness ratio |
|---|---|---|
| `flat_latency_test` | one-way publish to receive | **1.5×** (`FLATNESS_MAX = 1.5`) |
| `cross_thread_rtt_test` | publish, echo and receive across threads | **2.0×** (`FLATNESS_MAX = 2.0`) |

(For contrast, ROS 2's default transport serializes and copies every message,
so its latency grows with payload: see the ROS 2 defaults column of the
fixed100 table above.)

> **How the gate reads flatness.** The ratio is max ÷ min across the per-size
> floors. The CI gate computes a *drop-one-outlier* robust form of it (it
> discards the single worst-size floor so one VM-stalled measurement window
> can't trip it) and fails above the ceiling. A real O(n) copy would inflate
> *every* size monotonically and blow past the gate. The message *fill*,
> writing your bytes into the slot, is O(n) and separate from transport; that
> is why the fill-*included* `latency_threshold_test` scales with size and
> carries absolute backstops instead (a small-message median under 500 µs, a
> large-message median under 2 ms). `flat_latency_test` excludes the fill and
> stays flat, which is what separates transport cost from payload size.

A real 3-node graph (`graph_latency_test`, a source, relay and sink round trip
through the scheduler, macro dispatch and transport) is gated the same way: a
p50 under 50 µs at the smallest payload, and a flatness ceiling of 1.5× from
64 B to 1 MiB. Run the tests to read the absolute figures for your hardware
(commands at the end of this doc).

## Demand-paged shared-memory pools

Cerulion sizes SHM slots by a per-message-type tier (up to 128 MiB for
image-class messages) so common payloads fit without configuration. On Linux and macOS
these `Static` pools are **demand-paged**: the tier-max reservation is
virtual/sparse and costs almost no resident RAM until it is written.

`shm_footprint_probe` (Linux, `#[ignore]`d, run by hand) pins three properties:

| Probe | What it asserts |
|---|---|
| one 16 MiB pool carrying 1 KiB payloads | resident `/dev/shm` is at most one eighth of the apparent reservation |
| many pools whose apparent reservation exceeds physical RAM | every pool creates, and resident `/dev/shm` stays under 128 MiB |
| a 128 MiB-tier pool beside a small-tier pool | the big tier's floor, p50 and p99 stay within 2× of the small tier's |

An apparent reservation *larger than physical RAM* stays resident-tiny: the
tmpfs/commit limit binds on the resident working set, not the reservation. So
a generous default tier costs little resident memory on the deploy platforms,
and a 4K/8K camera frame (24 to 95 MiB) fits inside the image-class tier. A
payload larger than its tier is still refused.

(Windows commits shared memory eagerly, so this demand-paging property is
specific to Linux and macOS; Windows is not a supported platform.)

## vs Copper Robotics (cu29)

(Copper claims below are from its public documentation; check the current version
before relying on specifics.)

| Dimension | Cerulion | Copper |
|---|---|---|
| Transport | iceoryx2 shared memory (same- **and** cross-process, zero-copy) | CopperList (cache-line contiguous), single-process; cross-process via an iceoryx2 integration (not native) |
| Network | native zenoh | not built-in |
| Scheduling | runtime DAG-level, deterministic, 4 trigger policies | compile-time generated |
| ROS 2 compat | native `.msg` codegen + `rmw_cerulion` drop-in | not documented |
| Dynamic nodes | runtime cdylib loading | compile-time only |

**Where Copper wins:** compile-time scheduling (zero runtime dispatch overhead);
embedded/baremetal targets (Cerulion targets Linux/macOS; Windows is not
supported).

## Determinism

Execution order is derived from the graph DAG, not from callback timing: the
scheduler assigns nodes to dependency levels and fires each level in graph
order, where a ROS 2 `MultiThreadedExecutor` dispatches callbacks in the order
they become ready. The scheduler runs
on a `VirtualClock` for stepped, bit-for-bit reproducible re-execution of
scripted inputs. Deterministic record/replay of a *live* run is supported:
`cerulion graph run --record` + `cerulion bag play <bag> --resim all --verify`
re-executes the
recording and byte-diffs every produced frame, with a stable exit-code
verdict (the `bag play --resim` entry in [`user-api.md`](user-api.md) states the exit-code contract).

## Zero-copy verification

These properties are enforced by CI tests, not asserted by hand:

| Test | Checks |
|---|---|
| `flat_latency_test` | full publish→receive path is flat vs payload size (fill excluded), 1 KiB → 16 MiB |
| `cross_thread_rtt_test` | round-trip flat vs size, and ports are `Send` across threads |
| `zero_alloc_test` | zero heap allocations on the subscriber receive path |
| `zero_copy_ci_test` | payload bytes survive the round trip byte-identical |
| `graph_latency_test` | user-POV latency through a real `#[cerulion_node]` graph |
| `latency_threshold_test` | release-mode latency stays under a regression backstop |
| `shm_footprint_probe` | `Static` pools are lazy (resident ≪ apparent) and oversizing is latency-free |

## Logging

Cerulion uses `tracing` with compile-time level elimination:

- **Release builds:** `trace!()` and `debug!()` compile to **zero instructions**
  (not even an atomic load) via `release_max_level_info`; `info!()` and above
  stay active.
- **Debug builds:** all levels active.
- **No release override:** the `debug-logging` feature
  (`tracing/max_level_trace`) cannot lift that ceiling, because `tracing` reads
  every `release_max_level_*` feature first in a build without debug assertions.
  Use a dev-profile build (no cargo `--release`) when you need `debug`/`trace`
  lines.
- Structured fields (`topic = %topic, seq = seq`) are not formatted until a
  subscriber renders them.

So the hot path (publish + receive) carries no `trace`/`debug` logging overhead in
production.

## Running the benchmarks

```bash
# Transport flatness regression tests (release)
cargo test -p cerulion_core --test flat_latency_test --release -- --test-threads=1 --nocapture
cargo test -p cerulion_core --test cross_thread_rtt_test --release -- --test-threads=1 --nocapture

# User-POV graph latency + release-mode regression backstop
cargo test -p cerulion_core --test graph_latency_test --release -- --test-threads=1 --nocapture
cargo test -p cerulion_core --test latency_threshold_test --release -- --test-threads=1

# Lazy-pool / "oversize is free" residency probes (Linux only; #[ignore]'d)
cargo test -p cerulion_core --test shm_footprint_probe --release -- --ignored --test-threads=1 --nocapture

# The public latency suite (benches/latency)
python3 benches/latency/bench.py smoke
```
