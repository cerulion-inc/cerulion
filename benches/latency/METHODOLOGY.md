# Methodology

How every number in this suite is produced, and the rules that keep the
cells comparable. `README.md` covers *what* is measured (the line
inventory); this file covers *how*. `PITFALLS.md` covers what goes wrong.

No latency numbers appear in this document. Where a prior campaign's
finding shaped a rule, the finding is cited with its provenance. Those
numbers are not this suite's results.

---

## 1. Three pacing modes: quiescent (primary), fixed100 (uniform-rate), back-to-back (secondary)

One tree, three pacing modes, selected by `CER_BENCH_PACING`
(`bench.py --variant`). The cell matrix is identical in all of them,
with two workspace exceptions, stated here because "identical" would
otherwise overclaim:

- **`split` × backtoback is a structural skip (rc=77)**: back-to-back
  runs `--time-source virtual`, and on a `process_groups:` graph that
  dispatches to the supervisor with the time-source IGNORED (real-clock
  multi-process has no uncapped mode; the period floor is 1 ms); a
  "backtoback" split row would really be a 1 kHz-capped line masquerading
  as saturation, so the runner refuses to mint it.
- **`mono` × backtoback changes more than pacing**: `--time-source
  virtual` swaps the live WaitSet loop for the deterministic polled
  step loop. Its delta vs the quiescent mono cell therefore includes
  the executor-path change, not publish pacing alone; read it as the
  saturation bound it is, never as a pacing-only A/B.

Everywhere else, only the publish pacing differs between the modes.

### Quiescent: the primary suite

Publishers send at a fixed, payload-appropriate rate matching typical
robotics sensor rates, and sleep between iterations. Queues drain; the
system is mostly idle between messages.

> Question answered: *"In a typical robotics workload (camera @ 30 Hz,
> lidar @ 10 Hz, IMU @ 1 kHz), how long does each round-trip take?"*

This is the dominant production workload, the convention used by
industry-standard harnesses (the ROS 2 `performance_test` harness), and therefore
the apples-to-apples basis for customer-facing claims and any published
comparison.

Trade-off to know about (measured in the prior campaign): rate-limiting
forces the process to sleep between iterations, so at small payloads the
measurement includes the wake-up/context-switch cost on every iteration;
the quiescent small-payload p50 sits *above* the transport floor. That is
realistic (a 1 kHz control loop really does pay the wake-up), but it means:
**for pure transport-floor measurements at small payloads, read the
back-to-back variant.** The DMA lock (§7) exists to keep that wake-up cost
from being inflated further by CPU idle-state exits.

Scope of that mechanism claim (it is line-dependent, and the README's
topology matrix carries the classification): the wake/context-switch cost
lands **inside the timed window** only on blocking-receive lines (zenoh,
the workspace legs, every ROS 2 cell). The spin-receive floor
(`iox2_chrt0`) never blocks inside the window; its quiescent inflation
mechanism is post-sleep cold microarchitectural state (caches, branch
predictors, DVFS ramp after the pacer sleep), not a context switch. A
per-line quiescent-vs-backtoback delta should be read against that
line's mechanism, not a universal one.

### fixed100: the uniform-rate variant

ONE target rate, **100 Hz, at every payload size**, with uniform
counts (2 100 total / 100 warmup → 2 000 measured, tail-resolved). This
is the field convention for a payload sweep (§17, "The rate axis"):
holding the rate constant makes the idleness tax a CONSTANT across
sizes, so payload-flatness is measurable without the rate axis riding
along. The sensor-rate quiescent schedule stays the REALISM variant:
it answers "what does a robot's actual topic mix observe", while
fixed100 answers "how does latency scale with payload, rate held
fixed".

> Question answered: *"At one fixed publish rate, how does round-trip
> latency scale with payload size?"*

**The fallback ladder.** 100 Hz at 16 MB sits at a copying transport's
feasibility edge (§17), so a cell that cannot SUSTAIN the target at a
size (delivery accounting: the full measured sample count did not
arrive within the watchdog window, or (workspace legs) `drop_oldest >
0`, or (native bins) the pacer skipped > 1 % of its grid slots) steps
DOWN the ladder **100 → 50 → 20 → the size's sensor rate when that is
below 20 Hz** (strictly descending; today only 16 MB @ 10 Hz gains the
4th rung; the sensor-rate schedule is the ladder's floor) and RE-RUNS
the size at the next rung, fresh warmup included. Counts stay the
uniform (2 100, 100) at every rung, so the measured 2 000 is
rate-independent and the exact-count CSV gate holds everywhere. The
ladder is pinned in three places (`native/src/lib.rs::fixed100_ladder`,
`bench.py::fixed100_ladder`, `workspace/run_workspace.sh::ladder_for`);
the (rate, total, warmup) tuple shares the §2 four-place lockstep.

**No silent mixed-rate lines.** The rate a size ACTUALLY ran at is
recorded in a `<cell>_<size>.rate` sidecar beside the `.bin` (first
line: integer Hz, or `did_not_sustain`); `compile_csv.py` folds it into
the `achieved_rate_hz` CSV column, and `plot.py` annotates any point
whose achieved rate is not the 100 Hz target (`@20Hz` at the marker,
kept in `--release` renders: an unmarked fallback point would claim a
measurement the cell never made). A cell that exhausts the ladder at a
size renders **"did not sustain"** (an EMPTY CSV row plus a loud note
and an in-figure gap explanation), never a latency number (Principle
#13). That outcome is itself a product contrast, not a harness failure:
the accurate rendering of "this transport cannot move 16 MB at any ladder
rate" is exactly that sentence.

### Back-to-back: the labeled secondary

Every iteration publishes the instant the previous round-trip completes.
Publishers and subscribers are always busy; queues fill; the system runs
at maximum throughput.

> Question answered: *"If I push data through this transport as fast as it
> will accept, what does each round-trip cost?"*

Useful as the transport's saturation bound and for the clean small-payload
floor (the executor never sleeps, so no wake-up cost). **Not** what a user
should read as "latency" at large payloads: there, the measured time is
dominated by queueing and memory-bandwidth saturation, not transport
latency (the prior campaign measured order-of-magnitude divergence
between the two modes at 16 MB). Back-to-back results are always labeled
as saturation numbers.

## 2. The schedules

### Payload sweep (pinned)

10 sizes: 64 B, 256 B, 1 KB, 4 KB, 16 KB, 64 KB, 256 KB, 1 MB, 4 MB,
16 MB. (The prior campaign trees disagreed between 9- and 10-size sweeps; this
suite pins 10, with 256 B included.)

**ONE INVENTORY, EVERY RUNNER**. All four
mirrors of the `CER_BENCH_PAYLOAD_SIZES` override (`bench.py::
ambient_payload_restriction`, `ros2/run_bench.sh`,
`workspace/run_workspace.sh` and `native/src/lib.rs::sweep_payload_sizes`)
refuse a size outside this list, and they refuse it identically.

The rule is a CAMPAIGN constraint, not a transport one, so it is
deliberately stricter than what an individual runner could carry: the
workspace graph's variable payloads would accept other in-range sizes on
their own, and the decision is that the campaign does not, because a size no
other stack measures produces a row comparable to nothing while parity
checks and plots key on one size list. A per-runner scoping was proposed
and rejected; if a new size is wanted, it is added HERE and to all
four mirrors together.

### Quiescent schedule (pinned; lockstep in four places)

Roughly a one-minute run at every payload (total iterations = rate ×
60 s), the leading iterations discarded as warmup; the per-size rate is
anchored to a real sensor class. The two largest sizes run **extended,
tail-resolved windows**: ~70 s at 4 MB and
~205 s at 16 MB, sized so the measured count reaches 2 000 and a
single-rep p99 is citable at every size (the ≥ 20-exceedance rule
below); the rate stays on the sensor anchor; only the window grows:

| Payload | Rate | Total iterations | of which warmup (discarded) | Measured samples | Real-world analog |
|---|---|---|---|---|---|
| 64 B to 1 KB | 1000 Hz | 60 000 | 5 000 | 55 000 | IMU, joint state, control loops |
| 4 KB to 16 KB | 500 Hz | 30 000 | 2 500 | 27 500 | Joint encoders, sparse sensors |
| 64 KB | 200 Hz | 12 000 | 1 000 | 11 000 | Small image, sparse scan |
| 256 KB | 100 Hz | 6 000 | 500 | 5 500 | Compressed thumbnail |
| 1 MB | 60 Hz | 3 600 | 300 | 3 300 | Depth frame, point cloud |
| 4 MB | 30 Hz | 2 100 | 100 | 2 000 | HD camera, full lidar scan (~70 s window) |
| 16 MB | 10 Hz | 2 050 | 50 | 2 000 | RGBD frame, dense lidar (~205 s window) |

Measured samples per size = total − warmup, the **Measured** column.
That column is the suite's sample-count contract:
**`CER_BENCH_TARGET_SAMPLES` means MEASURED samples in every
component** (§11). The schedule tuple itself stays
`(rate, total, warmup)`; the runners derive measured = total − warmup
before exporting it (`bench.py` feeds the derived value to the ROS 2
containers and the workspace runner; `ros2/run_bench.sh` and
`workspace/run_workspace.sh` derive the same value from their own
schedule copies when driven directly), and every `.bin` sample-count
gate checks the measured count (`bytes / 8 == measured`). Every stack
therefore dumps **identical** per-payload measured counts (55 000 at
64 B down to 2 000 at 4/16 MB), which is what makes cross-stack CSV
rows apples-to-apples.

**The schedule lives in four places and MUST stay in lockstep:**
`native/src/lib.rs` (`quiescent_schedule`), `ros2/run_bench.sh` (the
per-payload case statement), `bench.py` (`quiescent_schedule`), and
`workspace/run_workspace.sh` (`schedule_for`). A drift between them
makes cells silently non-comparable. Any schedule change is a
four-file change.

### Which percentiles the schedule can support

A percentile q estimated from n samples rests on its **tail
exceedances**, the expected n × (1 − q) samples above it. This
suite's rule is **≥ 20 exceedances or the number is not citable**, and
the rule is mechanical, not aspirational: `compile_csv.py` emits any
tail-percentile cell below the threshold **EMPTY**, with a loud stderr
note counting the suppressions; a too-thin p99.9 cannot be quoted
from a committed CSV. Per size (parenthesized values = expected
exceedances, n × (1 − q)):

| Payload (measured n) | p50 | p95 | p99 | p99.9 |
|---|---|---|---|---|
| 64 B to 1 KB (55 000) | ✓ | ✓ (2 750) | ✓ (550) | ✓ (55) |
| 4 KB to 16 KB (27 500) | ✓ | ✓ (1 375) | ✓ (275) | ✓ (27.5) |
| 64 KB (11 000) | ✓ | ✓ (550) | ✓ (110) | ✗ (11) |
| 256 KB (5 500) | ✓ | ✓ (275) | ✓ (55) | ✗ (5.5) |
| 1 MB (3 300) | ✓ | ✓ (165) | ✓ (33) | ✗ (3.3) |
| 4 MB (2 000) | ✓ | ✓ (100) | ✓ (20, exactly at the floor) | ✗ (2) |
| 16 MB (2 000) | ✓ | ✓ (100) | ✓ (20, exactly at the floor) | ✗ (2) |

In words: **p50, p95 and p99 are citable at every size**; the top two
sizes sit exactly at the 20-exceedance floor, which is what their
extended windows were sized for (this is the tail-resolved
schedule change; the previous revision suppressed p99 at 4/16 MB with
n = 1 650 / 550, trading sweep time for tail citability; that trade
is now taken: ~70 s at 4 MB, ~205 s at 16 MB); **p99.9 is citable only
at 64 B to 16 KB**; at n = 2 000 a p99.9 rests on ~2 tail samples,
i.e. it is the max with extra steps. These bounds cover **within-run
sampling error only**; the between-run environmental term is larger
and is governed by the rep rules in §10 (reps also pool: k reps of
2 000 raise the pooled tail count).

### Workspace legs: tick source vs pacing authority

The workspace legs pace in TWO layers. The ping node's
`#[cerulion_node(period_ms = N)]` attribute (sed-rewritten per size,
`PERIOD_MS = 1000 / rate` integer division) is only the **tick
source**; the **pacing authority is a wall-clock slot grid** inside the
ping node (`CER_BENCH_TARGET_RATE_HZ`, ns-resolution, the same
grid-slot semantics as the native `RateLimiter` and the ROS 2 timers:
a tick before its slot publishes nothing, a late tick consumes exactly
one slot and SKIPS missed slots rather than bursting, with the skip
count printed as `slots_skipped=` on the ping's RTT_DELIVERY line and
gated by the fixed100 sustain verdict at 1 % of the window's slots).

<!-- Benchmark provenance: the remeasured split rows came from 2026-08-14-fixed100. -->

The gate exists because the period attr alone was only true on ONE
leg: the mono leg's single-process live loop fires periods on the wall
clock, but the SPLIT leg runs multi-process on the
handed-quantum barrier clock, where `period_ms` is **logical** time:
the cohort steps as fast as the barrier turns. Measured on
the bench machine: a split leg without the gate free-runs at ~920 Hz wall while
its labels (and fixed100 `.rate` sidecars) say 100 Hz: **every
ungated split-leg rate label, quiescent and fixed100, is false**; the
earlier fixed100 run directory's split rows were re-measured with the
gate, and older quiescent split-leg campaign rows should be read as
free-running (their latency samples are real wall RTTs; only the rate
label was wrong).

The integer-period quantization is therefore a TICK-cadence fact, not
a publish-rate fact: `PERIOD_MS` floors, so ticks fire at ≥ the target
rate (60 Hz → 16 ms ticks = 62.5 Hz) and the wall grid gates publishes
back down to the exact schedule rate. The runner still prints the
effective tick rate beside the schedule rate in each leg's per-size
log so the two layers stay visible in the artifacts.

### fixed100 schedule (pinned; same lockstep)

Every size runs the SAME tuple, **100 Hz, 2 100 total, 100 warmup →
2 000 measured** (per-size window ~21 s at the target), so the
measured count, and therefore every percentile's citability row above
(the 4/16 MB line: p99 exactly at the 20-exceedance floor), applies
uniformly. A size that falls down the fallback ladder (§1) keeps the
same counts at 50/20/10 Hz (windows ~42 s / ~105 s / ~210 s): the
exact-count contract is deliberately rate-independent. The tuple lives
in the same four places as the quiescent schedule
(`native/src/lib.rs::FIXED100_{RATE_HZ,TOTAL,WARMUP}`,
`ros2/run_bench.sh`'s fixed100 arm, `bench.py::fixed100_schedule`,
`workspace/run_workspace.sh::FIXED100_*`) and any change is a
four-file change; the ladder adds a three-place lockstep
(`fixed100_ladder` ×2 + `ladder_for`). On the workspace legs every
ladder rung divides 1 000 ms exactly, so the integer-period
quantization note above does not bite this variant.

### Back-to-back counts

10 000 measured + 1 000 warmup per size
(`CER_BENCH_TARGET_SAMPLES` / `CER_BENCH_WARMUP` defaults;
`TARGET_SAMPLES` is the measured count here too).

### Smoke override

`CER_BENCH_SMOKE_N` overrides (total, warmup) for the smoke gate while
keeping the pacing mode: a smoke run is a short run of the *same*
measurement, never a different measurement. `bench.py smoke` hands the
paced native/workspace cells `CER_BENCH_SMOKE_N=SMOKE_TOTAL` (1000; the
schedules carve warmup = N/10 out of it) and the ROS 2 + backtoback cells
the equivalent MEASURED pair (`CER_BENCH_TARGET_SAMPLES=SMOKE_MEASURED`
= 900, `CER_BENCH_WARMUP=SMOKE_WARMUP` = 100), so **every smoke cell
retains 900 measured samples after 100 warmup** whichever spelling
carried the count (pinned by `check_percentile_parity.py`). Smoke
baselines captured before this alignment gated the
MEASURED-pair consumers (ROS 2 cells on every variant; native/workspace
under backtoback) at 1000 measured, immaterial to the ±2x catastrophe
gate.

## 3. Fill exclusion: "Mode A" (payload writes never in the timed window)

Every cell in this suite measures **transport latency**, defined as: the
time from the publisher committing a message to the subscriber observing
it (× 2 for the round trip), with **zero payload-byte writes inside the
timed window**.

- Native binaries loan an SHM slot (`loan_slice_uninit(N)` /
  Cerulion `loan_data(N)`) and never touch the returned bytes.
- The zenoh cells allocate one SHM buffer outside the loop and never
  write it, cloning the handle per iteration, strictly purer than
  zenoh's own `z_ping` convention (which pre-fills a pattern once,
  also outside the loop).
- ROS 2 cells use fixed-size POD message types whose timestamp field is
  the only written member. On the non-loan path (the rclcpp nodes and
  their rcl variants), ping and pong **preallocate the outbound message
  once, outside the measurement loop, and reuse it**: rosidl's default
  constructor zero-fills the entire payload array, so a per-iteration
  construction is exactly the O(N) fill Mode A excludes, and on ping's
  side it would sit *between* the stamp and the publish. Per iteration,
  the only work between the stamp and the publish is the 8-byte
  timestamp write (ping) / timestamp copy (pong). The loan paths keep
  their per-iteration loan (the loan **is** the middleware cost being
  measured) with no payload writes inside the window.

This matches the upstream conventions of both reference transports
(iceoryx2's canonical bench does no payload writes by default; its
`--send-copy` fill mode is explicitly a separate measurement). The
rationale is arithmetic: a per-iteration fill is O(N) memory-bandwidth
work, and including it turns a transport measurement into a memcpy
measurement at large payloads, for *every* transport, drowning exactly
the differences the bench exists to show. Application fill cost is real,
but it is the application's cost, identical whichever middleware carries
the bytes. (The prior campaign's predecessor made this mistake, and
its "exponential" latency curve came entirely from the bench's own fill
loop.)

Where a receive path itself copies (the ROS 2 `rclcpp` callback lane
memcpys the payload into a fresh heap message before the user callback
runs), that copy **is** measured; it is part of the transport's delivery
path, not application fill. That distinction is the whole point of the
`rclcpp` vs `loan` receive-path axis.

**Labeled difference from the prior campaign:** the prior
pong constructed a fresh message per echo (a per-echo `make_unique`),
so its large-payload ROS 2 numbers included the constructor's zero-fill
of the payload array inside the timed window. This suite deliberately
preallocates instead (the rule above). Consequence: the
climbing-with-payload ROS 2 curve must re-earn itself from the
receive-side delivery copy alone (the `rclcpp` lane's per-receive
memcpy); it can no longer borrow slope from the bench's own message
construction. Prior-campaign-vs-this-suite ROS 2 comparisons at large payloads are
therefore **not** apples-to-apples on the non-loan path.

## 4. Timing and the wall-stamp rule

All timestamps are `CLOCK_MONOTONIC` nanoseconds (`wall_ns()` in the
native crate and the C++ common header, the same kernel clock on both
sides of every process boundary).

Two timing boundaries, forced by process shape:

- **Harness-bracketed** (native + zenoh cells): the initiating thread
  brackets publish→echo-receive with two clock reads.
- **Embedded-timestamp** (workspace + ROS 2 cells): the ping node stamps
  the send time into the message payload; the sink node reads its own
  clock on receive and subtracts. Required wherever dispatch is
  scheduler- or executor-driven and no single thread can bracket the pair.
  Both endpoints read the same `CLOCK_MONOTONIC`, so the two boundaries
  measure the same event sequence.

**The stamp-last rule (G3) applies to every embedded-stamp line
uniformly**: the send stamp is taken and written as the LAST work
before the publish; on the ROS 2 ping the only thing between the
stamp and the publish is the 8-byte timestamp write, and the workspace
ping node is structured the same way (all port setup, the lazy SHM
loan, and `loan_data(N)` happen BEFORE the stamp; the stamp's two
fixed-field writes are the final port writes before the Drop-commit
publishes). Receive side is read-then-stamp everywhere. This is what
makes the two stacks' timed windows cover the same event span
(publisher commit → subscriber observation) rather than one stack
paying its message-prep inside the window while the other excludes it.
Pacing is uniformly excluded the same way: every line takes its stamp
AFTER the initiator's wake (harness bracket after the pacer sleep;
ROS 2 ping stamps inside the kick callback after the executor woke it),
so initiator wake cost never rides any line's window.

### The wall-stamp rule: deterministic stamps are the platform default

Cerulion nodes stamp the **deterministic gating clock**
(`now_ns()`) by default so that `graph run --record` replays byte-exact.
Deterministic-clock stamps are useless for wall latency: under
`VirtualClock`, a publish-side and receive-side stamp inside one
scheduler step read identical values, and every RTT computes to zero.
Any latency-measuring node must stamp the real kernel clock via
`real_ns()`, the framework's sanctioned explicit-source real-clock
accessor.

**This suite's workspace nodes stamp `real_ns()` unconditionally**: no
env gate. There is no record/replay use of this workspace, so the
deterministic-stamp mode has nothing to protect here, and an
unconditional wall stamp also keeps the workspace lines on the same
clock source as the native floor bench. This differs from
graphs built as record and replay assets, whose nodes stamp the
deterministic clock by default and only stamp wall time under
`CER_BENCH_WALL_STAMP=1`. `run_workspace.sh` still exports
`CER_BENCH_WALL_STAMP=1` on every leg for env-contract parity across the
suite (a no-op today, load-bearing if these nodes ever adopt the
robotics gating). If you port a robotics-style gated node into a latency
harness, the unset-gate failure mode is zero samples collected; see
`PITFALLS.md` #16.

This is a bench-only, documented use of the framework's non-deterministic
escape hatches; production nodes should never stamp the real clock on the
hot path.

## 5. Sample collection and reporting

- Binaries write raw samples only: `${CER_BENCH_RAW_DUMP_DIR}/`
  `${CER_BENCH_RAW_NAME}_<payloadbytes>.bin`, little-endian `u64` ns per
  sample. Both env vars are **required**; binaries fail fast if either is
  unset. No percentile math and no CSV writing happens inside a
  measurement binary.
- `compile_csv.py` computes percentiles offline from the `.bin` files.
  `check_percentile_parity.py` pins that the smoke gate's inline
  percentile math agrees with `compile_csv.py`: one definition of "p50",
  verified, not assumed.
- **The estimator is named so reproducers can match it:** both
  implementations compute the linear-interpolated quantile,
  Hyndman-Fan **type 7**, numpy's default: rank = q × (n − 1), linear
  interpolation between the bracketing order statistics. Nearest-rank
  and HDR-histogram conventions common in latency literature read
  systematically HIGHER in the tail at low n (type 7 interpolates
  between the top order statistics); an external "failure to
  reproduce" a committed tail number under a different convention is
  an estimator difference, not a data discrepancy.
- Reporting is **floor / p50 / p99 / max, never max-free**. The floor
  shows what the path *can* do; the max shows the worst thing that
  actually happened. Percentile tables that stop at p99 hide the exact
  stalls robotics deadlines care about.
- **floor and max are per-run extreme statistics, not estimated
  percentiles**, and both are n-dependent: the expected minimum over
  55 000 draws (64 B) sits lower than over 2 000 draws (16 MB) purely
  from sample count. Cross-size and cross-cell SHAPE claims are
  therefore made on **p1/p10** (present in every CSV row), and any
  future flatness GATE on this data must use p10, per
  `benches/AGENTS.md`. max is reproducible in distribution, never in
  value: a published max names its run, and recurring-vs-one-off
  outliers are separated by the rep structure (§10), never by one
  run's max. The `.bin`s preserve chronological sample order by
  design, so an outlier can be located in time and correlated across
  the leg's processes before being attributed to the transport.
- **`one_way_p50_ns` is derived, never measured**: it is
  `round_trip_p50 // 2`, valid only for path-symmetric lines, and
  even there, half the median of a two-hop sum is not the median
  one-way in general. It is the customary approximation (zenoh's
  `z_ping` reports the same); the CSV schema comment in
  `compile_csv.py` carries the same definition.
- Warmup samples are collected and discarded per the schedule (§2):
  first-iteration costs (connection establishment, cache warmup, lazy SHM
  page-in) are real but are start-up costs, not steady-state latency.
- **Timer granularity bounds what a sub-µs delta can mean** (prior-campaign
  lesson): `Instant::now()`/`CLOCK_MONOTONIC` has a coarser tick on ARM
  (Apple Silicon / Jetson) than on x86, so a difference between two
  readings that is near the tick size is timer noise, not signal. This
  never threatens the claims this suite makes (a real zero-copy failure at
  a multi-megabyte payload shows up as a memcpy orders of magnitude above
  the floor), but any future gate comparing near-floor numbers must carry
  an explicit noise floor rather than asserting raw equality.

## 6. Delivery accounting: full delivery is measured, not inferred

Every workspace leg prints delivery accounting alongside its latency
result: the ping node
reports `published`, the pong node `forwarded`, the sink node
`received` / `skipped` / `measured` (the per-node `RTT_DELIVERY`
lines), and the host runtime prints its per-input `drop_oldest`
eviction counters ("live loop delivery telemetry", one line per
data-trigger input, per process).

The **no-silent-loss reading** of those lines is:

```
received ≈ forwarded ≈ published   AND   every drop_oldest == 0
```

Every published frame was either delivered or superseded by a newer
frame in the same drain (latest-wins is the data-trigger drain's
by-design semantic). Expect a small `received` deficit on hosts prone
to timer coalescing (a late wake publishes >1 frame in one step; the
drain keeps the latest): coalescing accounted by the counters, never
silent loss. Neither reading is `n == window × rate`; the wall window
also contains graph build and drain time, so a fires-vs-expected gap is
window arithmetic, not loss.

**On both stacks the accounting is REPORTED, never gated**: the
runner warns only when the accounting LINES are missing from the log,
it does not parse them into a pass/fail. The hard gate on every stack
is the `.bin` sample-count check: each `.bin` must hold exactly the
schedule's **measured** count for its size (§2); an under-sampled
cell is a failed cell, never a quietly thinner percentile. A latency
percentile from a leg that silently dropped frames is a survivor
statistic; that is why the accounting is persisted as a first-class
artifact beside every number: the receipts are published so a
reviewer can audit the collection cost, and a row whose accounting
shows real eviction (`drop_oldest > 0`) or a large unexplained
`received` deficit must never be quoted without that context.

### ROS 2 cells: the same rule, plus the QoS caveat

ROS 2 cells print delivery accounting too: `ping_node` prints
`DELIVERY role=ping published=N` at exit, and the latency node prints
`DELIVERY role=latency received=M kicks_sent=K unstamped=U
nonpositive_rtt=P` at finalize (each of the bench binaries prints its
role's line as applicable; the loaned-take sink adds `take_failures=`
and `return_failures=`). `ros2/run_bench.sh` greps the node logs and
persists both lines to `_logs/<cell>_<size>_delivery.txt` under the raw
dump directory, so the accounting survives into the run artifacts.

`unstamped` and `nonpositive_rtt` are echoes that **arrived and yielded
no sample**: the message carried no stamp at all (`send_ns == 0`, a
publisher wiring fault), or its receive instant was not strictly after
its send instant (a duplicate stamp at the clock's resolution, or a
non-monotone one, and the subtraction is unsigned, so taking that
sample would record a ~1.8e19 ns "latency"). They are counted
separately because the remedies differ, and they are what makes the
identity `received == samples + warmup + unstamped + nonpositive_rtt`
readable once the receipt is set beside the `.bin`'s sample count and
the cell's warmup (neither `samples` nor `warmup` is on the DELIVERY
line itself, which is also why `run_bench.sh`'s own gate asserts only
the weaker `received >= samples`); before they existed the gap had no
explanation on the artifact at all. Each condition also logs ONCE at `WARN`
when it first fires, because a cell whose stamps are all unusable never
reaches finalize. For exactly that case the receipt is printed a second
way, when the run ends without finalizing, which works only because the
per-cell watchdog now climbs a ladder, SIGINT first with a bounded grace,
then SIGTERM with another, and SIGKILL only as the backstop. SIGINT is
what makes the second receipt reachable, and that is MEASURED on the
jazzy bench image rather than assumed: these binaries call plain
`rclcpp::init(argc, argv)`, which installs a handler for SIGINT and
nothing else, so `kill -INT` exits rc 2 with the DELIVERY line printed
while `kill -TERM` exits rc 143 with no line at all; the default
disposition kills the process before the spin can return. The later
rungs are escalation for a node that ignores the first, never the rung
that delivers the receipt. Ping and pong needed the same fix and did NOT
already have it: they were reaped with SIGTERM and a grace, which reads
like a graceful stop and is inert for this purpose, so their timeout-path
receipts had never been printed either. Under the latency node's previous
straight SIGKILL the spin never returned and the counts died with the
process.

Reading them BESIDE the Cerulion legs: the workspace sink applies the
same two-condition guard and already counts it, as `skipped=` on its
`RTT_DELIVERY role=latency` line, but as ONE number. So the quantity
that compares with it is `unstamped + nonpositive_rtt`, never either
count alone, and the ROS 2 cells simply report separately what the
Cerulion leg merges. (Do not read the ping's `slots_skipped=` as the
same thing: that is a producer-side pacing miss (a timer slot the ping
never published into) and has nothing to do with a stamp the sink
could not use.)

Here too the accounting is **reported, never gated**. Under
`be1` (BEST_EFFORT), loss at large payloads is a real property of the
configuration being measured, a finding to report in the results, not
a harness failure to retry away. The measured-sample-count gate on the
`.bin` still applies unchanged: the latency node must observe the full
measured count of echoes (a lossy cell simply needs more kicks, bounded
by the per-cell wall-clock timeout), so a cell that passes always has
its full sample population, and its `delivery.txt` tells you what it
cost to collect it.

### Bisecting a fixed latency term inside a chain (prior-campaign pattern)

When a multi-hop leg carries an unexplained fixed cost, bisect it with a
mid-chain stamp rather than guessing: have the MIDDLE node compute its
own observed delta against the initiator's stamp and overwrite a
non-load-bearing payload field with it, and have the sink emit TWO
result series, end-to-end (as today) and initiator→middle, so
`end_to_end − middle` isolates the second half. A relay-stamp probe
of this pattern located the park-cadence term of a multi-process
chain in one run.
Keep such probes opt-in (default byte-identical single-series behavior) so
the published legs never carry probe overhead.

## 7. The DMA lock and the chrt axis

Two knobs isolate scheduling effects; both are recorded per cell.

### CPU DMA latency lock

A `/dev/cpu_dma_latency` PM_QoS request held for the duration of every
cell prevents the CPU from entering deep idle states (C-states). On
wait-then-wake receive paths, C-state exit dominates tail percentiles;
without the lock, p99 measures the CPU's idle-exit latency, not the
transport (established in the prior campaign and in this repo's
wake-path work).

- Native binaries acquire it via `acquire_dma_lock()` and **refuse to
  run** if `/dev/cpu_dma_latency` exists but is not writable (a run
  without the lock has tails that measure the OS, not the middleware).
  `sudo chmod 666 /dev/cpu_dma_latency` fixes it until reboot; a udev
  rule (`KERNEL=="cpu_dma_latency", MODE="0666"`) makes it permanent.
  `CER_BENCH_ALLOW_NO_DMA_LOCK=1` downgrades the refusal to a loud
  warning for hosts where root is unavailable; such runs print
  `cpu_dma_lock: SKIPPED` into the runner log and are **not citable for
  p99/p99.9** (same escape-hatch pattern as
  `CER_BENCH_ALLOW_UNVERIFIED_SHM`).
- The `cerulion` CLI acquires it when `CERULION_CPU_DMA_LOCK=1`; the
  workspace runner sets this.
- ROS 2 containers get `--device /dev/cpu_dma_latency` (when present on
  the host) and the C++ nodes hold the same lock.

If the device is missing entirely (macOS; a container run without
`--device`), the lock **soft-fails with a loud warning** and the run
continues: valid, just potentially noisier in the tails, and not
comparable to locked runs. If the device is **present but the lock
fails anyway**, the per-cell smoke check aborts the cell (exit 12, §9's
exit-code table) rather than recording a silently-worse number under a
label that claims the lock was held.

**Prior-campaign finding (qualitative: its table is not published with
this tree, so no figure from it is quoted here):** retrofitting the lock
onto the same code, hardware and runs lowered the small-payload median on
every line class. The spin-bound raw iceoryx2 line moved least (it gains
background-task isolation only); the Cerulion listener-based lines and
zenoh-SHM moved most. The p99 picture was asymmetric: the listener lines'
p99 dropped with their medians, but **zenoh-SHM's p99 got WORSE under the
lock**. Working hypothesis: pinning the CPU out of idle forces zenoh's
several async tasks into harder contention instead of letting wait-states
multiplex. Recorded as an empirical finding, not a claim; it is why
tail deltas under the lock must be read per line, never as one
mechanism.

### chrt (SCHED_FIFO)

Cells suffixed `_chrt1` run every process of the cell under `chrt -f 80`
(SCHED_FIFO, priority 80): direct invocation when the user has `rtprio`
in `/etc/security/limits.conf`, else `sudo -nE chrt`. The prior
campaign's consistent signature (worth knowing when reading results)
is that RT priority is a *tail* tool: it protects p99/max from
preemption; it does not meaningfully move the median.

That signature INVERTS for multi-process spin cohorts (measured
2026-08-15): the split
leg's worker mains are ~94 %-CPU park/spin loops, and with the WHOLE
process tree at SCHED_FIFO 80 they exhaust the kernel's RT-bandwidth
budget (`sched_rt_runtime_us=950000` of a 1 s period ⇒ a forced
50 ms/s stall of saturating RT tasks). The observable is a comb:
p99/max spikes of 12.8 to 29.6 ms spaced exactly 1.000 s apart
(`rt_throttled=1` sampled live during the cell), and the same stalls
skip wall-grid slots, driving the fixed100 fallback-ladder steps the
tuned split leg shows (7/10 sizes below 100 Hz in the diagnostic run),
while the chrt-off split control holds 100 Hz with single-digit-µs
p99. So on split workspace legs chrt currently buys a strictly WORSE
tail; dropping whole-tree chrt from the tuned posture's Cerulion legs
(or chrt-ing only non-spinning threads) is PROPOSED, pending a
decision, since it changes what the `_chrt1` label means there. Never
"fix" this by disabling the RT-bandwidth guard; see PITFALLS.md #7.

Spin-bound cells (the raw iceoryx2 floor) are enumerated chrt-off only:
a spinning thread is already never descheduled voluntarily, so
SCHED_FIFO changes nothing measurable there.

### Governor, turbo, and CPU pinning: the declared stance

The DMA lock pins **C-states only**; P-states (frequency scaling,
turbo, EPP) are a separate axis, and silence about it reads as
oversight rather than stance, so here is the stance:

- **A citable sweep requires the `performance` governor** on the bench
  machine (README prerequisites). On a
  schedutil/ondemand machine the sleep-wake quiescent workload runs at a
  drifting clock frequency, and the exposure is asymmetric (one
  mostly-idle Cerulion process vs three ROS 2 node processes plus
  daemons keeping clocks higher), the DVFS cold-head class this repo
  has measured before. The runners read and PRINT the active governor
  and turbo/boost state per phase and record them in the run manifest
  (`run.json`); a governor change re-keys the machine hash, which is
  the right containment (differently-governed results can never be
  silently mixed). macOS exposes no governor, so a macOS sweep carries
  no governor receipt: it is published as its own platform package
  (Cerulion against Cerulion on that machine) and is never mixed into
  the cross-stack comparison.
- **No CPU pinning, deliberately.** No process on either side is
  pinned (zero occurrences of taskset/cpuset/isolcpus/affinity
  anywhere in the suite), so both stacks are placed by the stock
  kernel scheduler exactly as deployed, and the chrt axis is the only
  scheduling knob. Pinning would hand-pick a core topology per stack
  and open the reverse "you tuned placement" objection.
- **Daemons are deliberately not RT-wrapped** in chrt-on cells:
  iox-roudi (registry/setup only) and rmw_zenohd (discovery only:
  zenoh data flows peer-to-peer between same-host peers, §15) are off
  the per-message data path, so wrapping them would change nothing the
  window measures.
- **PM-QoS is host-global**: one `/dev/cpu_dma_latency` holder covers
  the whole machine, so a container-held lock and a host-held lock are
  equivalent; the lock cannot differ across the container/bare-host
  boundary (§14).
- **The machine must be otherwise idle** during a citable sweep: no other
  tenants, no concurrent sweeps (see also `PITFALLS.md` #21).

## 8. The two QoS pins (ROS 2 axis), and why both

ROS 2 QoS changes which code path the RMW takes, not just delivery
semantics. Two pins, each anchored to a prior campaign for
comparability:

- **`be1`**: BEST_EFFORT / VOLATILE / KEEP_LAST(1). The default and the
  prior campaign pin: what a latency-first sensor pipeline
  configures, and the eligibility-friendliest triple across the RMWs
  under test. One correction over earlier drafts of this suite,
  verified against upstream source: `rmw_fastrtps` does **not** gate
  `can_loan_messages()` on reliability: since Iron
  (ros2/rmw_fastrtps#568) the loan gate is `is_plain` alone, and
  reliability never appeared in it in any era. What `be1` buys is
  headroom on the constraints the documented SHM/zero-copy paths DO
  check (VOLATILE durability, bounded KEEP_LAST depth: CycloneDDS's
  iceoryx gate and Fast DDS DataSharing's reader-depth ≤ writer-depth
  rule), plus prior-campaign comparability. Whether a BEST_EFFORT pair
  actually rides CycloneDDS's iceoryx path is code-vs-docs ambiguous
  upstream (the 0.10.5 docs say RELIABLE is required; the 0.10.5 code
  gates never check reliability *kind*), which is exactly why SHM
  engagement is verified per (rmw, shm-mode, QoS lane) at the data
  plane (§9) instead of assumed from config. Note also that `loaned=1`
  in a CSV row proves the API-level loan engaged, NOT that a zero-copy
  data path carried the payload; the per-RMW meaning of the `loaned`
  bit, and Fast DDS's separate DataSharing (`zc`) axis, are in §15.
- **`rel10`**: RELIABLE / VOLATILE / KEEP_LAST(10). The QoS
  reliability-typical control stacks actually run (services are RELIABLE
  by construction). Durability stays VOLATILE in **both** pins; the
  axis moves reliability + history depth, nothing else. Enumerated only
  on jazzy × `shm` × {cyclonedds, fastdds} × rclcpp:
  **SHM-only**, because the pin was an SHM head-to-head (a
  `no_shm` rel10 cell answers no pinned question) and narrow enough to
  answer "what does RELIABLE cost per RMW?" without doubling the sweep.

Why both: `be1` answers "how fast can ROS 2 go on this transport when
configured for speed"; `rel10` answers "what do the settings most
production graphs run actually cost". A suite with only `be1` flatters
every RMW; a suite with only `rel10` denies the stock RMWs their loan
path. Which pin becomes the headline is decided **after** results exist,
not before.

## 9. Cell isolation and runner hardening

Rules encoded in the runners (restated here so a reader can audit a run
log against them):

- **Rebuild-always.** Every runner rebuilds the binaries *and* cdylibs it
  is about to measure, unconditionally, never `if [ ! -x ]`. A stale
  binary attributes the old build's behavior to the new code; this
  exact failure voided a full aarch64 result set in a prior campaign
  (`PITFALLS.md` #13). The workspace runner rebuilds the `cerulion` CLI
  binary and the node cdylibs from one checkout in the same pass, which
  is also the guard against binary↔cdylib iceoryx2 version skew
  (`PITFALLS.md` #15).
- **Clean transport state between cells and sizes:**
  `rm -rf /tmp/iceoryx2/{services,nodes}`, `/dev/shm/iox2_*` (and the
  zenoh/iceoryx-v1 segment patterns on ROS 2 hosts). POSIX SHM segments
  outlive their processes; a leftover segment from a dead cell corrupts
  the next one's numbers or startup.
- **`ulimit -n 65536`** before workspace runs (`PITFALLS.md` #14).
- **One ROS 2 cell = one fresh `docker run --rm`**: no state of any kind
  (SHM segments, discovery daemons, zombie nodes) survives a cell
  boundary; retries are fresh containers, not restarts. Flags per cell:
  `--shm-size=4g --cap-add SYS_NICE --ulimit rtprio=99 --ulimit
  memlock=-1 --device /dev/cpu_dma_latency` (when present), plus the
  namespaced ipfrag sysctl for `no_shm` cells (`net.core.{r,w}mem_max`
  is kernel-global and must be raised on the host; see README
  prerequisites). `BENCH_CELL_TIMEOUT_S=300` bounds each cell; 3 attempts
  per cell, partial `.bin`s cleared between attempts.
- **Per-cell stdout smoke checks** (the exit-11/12/13 convention from the
  prior campaign): the cell aborts and retries if the binary's startup
  lines don't confirm `is_plain` on the POD message type (else the
  loan/CDR-memcpy assumption is wrong and the row would be misleading)
  and the DMA lock engaging. The `loaned=0/1` column rides every ROS 2
  CSV so post-hoc analysis can tell which rows actually took the
  publisher-side zero-copy path; capability probing happens at runtime,
  never assumed from config.
- **SHM engagement is verified per (rmw, shm-mode, QoS-lane) batch, at
  the data plane, and a failed verification is a HARD failure.**
  `ros2/verify_shm.sh` smoke-publishes under the batch's exact config
  AND the cell's `CER_BENCH_QOS` (a QoS-mismatched probe can pass while
  the measured lane silently falls back: CycloneDDS's per-endpoint
  eligibility is QoS-gated), and checks the RMW's SHM engagement
  marker. For CycloneDDS the pass condition requires topic-attributable
  `Created new PublisherPort` / `Created new SubscriberPort` lines in
  the roudi-log delta during the probe window; bare runtime
  registration only proves the plugin LOADED, not that the bench
  topic's data rides SHM. A nonzero verify fails the cell into the
  retry / documented-empty path rather than recording rows whose `shm`
  label may be a silent UDP fallback. Escape hatch:
  `CER_BENCH_ALLOW_UNVERIFIED_SHM=1` records the cell anyway but writes
  a `_logs/<cell>_<size>_SHM_UNVERIFIED` marker under the raw dump
  directory and warns loudly on stderr in both the container and the
  runner; the rows exist, permanently labeled unverified, never
  silently trusted.
  (Marker patterns drift across releases: `PITFALLS.md` #6.)
- **Missed-slot pacing policy, unified suite-wide: SKIP, never
  burst.** Every pacer in the suite advances past missed slots, so a
  host stall is never followed by a catch-up burst (burst samples
  would measure back-to-back queueing under a quiescent label). The
  ROS 2 quiescent kick timer (latency node → `/kick` → ping) re-arms
  as `next_kick = now + period` after firing (the rclcpp
  lane reaches the same policy through rcl's whole-period catch-up
  clamp, which is phase-preserving, while the rcl lane's fire-time
  anchor accrues a small bounded rate droop under sustained lateness).
  The native Rust `RateLimiter` implements the identical policy
  grid-anchored: when a deadline is already past, it advances its
  iteration index to the next FUTURE epoch slot
  (`ceil(elapsed/period)`): phase-preserving, no burst, no droop.
  Under stall the effective rate droops below the schedule rate on
  every line; it never overshoots. The one residual is the workspace
  legs' Period trigger (product scheduler semantics the harness does
  not fork), bounded and bias-labeled in §13. The kick topic rides the
  cell's QoS profile (`bench_qos()`, so `rel10` where the cell says
  so), and kick counts ride the delivery accounting (§6).
- **Watchdog + flood guard (workspace runner):** each quiescent size
  runs under a watchdog of 2 × its nominal wall time + 60 s
  (back-to-back sizes: 120 s), a hung graph is killed and failed,
  not waited on, and
  the run log is scanned for iceoryx2 connection-flood symptoms (more
  than 16 "Unable to establish connection" lines fails the leg; that
  signature means the graph was wedged reconnecting, and any samples
  collected are suspect).
- **FastDDS SHM segment size** is raised to 256 MiB for the 16 MB cells
  (both directions of a 16 MB round trip must fit in flight), with
  `useBuiltinTransports=true` kept so participant discovery survives
  (`PITFALLS.md` #1).

### Exit-code contract

`ros2/run_bench.sh` (the per-cell driver; `bench.py` maps these, so do
not renumber):

| Code | Meaning |
|---|---|
| 0 | every requested size produced a full `.bin` (exact measured count), smoke checks clean |
| 2 | setup / env error (bad QoS or recv value, missing prereq, mislabeled cell) |
| 11 | The class label is contradicted or UNVERIFIABLE. Contradicted: `Msg::is_plain: 0` on a pod cell (the POD message is not trivially copyable; the loaned / CDR-memcpy assumption is broken: bad build), or the inverse `Msg::is_plain: 1` on an image cell. Unverifiable, and only where the run itself succeeded: the class's marker is ABSENT or the log cannot be read. Every node logs the line from its constructor, so a log without it LOST it (truncated, rotated, a dropped stderr redirect, a suppressed INFO level) or came from binaries older than `is_plain_check`; absence of evidence is not evidence. `check_structural_loan_skip` also exits 11 on an unreadable log. |
| 12 | `/dev/cpu_dma_latency` is present but the DMA lock failed; tails would silently include C-state exits (device absent = soft-warn, run continues) |
| 13 | one or more sizes produced a missing / short `.bin` (bootstrap timeout, transport failure, sample-count mismatch) |
| 77 | structural skip: the loan lane on an RMW whose `rmw_take_loaned_message` is a NO-OP stub (rmw_zenoh) |

`bench.py smoke`:

| Code | Meaning |
|---|---|
| 0 | every gated cell's p50 in range |
| 1 | an env-contract violation refused before any cell ran: a contradictory `CER_BENCH_PACING` or `CER_BENCH_DMA_LOCK` export, or a malformed `CER_BENCH_PAYLOAD_SIZES` (Python's own code for an uncaught `SystemExit(<message>)` rather than a chosen one, which is why it sits outside the deliberate 2/3/4 series). What `cmd_smoke` runs ahead of the baseline lookup is the **validation** of those three (parsing only); their consequences (the posture banner, the deliverable refusal, the curated-subset refusal) sit BELOW the no-baseline classification, since they gate the quality of a measurement a no-baseline run never takes. So a MALFORMED value is 1 on every path, while a WELL-FORMED restrictive `CER_BENCH_PAYLOAD_SIZES=64` is 4 with no baseline on file and 3 with one. That distinction is load-bearing: 4 is the code `run_benchmarks.sh` deliberately SWALLOWS, so answering a typo with 4 would silently continue a run instead of aborting it; listed because it is reachable and a wrapper branching on the table would otherwise meet a code the table denies exists. **Deliberately NOT here**: `CER_BENCH_USAGE` and `CER_BENCH_NATIVE_TIMEOUT_S`. Both are read PER CELL inside `_run_smoke_cells`, whose `except SystemExit` converts them to **3**; on the no-baseline path no cell runs, so they are never consulted and the run is a **4**. Measured on the current code, malformed value each, on a host with NO baseline: `PACING` -> 1, `DMA_LOCK` -> 1, `PAYLOAD_SIZES` -> 1, `USAGE` -> 4, `NATIVE_TIMEOUT_S` -> 4; and a well-formed `PAYLOAD_SIZES=64` -> 4 there, 3 with a baseline on file |
| 2 | at least one `FAIL_HIGH` / `FAIL_LOW` |
| 3 | crash / build / setup failure, or a cell that RAN with no range in the baseline |
| 4 | no baseline for this host+variant: nothing to gate against. Four shapes reach it: the ranges file absent; `hosts: {}`; a shared file holding only OTHER hosts' entries (the designed steady state); and this host's own entry carrying no ranges for this variant. One code, four shapes, **different remedies**, so the gate names which it hit: only the third can be identity drift (the only shape where this host's key is absent), so only it warns about drift and prints the live governor/turbo posture, `machine_hash` covering kernel, governor and PREEMPT_RT; the fourth is a partial or hand-edited capture and says so. Separate from 3 so a wrapper can skip the gate without also tolerating a build failure |

## 10. Same-window A/B discipline

Absolute numbers on the same host drift day to day: ambient load,
thermals, kernel/firmware updates. Between same-host sweeps on different
days, the prior campaign observed drift on the order of ~12% with no
code change. Rules:

- **A/B comparisons are only valid same-window**: run A and B
  interleaved, in one session, on one host, from one build environment.
  "A from Tuesday vs B from Thursday" is not an A/B.
- Cross-day, cross-host, and cross-campaign comparisons are trend data
  only, and every such comparison must name the machine hash and date of
  both sides.
- On a degraded/suspect host, relative same-window A/Bs remain valid;
  absolute numbers do not; re-baseline before trusting absolutes.
- The smoke gate's `[p50/2, p50*2]` bounds are deliberately wide for the
  same reason: it is a catastrophe detector, not a drift detector (its
  baseline is captured as the median of three smoke reps, so a lucky
  single rep cannot set the bounds).

### Campaign rules: reps are the error bar

Within-run sampling error at this suite's n is small (well under a few
percent on a p50); **between-run environmental variation dominates it
by an order of magnitude**: rep-to-rep p50 wobble at the
tens-of-percent scale has been observed on µs-class lines with zero
code change (run-level state: core placement, thermals, frequency,
cache/page alignment). The §2 reliability table bounds ONLY the
within-run term; the between-run term is controlled by reps:

- **A published close comparison requires k ≥ 5 reps per cell**,
  collected round-robin: `bench.py full --reps N` runs the WHOLE
  matrix per rep and interleaves at rep granularity, which is what
  satisfies this section's interleaving rule at campaign scale (a
  single serialized sweep measures the native lines hours before the
  ROS 2 cells they face). Each rep lands in its own `rep<k>/`
  subdirectory of the run dir; **reps accumulate, never overwrite**.
- **The headline number is the median of the per-rep p50s.**
  `compile_csv.py` aggregates across reps and carries the min-max rep
  spread beside the median; `plot.py` draws the median line with the
  rep spread as a shaded band.
- **A two-line comparison claim must exceed the measured rep spread of
  BOTH lines.** A delta inside either line's spread is noise, whatever
  any single rep says.
- The ≫-wobble comparisons (the order-of-magnitude native-vs-ROS 2
  gaps) remain readable from a single window, but then they are
  order-of-magnitude claims, never percentage claims. Which class a
  published claim belongs to is decided by the rep spread, not by the
  author.

## 11. Environment-variable contract

Names are shared with the predecessor trees where they existed there;
scripts written against the old suite keep working.

| Variable | Consumer | Meaning |
|---|---|---|
| `CER_BENCH_RAW_DUMP_DIR` | all binaries | **Required.** Directory for raw `.bin` dumps. Fail-fast if unset. |
| `CER_BENCH_RAW_NAME` | all binaries | **Required.** The cell's `raw_prefix` (see README line inventory); the `.bin` stem. Fail-fast if unset. |
| `CER_BENCH_PACING` | native binaries, runners | `quiescent` (default), `fixed100` (uniform 100 Hz + fallback ladder; §1, §17), or `backtoback`. `bench.py --variant` sets this. Under fixed100 every runner also writes the `<cell>_<size>.rate` achieved-rate sidecar beside the `.bin` (first line: integer Hz, or `did_not_sustain`). |
| `CER_BENCH_FORCE_RATE_HZ` | workspace runner | Quiescent-only DIAGNOSTIC: overrides the schedule rate for every size while keeping counts/gates: how the diagnostic rate-vs-payload flatness matrices were measured (§17). For a first-class uniform-rate sweep use `--variant fixed100`; this knob labels nothing downstream. Must be a decimal integer in `1..1000000000` with NO leading zeros; the runner refuses anything else. A non-numeric value is an unattributed bash arithmetic error; above 1 GHz the wall period floors to 0 ns (see `CER_BENCH_TARGET_RATE_HZ`); and `010` would be read as OCTAL 8 by the runner's arithmetic while the node's parse reads decimal 10, pacing the two ends differently under one label. Quiescent-only: under `fixed100`/`backtoback` the value is validated but never consumed, and the runner says so. **A forced-rate run is a DIAGNOSTIC figure, never a citable one:** the knob is recorded in no CSV column, `.rate` sidecar or `run.json` field, so post-processing cannot see it; the beneath-axis rate braces are derived from the variant's pinned schedule and therefore state what the sweep TARGETED (every label says `target`), which under this knob is not what it paced. The ROS 2 side has the same shape in `CER_BENCH_TARGET_RATE_HZ`: an AMBIENT value is the highest override precedence for every payload in `run_bench.sh` and is likewise invisible to post-processing (`bench.py` can leak neither knob; it passes `-e KEY=VALUE` only, and only on the fixed100 ladder, so both are hand-run exposures). Plot such a run for the flatness question it was run to answer, not as a rate claim. |
| `CER_BENCH_SMOKE_N` | binaries | Smoke override of (total, warmup) sample counts, keeping the pacing mode. |
| `CER_BENCH_TARGET_SAMPLES` / `CER_BENCH_WARMUP` | binaries, workspace nodes, ROS 2 nodes | **`TARGET_SAMPLES` is the MEASURED count in every component** (§2). Back-to-back defaults: 10000 measured / 1000 warmup. On quiescent legs the runners derive measured = total − warmup from the schedule and export it (`bench.py` → ROS 2 containers + the workspace runner; `run_bench.sh` / `run_workspace.sh` derive the same when driven directly); every `.bin` sample-count gate checks the measured count. |
| `CER_BENCH_TARGET_RATE_HZ` | ROS 2 nodes + workspace ping node | Per-payload publish rate. ROS 2: one payload per container invocation; `run_bench.sh` sets it from the schedule. Workspace: the ping node's wall-clock slot-grid publish gate, the pacing AUTHORITY on both legs (the `period_ms` attr is only the tick source; on the split leg it is logical time under the handed-quantum lockstep; see §2 "tick source vs pacing authority"). `0`/absent = ungated (backtoback). Ceiling 1 GHz, enforced by all five pacers (the three ROS 2 nodes, the native `RateLimiter`, the workspace ping node): every one derives its period as `1e9 / rate_hz` in integer arithmetic, so a higher rate floors it to 0 ns and stops gating while the run keeps the requested rate as its label. |
| `CER_BENCH_QOS` | ROS 2 nodes | `be1` (default) or `rel10` (§8). ROS 2 side only. |
| `CER_BENCH_MSG` | ROS 2 driver + nodes, workspace runner | The TYPE-CLASS axis (§18). ROS 2: `pod` (default) \| `image`; workspace: `variable` (default) \| `pod`; each side's default is its incumbent class, so an env-less invocation is byte-identical to the pre-axis suite. |
| `CER_BENCH_POD_BYTES` | workspace pod node build scripts | Bakes the `PodPayload` fixed-array length at build time (§18); exported per size by `run_workspace.sh`. Must be a multiple of 4 (the `#[repr(C)]` struct's u32 alignment rounds any other total up: 65 would bake 68); the build script refuses anything else and the runner refuses a misaligned `CER_BENCH_PAYLOAD_SIZES` before building. The pod nodes' init verifies baked-vs-runtime size: loud Err on a stale build, never a mislabel. |
| `CER_BENCH_PAYLOAD_SIZE` / `CER_BENCH_PAYLOAD_SIZES` | binaries, runners | Payload selection override (single size / sweep list). Both shell runners (`workspace/run_workspace.sh`, `ros2/run_bench.sh` via `SIZES`) enforce one contract on every token before any arithmetic touches it: decimal digits only (`0x40`, `1e3`, a sign are refused); at most 10 digits: bash arithmetic is 64-bit and `10#` wraps silently (`18446744073709551680` would read as 64), so the length is bounded first; then the surviving value must lie in 1..16777216, the pinned sweep's ceiling (schedule + SHM provisioning end there); a zero-padded token is read as decimal with a loud note; and a value that word-splits to NOTHING is refused rather than run as a zero-size sweep that would exit 0 having measured nothing (an EMPTY value is different; it reads as "no override" and runs the full sweep). Whatever survives must be one of the **ten pinned sweep sizes**, on BOTH runners and for BOTH type classes: an off-sweep size takes `schedule_for`'s `*)` fallback and would be measured under a schedule no other stack shares, so it is never comparable even where it is realizable. On top of that, each runner adds what its own stack can BUILD. Workspace: the `PodPayload` type is generated per size (`CER_BENCH_POD_BYTES`), so `run_workspace.sh` refuses a pod size that is not a multiple of 4 (the `#[repr(C)]` u32 layout rounds anything else up; §18); that gate runs first and its refusal names the pinned set too, so a misaligned size gets one complete answer; the variable class (`sensor_msgs/Image`, `data` loaned per tick) has no build-side constraint and is bounded by the pinned set alone. ROS 2: `Pod<N>` is a FIXED set of ten generated `.msg` types (`Pod64` … `Pod16777216`, dispatched at runtime by `pod_dispatch.hpp`), and the single `sensor_msgs/Image` instantiation validates the sweep point explicitly (`msg_class_dispatch.hpp::valid_sweep_size`), so `run_bench.sh` refuses any other size before ROS is sourced rather than leaving it to the in-container dispatch error. |
| `CER_BENCH_WALL_STAMP` | workspace runner (parity) | Exported as `1` on every workspace leg for env-contract parity. This suite's nodes stamp wall time unconditionally and do not consult it (§4); nodes built as record and replay assets DO gate on it, and collect zero samples without it. |
| `CER_BENCH_LEG` | workspace nodes | Leg tag (`split` \| `mono`) set by `run_workspace.sh`; rides the delivery-accounting lines so per-leg accounting is attributable in a shared supervisor log. |
| `CER_BENCH_DMA_LOCK` | ALL three stacks (bench.py, workspace runner, native bins) | `1` (default) = tuned C-state posture: the workspace runner exports `CERULION_CPU_DMA_LOCK=1` so the graph process holds the cap (§7), native bins acquire the lock, and bench.py passes `/dev/cpu_dma_latency` into ros2 containers (`--device`). `0` = **stock posture** (extended from the workspace-only knob): NO bench-side machine tuning anywhere; the workspace var is UNSET for the graph process (`env -u`, so an inherited `CERULION_CPU_DMA_LOCK=1` is REMOVED, not merely not-added; the graph runs the product's flagless `Auto` posture), native bins SKIP the lock (`lib.rs::acquire_dma_lock` reads the same var; distinct from `CER_BENCH_ALLOW_NO_DMA_LOCK`, which only tolerates a FAILED acquisition), and ros2 containers run WITHOUT the device (the in-container bins print their loud no-device note and run uncapped). Announced loudly at start; recorded as `dma_lock_posture` in the run manifest; every entry point that can produce a mislabeled row REFUSES the contradiction it can see, so a DIRECT run honours the contract too: `bench.py::dma_lock_enabled`, `run_workspace.sh`'s posture case and `native/src/lib.rs::acquire_dma_lock` each refuse a contradictory ambient `CERULION_CPU_DMA_LOCK` export (the workspace runner additionally passes `env -u CERULION_CPU_DMA_LOCK`, so stock mode is true by construction, not merely by the check having run), and `ros2/run_bench.sh` refuses the ROS 2 lane's OWN contradiction, `CER_BENCH_DMA_LOCK=0` while `/dev/cpu_dma_latency` is visible, which the C++ nodes would open unconditionally, capping a run labelled stock. (`CERULION_CPU_DMA_LOCK` is the Cerulion runtime's var; the ROS 2 nodes never read it, so refusing it there would enforce a contract that lane does not have.) Any other value = env-contract violation. The CODE depends on where the value is read: `cmd_smoke` gates `CER_BENCH_PACING`/`CER_BENCH_DMA_LOCK`/`CER_BENCH_PAYLOAD_SIZES` before the baseline lookup and exits **1**, while `CER_BENCH_USAGE`/`CER_BENCH_NATIVE_TIMEOUT_S` are read per cell and become **3** (or never fire at all, leaving a **4**, if no cell runs). The sweep subcommands catch none of them and exit **1**. See the smoke exit table. Stock rows and capped rows must never be mixed in one comparison without saying so. |
| `CERULION_CPU_DMA_LOCK` | `cerulion` CLI | `1` = the CLI acquires the DMA latency lock (§7). Native binaries call `cpu_dma_lock()` directly. |
| `CERULION_NETWORK` | `cerulion` CLI | **Deliberately NOT set by the workspace runner**: both legs run the product's default permissive posture; the gateway spawns, parks at zero demand, and sits outside the measured SHM chain. Setting `off` would measure a shape no flagless user gets (README § "Run shape and network posture"). |
| `CER_BENCH_DUMP_VERSIONS` | ROS 2 driver | `1` (set by `bench.py`) = `run_bench.sh` dumps the container's installed ros/rmw/dds/iceoryx package versions (`dpkg -l`) once per cell to `_logs/<cell>_versions.txt`, the compared-stack version record beside the run manifest (§15). |
| `ROS_DISABLE_LOANED_MESSAGES` | ROS 2 `loan` cells | Exported `=0` on the `loan` recv lane ONLY: rcl's subscription-side loan gate defaults OFF upstream (rclcpp#2335 / rcl#1110), and the lane exists to measure the loaned take, so its capability logging must describe the lane that ran. The `rclcpp` lane leaves it untouched: upstream's shipped default (§15). |
| `BENCH_CELL_TIMEOUT_S` | ROS 2 driver | Per-cell wall-clock bound (in-script default 300; `bench.py` exports a schedule-scaled value, 2× the nominal pacing window + 60 s, floored at 300, since the tail-resolved 16 MB window is ~205 s nominal). |
| `CER_BENCH_FASTDDS_PROFILE` | `verify_shm.sh` | Absolute path of the Fast DDS profiles XML to VERIFY, overriding the derived `configs/fastdds_<mode>.xml`. `run_bench.sh` sets it so the 16 MB cell's `fastdds_shm_16mb.xml` is verified with the same profile it will measure with; a one-time preflight against `fastdds_shm.xml` left that profile unverified. Ignored outside the fastdds non-`zc` arm (cyclonedds/zenoh/`zc` select their own config); a path that does not exist is refused (exit 2). |
| `IOX_ROUDI_LOG` | `run_bench.sh`, `verify_shm.sh` | Path of iox-roudi's `-l debug` log, the ONLY evidence the cyclonedds SHM oracle reads. `run_bench.sh` sets and exports it for a RouDi it starts. **Required when RouDi is already running externally**: only its owner can capture that log, so `start_iox_roudi` refuses (exit 2) unless the caller started it as `iox-roudi -l debug > LOG 2>&1` and exported `IOX_ROUDI_LOG=LOG`. |
| `CER_BENCH_ALLOW_UNVERIFIED_SHM` | ROS 2 driver | `1` = a failed `verify_shm.sh` no longer hard-fails the cell; the rows are recorded with a loud `_logs/<cell>_<size>_SHM_UNVERIFIED` marker + stderr warnings (§9). Default: hard fail. |
| `CER_BENCH_ALLOW_NO_DMA_LOCK` | native binaries | `1` = run without the C-state lock when `/dev/cpu_dma_latency` is unwritable; `cpu_dma_lock: SKIPPED` lands in the runner log and the run's p99/p99.9 are not citable (§7). Default: refuse to run. |
| `CER_BENCH_NATIVE_TIMEOUT_S` | `bench.py` | Watchdog on every native bench subprocess (defaults: 1800 quiescent / 900 backtoback; scaled down under smoke). On expiry the process group is killed (plus a best-effort pkill of the zenoh pong) and the bin is marked FAILED loudly. |
| `CER_BENCH_USAGE` | `bench.py`, workspace runner | `0` (default) = off, byte-identical run shape to a pre-usage tree. `1` = per-cell CPU + memory usage sidecars (`<cell>_<size>.usage.csv` beside the `.bin`; `usage_sampler.py`); see § "Usage sidecars" below. Linux-only (`/proc`); on other hosts a loud note and NO sidecar (absent, never fabricated). Any other value = env-contract violation. The CODE depends on where the value is read: `cmd_smoke` gates `CER_BENCH_PACING`/`CER_BENCH_DMA_LOCK`/`CER_BENCH_PAYLOAD_SIZES` before the baseline lookup and exits **1**, while `CER_BENCH_USAGE`/`CER_BENCH_NATIVE_TIMEOUT_S` are read per cell and become **3** (or never fire at all, leaving a **4**, if no cell runs). The sweep subcommands catch none of them and exit **1**. See the smoke exit table. Under `1` the native sweep is split into one invocation per payload size for per-size attribution. |

Every sweep subcommand (`native`, `workspace`, `ros2`, `full`) and the
`compile-csv` post-processing step REFUSE to run when they detect ambient
`CER_BENCH_SMOKE_N` (or, for back-to-back, `CER_BENCH_TARGET_SAMPLES` /
`CER_BENCH_WARMUP`) in the environment: a sweep measured under a stray
override is silently low-fidelity, and a CSV validated under one reads its
expected counts from the post-processing shell instead of from the run.
`smoke` is the one path that does NOT refuse: it injects those overrides
itself, per-subprocess, and clears the one that does not apply to its
variant, so a baseline can never be captured under a foreign budget.

### Usage sidecars (`CER_BENCH_USAGE=1`): CPU + memory companion data

Opt-in, default OFF. `usage_sampler.py` samples each cell's process set
from the HOST's `/proc`, `utime+stime` deltas from `/proc/<pid>/stat`
at 5 Hz (→ `cpu_pct`, % of one core), `VmRSS` from `/proc/<pid>/status`
at 5 Hz, and PSS from `/proc/<pid>/smaps_rollup` at 1 Hz (it walks the
VMA list and is pricier), into `<cell>_<size>.usage.csv` beside the
`.bin` (`ts_ns,pid,comm,cpu_pct,rss_kb,pss_kb`). Native and workspace
cells are sampled by descending the spawned process tree (workers,
supervisor, gateway, and the zenoh pong all appear as their own rows);
ROS 2 docker cells are sampled from the host via the container's init
PID (`docker inspect`), so `scope=procs` holds there for CPU/RSS.
**PSS is the accurate memory number for SHM-heavy cells**: iceoryx2
pools are mapped into every participant, so summed RSS double-counts
every shared page per process; both are recorded, labeled, so the
double-count is visible instead of silently quoted (plot_usage.py
draws PSS solid, RSS dashed). Unreadable values (e.g. `smaps_rollup`
of a root-owned chrt/sudo tree read by an unprivileged sampler) are
recorded as EMPTY and counted in the sidecar footer, never zeroed
(Principle #13). **A summed PSS is served only under provable FULL
coverage** (footer present with `pss_denied=0` and every PSS tick
covering every sampled process): a partially-denied tree summed only
its readable minority would fabricate a LOW memory number, so
plot_usage.py reports `pss=UNMEASURED` with the reason and shows the
cell's RSS, loudly labeled, instead (pinned by
`plot_usage.py --self-test`). **Docker-cell memory is CGROUP scope,
not per-process PSS**: ros2 containers run as the image default user
(root), so an unprivileged host sampler is `smaps_rollup`-denied for
the whole container tree, exactly the copy-based lanes where memory
matters. On a whole-tree-denied PSS tick the sampler falls back to the
container's cgroup `memory.stat` (anon+file, the container TOTAL,
written as labeled `scope=cgroup` rows and rendered as its own
series). A cgroup total and a native cell's per-process PSS are NOT
like-for-like: never compare them as one metric; the summary CSV
keeps them in separate columns (`pss_mib_mean` vs `cgroup_mib_mean`).
**Observer cost**: the sampler's own CPU is measured per run
(`os.times()`) and written to the sidecar's `# sampler_self:` footer, so
every usage artifact carries its own overhead record; quote a run's
cost from its own footers. For scale: on a kernel WITHOUT
`CONFIG_PROC_CHILDREN` (where descendant rescans walk all of `/proc`) the
sampler costs a few percent of ONE core at the default 5 Hz stat / 1 Hz
PSS+rescan cadence; rescanning on every tick cost several times that, which
is why the rescan is cached at 1 Hz. Latency numbers from a `CER_BENCH_USAGE=1` run are
companion numbers: the sampler is a real (if small) co-tenant load and
the native sweep shape differs (one invocation per size), so headline
latency rows come from usage-OFF runs.

## 12. What this suite deliberately does not measure

- **Throughput / messages-per-second capacity**: the back-to-back mode
  bounds saturation *latency*; it is not a bandwidth benchmark.
- **Concurrent multi-topic / loaded-system latency**: every cell is a
  single ping-pong flow on an otherwise-idle machine. A real robot
  runs tens of topics; contention effects are unmeasured here and are
  outside this suite's scope.
- **The prior `intra_*` lane shapes** (naive/forward): dropped and
  not resurrected. The intra-process AXIS itself returned as
  the `composed` lane (§16): intra-process bypasses the RMW entirely, so
  it answers a *framework*-overhead question, not a transport question;
  the composed rows carry that label, and the README pairing table pairs
  them with `mono` under the same caveat.
- **Cross-machine (network) latency**: every cell here is single-host
  SHM. This suite does not measure cross-machine Cerulion-vs-Cerulion lines.
- **Application fill cost**: excluded by construction (§3); measure your
  producer's fill separately if it matters to your budget.
- **Replay determinism**: a Cerulion-native property with its own gates
  in `cerulion_core`; nothing here makes or tests determinism claims, and
  per-run jitter is never called "determinism" (boundary discipline,
  README).

## 13. Coordinated omission: named, bounded, and what the artifacts cannot recover

**Coordinated omission** (the HdrHistogram term)
is the bias a load generator introduces when its next probe WAITS on
the previous one: the windows where latency was elevated contribute
FEWER samples than the schedule intended, so percentiles read low
exactly where the system was worst. This suite's quiescent percentiles
have that property on BOTH stacks, symmetrically, stated here so a
reviewer armed with the coordinated-omission critique finds it pre-answered rather than
discovered:

- The native loop serializes pacing and a blocking round trip: no
  probe is issued while an RTT stalls, so a stall window contributes
  at most one (long) sample.
- The ROS 2 kick pacing deliberately SKIPS missed slots (§9), and the
  measured-count gate keeps a cell running until the full sample count
  arrives, so backlogged intervals contribute fewer probes and the
  population is collected disproportionately from healthy intervals.
- The `.bin`s store **durations only, in chronological order** (no
  completion timestamps), so coordinated omission can be located in
  time (§5) but neither corrected nor precisely quantified post hoc.

Read every quiescent percentile as: **the distribution over
issued-and-completed probes under the schedule, with stall windows
under-represented.** Percentiles-per-schedule-slot (the HDR-style
corrected view) are NOT what the CSVs contain. The same property holds
for the upstream reference harnesses this suite mirrors
(Apex `performance_test`, zenoh's `z_ping`, iceoryx2's bench); the
delivery-accounting lines (§6: kicks sent, received counts, rate
deficits) are the per-cell record of how much the schedule drooped.

**The pacing policy is unified so the omission is symmetric** (§9):
skip-missed-slots everywhere; the Rust `RateLimiter` grid-anchors to
the next future epoch slot exactly as the ROS 2 timers do, so neither
stack bursts saturation samples into quiescent tails after a stall.

**The one residual asymmetry is the workspace Period trigger, which is
product code, documented, not forked.** Cerulion's `period_ms`
scheduler fires `floor(elapsed/period)` catch-up ticks after a stall (a
graph runtime must not silently drop scheduled work, and the harness
does not patch product semantics to flatter a bench). The wall-grid publish gate additionally bounds a catch-up burst at the
SOURCE: the burst's first tick consumes one slot, the rest publish
nothing, and the missed slots are counted (`slots_skipped`) instead of
burst-published. Two further things bound it. First, the data-trigger drain COALESCES a catch-up burst
(latest-wins), so burst frames become an accounted `received` deficit in
§6's lines, not extra samples. Second, the bias direction of the
samples that DO land immediately post-stall is stated plainly: they
run back-to-back, i.e. WITHOUT the sleep→wake cost quiescent mode
exists to include, so they read LOW, **in the workspace lines' own
favor**, which is precisely why it is named here rather than left for
a reviewer to find. It is rare on the idle, governor-pinned machine the
campaign requires (§7), and a stall large enough to matter is visible
as a rate deficit in the same leg's delivery accounting.

## 14. Environment boundary: containers vs bare host

Which lines run where is a named comparison axis, not an accident:

- **Containerized:** every ROS 2 cell runs in a fresh Docker container
  on the host kernel (SHM data plane; `--device /dev/cpu_dma_latency`,
  rtprio and memlock ulimits passed through; Docker's default
  seccomp/AppArmor confinement; a fresh 4 GiB container tmpfs; the
  container image's glibc).
- **Bare-host:** the native lines (`iox2` floor, `zenoh_shm`) and both
  workspace legs.

Two consequences, in decreasing strength:

1. **Every in-container head-to-head is environment-symmetric.** All
   ROS 2 cells of one distro run in the IDENTICAL container image
   (same kernel, seccomp profile, netns, tmpfs, libc, ulimits), so
   RMW-vs-RMW and lane-vs-lane comparisons never cross the boundary.
2. **Cross-boundary comparisons (workspace legs vs ROS 2 cells) share
   the host, the kernel, the DMA lock (PM-QoS is host-global; §7),
   the schedule, and the pacing policy, and differ in
   containerization.** The boundary's direction is named: Docker's
   default seccomp filter adds per-syscall cost to exactly the
   wake-path syscalls (futex, UDS send/recv, nanosleep) that quiescent
   mode deliberately measures, i.e. it inflates the CONTAINERIZED
   (ROS 2) side, at a scale of tens of ns per syscall against cells
   whose round trips are orders of magnitude larger. If a reviewer
   wants the container share isolated, the way to measure it is a
   one-cell host-vs-container A/B of a stock RMW in one window: a
   measurement, not a doc edit.

Known container-specific hazards from this suite's own history, RT
throttling under SCHED_FIFO (`PITFALLS.md` #7) and `RLIMIT_MEMLOCK`
(`PITFALLS.md` #12), are gated per cell, never assumed away.

## 15. ROS 2 configured at its best: the upstream record

Every "you hobbled ROS 2" attack this suite has been able to construct
or collect is answered here, with the upstream citation that decides
it. The frame: ROS 2 cells run **as shipped by ROS apt for each
distro** (stock binaries, stock defaults unless a cell's label says
otherwise), because self-built middleware would make the compared stack
numbers unreproducible from packages.ros.org and would open the
reverse unfairness claim.

### Executor choice: `rclcpp::spin` IS the shipped default

The rclcpp cells run the SingleThreadedExecutor because
`rclcpp::spin(node)` hardcodes it, in every distro through Lyrical's
rclcpp 32.x (`ros2/rclcpp` `executors.cpp`; no env override exists).
The events-executor family is opt-in, not default: Humble has NO
in-tree events executor (only the external `ros2-performance` package, since
deprecated); Jazzy ships `rclcpp::experimental::executors::
EventsExecutor` (experimental namespace; merged via ros2/rclcpp#2155);
Lyrical 32.0.0 ships a non-experimental `EventsCBGExecutor` whose own
release notes claim 10 to 15 % less CPU than the Single/MultiThreaded
executors; and post-Lyrical rolling (rclcpp 33.x) DEPRECATES the
experimental EventsExecutor. So the default-executor cells are "what
ROS 2 users get" by upstream's own hardcoded choice, and the suite
ALSO ships ROS 2's fastest sanctioned receive path as a first-class
lane: the `loan` cells are executor-LESS (`rclcpp::WaitSet` +
`rcl_take_loaned_message`, the same pattern as Apex
`performance_test`'s fastest rclcpp plugin, i.e. the reference tool's
own concession that the executor is overhead). An `EventsCBGExecutor`
variant on Lyrical is declined for this revision as future work: it is
in-release but not default, and the executor-less lane already
brackets what removing the executor buys.

### Subscription-side loans: OFF by default is upstream's decision, and the loan lane logs the truth

With ros2/rclcpp#2335 + ros2/rcl#1110 (backported to Humble
via rcl#1116), rcl DISABLES subscription-side loaned dispatch by
default for a documented safety reason (the middleware reuses the
loaned memory; a user-retained `ConstSharedPtr` would observe
corruption); `ROS_DISABLE_LOANED_MESSAGES=0` re-enables it.
Publisher-side loans default ON. Therefore:

- the `rclcpp` cells leave the env UNTOUCHED: the per-receive memcpy
  they measure is upstream's shipped default, not this harness's
  choice;
- the `loan` cells export `ROS_DISABLE_LOANED_MESSAGES=0`, because
  that lane exists to measure the loaned take, and with the env unset
  rcl's gate makes the `can_loan_messages()` capability probe read
  `false` even where the rcl-direct take path works; the published
  capability lines must describe the lane that actually ran.

### FastDDS: stock defaults AND the vendor-recommended zero-copy lane, both labeled

`rmw_fastrtps` forces `data_sharing().off()` on writer and reader in
its default path (on jazzy AND lyrical) unless
`RMW_FASTRTPS_USE_QOS_FROM_XML=1` (upstream `publisher.cpp` /
`subscription.cpp`; PR ros2/rmw_fastrtps#568 decoupled LOANS from
data-sharing, `can_loan_messages = is_plain` since Iron; it did NOT
remove the override). So the matrix carries both configurations,
labeled:

- the fastdds `shm` cells = **stock defaults**: the SHM *transport*
  engaged (loaned write → segment copy → reader-history copy),
  DataSharing off, exactly what an out-of-box ROS 2 user gets;
- the fastdds `zc` cells = **the vendor's own recipe** (the
  rmw_fastrtps README): publisher/subscriber default profiles carrying
  `<data_sharing><kind>AUTOMATIC</kind></data_sharing>` +
  `RMW_FASTRTPS_USE_QOS_FROM_XML=1`, Fast DDS's true zero-copy path,
  where the writer history IS the shared segment (no transport, no
  fragmentation, no flow controller).

Copy-count per lane, so loans are never conflated with zero-copy: the
SHM-transport lane copies at the transport and not at the API; the
DataSharing (`zc`) lane removes the transport copy. `loaned=1` in a
CSV row proves the API-level loan engaged, NOT a zero-copy data path
(on CycloneDDS > 0.10, `rmw_publish_loaned_message` forwards to a
plain publish; a `loaned=1` there proves even less).

### rmw_zenoh: the shipped config with one documented key, and no loan API exists to have disabled

The rmw_zenoh `shm` cells run the SHIPPED session config
(`DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`) with exactly one documented
change: `transport/shared_memory/enabled=true`, the README-blessed
switch (rmw_zenoh ships SHM disabled "until fully tested"). The
shipped SHM parameters are KEPT (48 MiB pool,
`message_size_threshold: 512`) because the vendor's README warns that
lowering the threshold "could be counter-productive for the latency of
small messages": under the shipped threshold, sub-512 B payloads ride
the network path by upstream's own choice, and that is upstream's
number to own, not this harness's to tune. (The NATIVE zenoh line is
different and labeled so: raw zenoh with threshold 0, forcing SHM at
every size, mirroring zenoh's own `z_ping_shm` methodology; see the
README line inventory.)

rmw_zenoh implements NO loaned-message API on any distro: the
LoanedMessage entry points return `RMW_RET_UNSUPPORTED` at both 0.2.10
(jazzy) and 0.10.5 (lyrical), tracked upstream as ros2/rmw_zenoh#175
and #893, open, no milestone. Every rmw_zenoh row, SHM included,
contains CDR serialization plus one copy into the SHM buffer BY
UPSTREAM DESIGN; there is no zenoh zero-copy for a harness to have
disabled. The `zenoh × loan` structural skip (rc = 77) is the same
fact at the cell level.

The `rmw_zenohd` router is the shipped default discovery topology
(multicast scouting ships disabled), and it is NOT on the data path
between same-host peers, "By default, Zenoh router doesn't forward
messages between peers" (rmw_zenoh 0.10.5 README), so running it
taxes zenoh's discovery, not its per-message latency, and NOT running
it would be a non-default deployment. Both distro rows ride the same
zenoh core: jazzy's rmw_zenoh 0.2.10 and lyrical's 0.10.5 vendor the
identical pinned zenoh-c 1.8 commit, so cross-distro zenoh deltas are
RMW-layer (0.10.x adds a buffer-aware publish fast path), never
zenoh-core.

### CycloneDDS: the deprecated `<SharedMemory>` alias IS the modern PSMX path

On lyrical (CycloneDDS 11.0.x), `convert_deprecated_sharedmemory()`
(`src/core/ddsi/src/ddsi_config.c`, releases/11.0.x) converts
`<SharedMemory><Enable>true</Enable>` into the IDENTICAL in-tree
`psmx_iox` instance the modern `<PubSubMessageExchange>` syntax
creates (same plugin, same defaults, Prefix → INSTANCE_NAME
`DDS_CYCLONE`), so the runtime configuration is byte-equivalent and
"you ran the other stack on a legacy path" is answered by upstream
source, not by assertion. The iceoryx2-based `psmx_iox2` plugin exists
in the 11.0.1 tree but is NOT shipped by ROS lyrical apt (iceoryx2-c
is not on the buildfarm; no rosdistro entry), out of scope by ROS's
own packaging choice, not this suite's.

### rosout / parameter services: stripped uniformly, disclosed with the real motivation

`bench_node_options()` disables `/rosout`, the parameter services, and
the `/parameter_events` publisher on EVERY bench node for EVERY rmw:
the machinery is off the measured path (no rosout traffic inside the
timed window; logging happens at startup/finalize), and the reference
methodology does the same (the `ros2-performance` harness's memory
benchmarks run `enable_rosout(false)`). Full disclosure, so it is read
here first rather than found in a source comment: the change was
originally MOTIVATED by `rmw_cerulion`'s publisher-slot limitation
(three stock nodes abort under rmw_cerulion with rosout +
parameter services on). That is this repo's product gap, tracked
openly; the stripping itself is retained as uniform bench hygiene that
helps every RMW, not an accommodation hidden in a config file.

### Declined lanes, with reasons

- **rmw_iceoryx / rmw_iceoryx2**: rmw_iceoryx's newest branch is
  `iron` at the 2026-08-13 evidence snapshot (no jazzy/kilted/lyrical branch); rmw_iceoryx2
  self-describes as alpha (events/services/graph/QoS unfinished, no
  released binaries). Neither is a released, supportable configuration
  for the distros under test.
- **rclcpp intra-process lanes**: originally declined here as "a
  different question than a transport comparison"; still true, and now
  measured anyway with exactly that label: the `composed`
  lane (§16) is the framework-overhead row, paired with `mono` in the
  README pairing table, never quoted as a transport number.
- **Self-built or patched middleware of any kind**: the "as apt ships
  it" frame above.

### Compared-stack versions are recorded, not archaeology

Per cell, the ROS 2 driver dumps the container's installed
ros/rmw/dds/iceoryx package versions to `_logs/<cell>_versions.txt`
(`CER_BENCH_DUMP_VERSIONS=1`, set by `bench.py`), and the run manifest
(`run.json`) records the docker image IDs, so which CycloneDDS /
Fast DDS / rmw_zenoh was actually benched is recoverable from any
committed run. The native zenoh crate is exact-pinned `=1.7.2` with
the rationale in `native/Cargo.toml` and the README line inventory.
Version anchors from the 2026-08-13 evidence snapshot (not a current-version claim): jazzy =
Fast DDS 2.14.x / CycloneDDS 0.10.5 / rmw_zenoh 0.2.10; lyrical =
Fast DDS 3.6.2 / CycloneDDS 11.0.1 / rmw_zenoh 0.10.5.

### Alignment with reference methodologies

`performance_test` parity: `/dev/cpu_dma_latency` written 0
and held open (identical mechanism); fixed-size POD message types;
warmup-discard; zero-copy as an opt-in labeled axis; QoS as an
explicit labeled axis; and its fastest rclcpp plugin is the
executor-less `rclcpp::WaitSet` pattern this suite's `loan` lane
uses. `ros2-performance` precedent: `enable_rosout(false)` and
an explicit executor switch. Where this suite differs (offline
percentiles from committed raw samples, hard SHM-engagement gates,
delivery accounting as a first-class artifact, tail-percentile
suppression), it differs in the stricter direction.

## 16. The usage-pattern lanes: stock and composed

Evidence base: `ros2/memo.md` (upstream PR/doc reads +
GitHub-wide code search, every claim linked there). The memo's finding
that shapes both lanes: the matrix's configured cells bracket what ROS 2
*can* do, while what ROS 2 users *actually run* is (a) the zero-config
default (the slowest common shape) and (b) composition with optional
intra-process comms (the fastest common shape). Publishing both, beside
the configured matrix, is a strict superset of what the published ROS 2 benchmark
harnesses report (memo §4) and closes the cherry-picking attack from
both directions.

### The `stock` lane (slowest-common: the zero-config default)

`{distro}_stock_rclcpp_chrt{N}`, what `ros2 run` gives you on
jazzy/kilted/lyrical, with zero configuration (memo §3, each row cited
there): one process per node (3 processes), `rmw_fastrtps_cpp` (the
default rmw), Fast DDS default transports (UDPv4 + the builtin
copy-based SHM transport; the SHM *transport* is a default, and it is
NOT zero-copy), DataSharing OFF at the rmw layer, synchronous publish,
`qos=stock` = `rmw_qos_profile_default` (RELIABLE / VOLATILE /
KEEP_LAST(10); `rclcpp::QoS(rclcpp::KeepLast(10))` with no modifier
calls, so it cannot drift from the profile), plain `publish()` +
typed-callback receive, subscription-side loan dispatch at its shipped
default (disabled; rclcpp#2335 / rcl#1110).

Lane rules, enforced loudly by `run_bench.sh`:

- **No transport claim, no `verify_shm` gate.** The matrix's SHM gate
  exists because those cells' LABELS claim engagement. The stock cell's
  claim is "the defaults, whatever they do"; there is nothing to
  verify, and gating it would quietly turn the lane into a configured
  one. Each cell gets `_logs/<cell>_provenance.txt` instead, "stock
  (fastdds defaults: UDP+builtin SHM transport, datasharing off)" with
  the memo citations, so the absence of a gate is itself documented.
- **Zero-config is asserted, not assumed**: any inherited
  `FASTRTPS_DEFAULT_PROFILES_FILE` / `FASTDDS_DEFAULT_PROFILES_FILE` /
  `RMW_FASTRTPS_USE_QOS_FROM_XML` / `CYCLONEDDS_URI` / `ZENOH_*` config
  env is scrubbed with a loud warning before the cell runs.
- The qos label is `stock`, not `rel10`, although the numbers coincide
  by construction: the cell name must say "the default profile was
  requested", which is a different claim from "a pin happened to match
  the default". The `be1` matrix cells remain *friendlier to ROS 2 than
  its own default* (memo §3's bench-fairness note); the stock lane is
  where the out-of-box claim lives.

### The `composed` lane (fastest-common: composition + the IPC opt-in)

`{distro}_composed_ipc{on,off}_rclcpp_chrt{N}`, the memo-§2 pattern:
composition is mainstream (Nav2 composed-by-default since Humble, nav2
PR #2750; Autoware composes 49 launch files; realsense/image_pipeline
compose), while `use_intra_process_comms` is OFF by default even inside
a container (rclcpp `node_options.hpp`) and Nav2 only gained the opt-in
in Kilted→Lyrical (PR #5804, off by default). Both shades are therefore
cells: `ipcoff` is the composed-but-default shade (every hop still
round-trips the rmw, in-process), `ipcon` is the fastest-common ceiling
(rclcpp IntraProcessManager pointer-pass; the rmw is bypassed on the
data path).

Shape: ONE process, the same three roles (ping / pong / latency, same
topics, same pacing contract) manually composed onto one
`SingleThreadedExecutor` in `composed_rtt_node`: manual composition
rather than a component container because the container adds launch
infrastructure outside the measured window without changing the data
path (`ros2-performance` measures the same single-process shape).
`qos=stock` (common usage runs the default profile).

Discipline deltas vs the 3-process lane, named rather than hidden:

- **Publish gesture is the pattern's own.** Intra-process pub/sub
  transfers ownership: `publish(std::unique_ptr)` is the documented
  0-copy gesture (design.ros2.org; 0 copies only because our callbacks
  take `ConstSharedPtr` and never claim ownership). A message therefore
  cannot be preallocated once and reused (Mode-A's shape in the
  3-process lane); each iteration allocates a fresh message with rosidl
  `MessageInitialization::SKIP`. SKIP keeps the O(N) payload zero-fill
  OUT of the timed window (the property Mode-A exists for) while the
  per-message heap allocation stays IN it, deliberately: allocation per
  publish is a structural cost of the intra-process ownership model
  that every composed+IPC user pays. Both IPC shades use the same
  gesture, so the ipc{on,off} delta isolates `use_intra_process_comms`
  alone.
- **No loan call sites** (`loaned=0` is printed as a fact of the
  binary, not a probe result): memo §1; loan callers in the wild are
  benchmarks and vendor SDKs, and this lane models the pattern as
  actually written.
- **Whole-chain bootstrap**: everything is in-process, so the first
  kick waits for kick AND ping AND echo to each have ≥1 matched
  subscription (inter- or intra-process count), load-bearing under
  backtoback pacing, where the bootstrap kick is the only driver.
- **What a composed row may be quoted as**: framework overhead, never
  transport. With ipc=on there is no middleware in the measured data
  path; the README pairing table pairs `composed` with `mono` and
  carries the asymmetry text (Cerulion's `mono` keeps its full SHM
  transport + observability plane in the path).

### Distro coverage

`jazzy` lane cells are first-class (swept before any citation).
`humble`/`lyrical` lane cells are enumerated so the matrix shape is
stable, and every run of one prints `! UNVERIFIED lane cell ...` until
a sweep has actually exercised them; a first failure there is an
unswept-distro finding, not a harness regression.

## 17. The rate axis: why a uniform-rate variant exists

Publish RATE is its own latency axis, independent of payload size, and
the suite treats it explicitly rather than letting it ride the payload
sweep. The summary of the evidence and the field survey:

- **The idleness tax is real and rate-proportional.** The diagnostic
  rate-vs-payload flatness matrices (measured with the workspace
  runner's `CER_BENCH_FORCE_RATE_HZ` diagnostic on the bench machines; that
  evidence is not part of a published package, so no figure from it is quoted here)
  put the same mono leg at a p50 that climbs steadily as the rate falls
  from 1 kHz to 100 Hz to 10 Hz: a large swing from the rate axis alone,
  payload held fixed. A
  size-dependent schedule therefore CONFLATES payload-flatness with
  idleness tax: under the sensor-rate schedule the 16 MB row runs at
  10 Hz and the 64 B row at 1 kHz, so a payload sweep's shape carries
  both effects at once.
- **The tax is not shieldable co-tenancy warmth.** The shielding
  experiment held a CPU cluster provably at max
  clock with a busy neighbor and moved p50 by ZERO: the cost is
  per-wake and thread-local, so no co-tenancy trick removes it. The
  practical consequence: pick the rate deliberately and hold it
  CONSTANT, which turns the tax into a constant offset across sizes.
- **The tax was localized by a diagnostic ladder.**
  One part of it
  is the graph process's OWN tick/wake/executor path (recovered by
  in-process 1 kHz ticks that publish nothing, while an external
  same-core 1 kHz waker recovers none of it), and the rest is
  the message path's own instruction/data/branch/TLB footprint evicted
  during the quiet gap: monotone in gap length from 1 ms to 500 ms
  with no threshold, accelerating at the longest gaps, paid on every
  wake on BOTH hops of the RTT, and insensitive to CPU-freq pinning,
  EMC locking, core pinning, and SHM-slice size (each a small rider;
  measured under the DMA-capped posture, and stock long-gap behavior was
  not swept). It is a per-wake platform tax, not product overhead,
  which is why the suite holds rate constant rather than trying to
  engineer it away. The shipped fixed100 packages are the
  demonstration: the single-process row is payload-flat from 64 B to
  16 MiB, every size at an achieved 100 Hz (`docs/benchmarks/results/`).
- **The field convention for a payload sweep is one uniform rate.**
  Findings doc Part 2: every published cross-payload, cross-middleware
  comparison (the `performance_test` school, the Fast DDS vendor's own
  1 KB → 4 MB campaign at a fixed 100 Hz, Maruyama's 256 B → 4 MB at
  10 Hz) holds frequency constant across sizes or crosses rate × size
  as explicit axes; none silently scales rate with payload. 100 Hz is
  the field-modal choice, hence `fixed100`.
- **Sensor-rate stays the realism variant, and the ladder's floor.**
  The quiescent schedule remains the primary suite (it answers the
  production question: a robot's topics really do tick at 1 kHz → 10 Hz
  by payload class), and its per-size rate is the LAST rung of the
  fixed100 fallback ladder where it sits below 20 Hz; a size that
  cannot hold 100 Hz degrades toward the rate the realism variant
  already measures, never below it.
- **100 Hz × 16 MB is a real feasibility edge, and the suite says so
  plainly.** 1.6 GiB/s of payload bandwidth (× 3 to 5 traversals on a
  copying transport) is where the empirical record shows collapse.
  Zero-copy legs are
  indifferent; a ROS 2 cell that cannot sustain it steps down the
  ladder with the achieved rate recorded and annotated, and one that
  exhausts the ladder renders **"did not sustain"** (a real product
  contrast), never a fake latency (§1).
  One semantics note the per-sample traces made explicit:
  the publish grid is OPEN-LOOP (the pacer does not wait for
  the previous round trip), so a copying transport can genuinely
  sustain the 100 Hz publish rate at 16 MB while per-sample RTT reads
  47 to 175 ms, i.e. ~10 round trips in flight at once (observed on the
  stock ROS 2 16 MB cell). An achieved-rate sidecar is therefore a
  publish-rate claim, not a closed-loop-latency claim; the latency
  samples themselves remain valid, and the two must not be conflated
  when reading a cell that "sustained" its rate with RTT far above the
  publish period.

## 17b. A bench recording is EVIDENCE, not a replay oracle

`graph run --record` on a workspace leg produces a bag that is
**replay-grade**: every channel carries a schema name and a definition a
reader can obtain, so the bag renders and decodes on a machine that never
compiled these nodes. That is the only claim the phrase makes.

It does **not** mean the run re-simulates. `cerulion bag play <bag>
--resim all --verify` exits **6** (structural trace divergence) on a bench
recording, and it does so **by design**: these nodes are deliberately
non-deterministic. The ping node reads `real_ns()` to stamp each frame and
to decide, against a wall-clock slot grid, whether a tick publishes at all
(`ping_node`'s own header: "Bench-only non-determinism … the framework's
documented escape-hatch set for latency benches — wrong for production
nodes"). A re-execution therefore reads a different clock, gates a
different set of ticks, and stamps different bytes.

MEASURED, both classes, one sitting, 64 B, this tree's own binary:

| leg | `--record` verdict | `bag play --resim all --verify` |
|---|---|---|
| `rtt_bench_pod` (fixed-POD) | is REPLAY-GRADE | **exit 6**: edge-read divergence at step 1 on `pong.ping_in` and `latency.echo_in` |
| `rtt_bench` (variable, the incumbent leg) | is REPLAY-GRADE | **exit 6**: the same two edges, the same step |

The two classes behave identically, which is the point: this is a property
of every bench leg, not of the type-class axis. A bench bag is for
inspecting what a run carried (frames, schemas, delivery accounting) and
for feeding a decoder. **It is not a determinism oracle, and a non-zero
`--resim --verify` on one is the expected result, not a defect.** The
suite's determinism claims live where they always have: the per-size
`.bin` sample files and the parity harness's hand oracles.

## 18. The type-class axis (pod | variable)

Every cell in the original matrix carries a FIXED-size POD message
(ros2_rtt_msgs `Pod<N>`; native `[u8; N]` slices), the shape that lets
loan-capable transports engage. Real robot topics are dominated by the
OTHER shape: unbounded types (`sensor_msgs/Image`, point clouds,
variable arrays), which no RMW can loan. The type-class axis adds a
labeled VARIABLE-message twin beside the POD cells so the suite can
test the hypothesis directly instead of implying it:

> **Hypothesis.** Cerulion's variable path stays within a few % of its
> POD legs: a variable field is written (or here: loaned, never
> written) directly into the loaned SHM slot, so the class changes the
> write gesture, not the transport mechanism. ROS 2's variable cells
> degrade at 256 KiB+: an unbounded type is non-plain, so every RMW is
> structurally forced out of its zero-copy lanes into full serialize +
> delivery memcpy, and that cost is O(payload).

No number appears in this section until the cells have been measured (Principle #13);
the hypothesis is stated so the sweep can confirm or refute it.

### The two classes, per stack

| Stack | pod class (incumbent, names unchanged) | variable class (new) |
|---|---|---|
| Cerulion workspace | **NEW**: `PodPayload`, a purely-fixed workspace schema (`uint32 prep + uint32 stamp_hi + uint32 stamp_lo + uint8[N-12] data`), generated per sweep size by `workspace/pod_schema/pod_codegen.rs` (`CER_BENCH_POD_BYTES` bakes the array length; the nodes' `init` verifies baked-vs-runtime size: a stale build errs loudly, never mislabels). Rows: `cerulion_workspace_{split,mono}_pod_chrt{N}`. | **The incumbent legs**: the shipped workspace nodes already ride `sensor_msgs/Image` with the unbounded `data` field loaned per tick (`loan_data(N)`), so the pinned token-less prefixes (`cerulion_workspace_{split,mono}_chrt{N}`) ARE the variable class. They are unchanged byte-for-byte; only their labeling now names the class. |
| ROS 2 cells | The incumbent `Pod<N>` matrix (all lanes). | **NEW**: `sensor_msgs/msg/Image` (the real type) with `data` resized to the sweep point at prealloc; cells `{distro}_{rmw}_shm_image_rclcpp_be1_chrt{N}`, matrix rmws × shm × rclcpp × be1 only. |
| Native lines | `iox2` floor + `zenoh_shm`: raw byte slices, pod-class by construction; the floor anchor. No variable twin (they exist to anchor the transport floor, not to model a typed schema layer). | none |

### Matched-quantity rule

The two classes are matched PER SWEEP POINT, with the matched quantity
stated per class (never silently equated):

- **pod**: the message totals exactly N bytes (`Pod<N>` = `uint64
  ts_ns + uint8[N-8]`; `PodPayload` = 12 fixed bytes + `uint8[N-12]`).
- **variable**: the unbounded `data` array carries exactly N bytes; the
  type's OTHER fields ride as a small constant overhead (ROS 2 Image:
  ~60 B of CDR-encoded header/encoding/geometry fields; Cerulion
  Image: the fixed section + offset table). The overhead is negligible
  at the 256 KiB+ sizes where the class hypothesis bites and disclosed
  here for the small ones: a 64 B variable cell genuinely moves ~2× a
  64 B pod cell's bytes, so cross-class deltas at the smallest sizes
  are read with that in mind.

This matches the incumbent conventions on both sides (the workspace
Image legs have always loaned `data = N`; `Pod<N>` has always totaled
N), which is why neither class's incumbent rows changed meaning.

### Structural exclusions (the hypothesis as skip inventory)

The variable class CANNOT ride the zero-copy lanes; that fact is the
hypothesis, so it is recorded as loud structural skips, not silent
absences:

- `image × loan` (exit-77 structural skip): `can_loan_messages()` gates
  on `is_plain` on every RMW that implements loans, so a loan-take lane
  for an unbounded type is empty by construction. The image class's
  receive cost IS the rclcpp lane's delivery memcpy.
- `image × zc` (exit-77 structural skip): FastDDS DataSharing requires
  plain, bounded types.
- `image × {stock, composed}`, not enumerated (exit 2 in the driver):
  the usage lanes are pod-only; a lane cell name carries no msg token.
- `image × no_shm`, `image × rel10`, not enumerated (exit 2 in the
  driver): MEASURABLE for an unbounded type (nothing structural forbids
  them, hence a setup error, not a 77 skip), but the image class pins
  shm × be1 (the rel10 pin was a pod SHM head-to-head; no_shm answers
  no type-class question), so no inventory names such a cell and a run
  would be an unenumerated measurement. The driver refuses rather than
  mints it; widening the class is an enumeration change first
  (`bench.py enumerate_ros2_cells` + `plot.py`'s image regex + the
  README grammar). The gates run before ROS is sourced, so
  `check_percentile_parity.py` drives every arm (refusals AND the
  enumerated shape passing) through the real script on any host.
- Publisher-side loan on an image cell is forced off in the nodes even
  if a future RMW claims support (a borrowed Image would publish an
  EMPTY data vector under an N-byte label, a mislabeled cell; such a
  claim would deserve its own lane). The per-row `loaned=` line records
  the post-override truth.
- The `is_plain` smoke gate INVERTS per class (`run_bench.sh
  check_is_plain`): pod cells fail on `is_plain: 0`, image cells fail
  on `is_plain: 1`; each class fails when its label lies. It also
  fails when the class's own marker is ABSENT and the run otherwise
  succeeded: a log that says NOTHING is not a log that says the right
  thing, so the assertion is POSITIVE, not merely "refuse the wrong
  marker". (A stale image is NOT the example: `is_plain_check`
  predates this axis, so such a binary logs `is_plain: 1` and the
  negative arm catches it on an image cell. The positive arm covers a
  log that lost the line.)
  Gated on the run having succeeded, because a cell that failed
  earlier (a chrt refusal, a node that died before construction) keeps
  its own, more precise verdict.

### Mode-A / G3 adaptations (fill exclusion, §3 to §4, unchanged in spirit)

- ROS 2 image cells pay their O(N) fill (the `data.resize(N)`
  zero-fill) ONCE at prealloc, exactly where the pod path pays its
  rosidl zero-fill; per iteration only the stamp (`header.stamp`, the
  field a real Image publisher stamps; CLOCK_MONOTONIC ns round-trips
  exactly) is written between receive/stamp and publish. The serialize
  + delivery copy `publish()` then performs on the N-byte vector IS the
  measured transport cost, the same rule §3 already applies to the
  rclcpp receive path.
- Cerulion pod legs write `prep` first (triggering the lazy SHM
  loan BEFORE the stamp is read, the pod twin of the Image legs' prep
  writes and the ROS 2 loan path's borrow-before-stamp), then the two
  stamp fields LAST; the fixed `data` array is never written.
- Both new legs keep the suite-wide wall-grid pacing, delivery
  accounting, watchdog, `.bin` gates, and schedules verbatim; the
  message class is the ONLY moving part within each stack.

### Env knobs

| Variable | Consumer | Meaning |
|---|---|---|
| `CER_BENCH_MSG` | ROS 2 driver + nodes (`pod` \| `image`), workspace runner (`variable` \| `pod`) | The type-class selector. Defaults preserve the incumbent cells byte-for-byte (`pod` on the ROS 2 side, `variable` on the workspace side; each side's default IS its incumbent class). Validated loudly on both sides; the vocabularies differ deliberately (each side names its classes by what they are, not by a shared token that would mislabel one of them). |
| `CER_BENCH_POD_BYTES` | pod node build scripts (workspace) | Bakes the `PodPayload` fixed-array length at build time (a fixed array is compile-time, the class distinction itself). Exported per size by `run_workspace.sh`; the nodes' init gate makes a stale value a loud failure, never a mislabel. **Multiples of 4 only**: `PodPayloadShm` is `#[repr(C)]` with three u32 fields, so any other total rounds up (65 → 68) and the matched-quantity rule cannot hold; the build script refuses such a value with the reason, and `run_workspace.sh` refuses a misaligned `CER_BENCH_PAYLOAD_SIZES` override under the pod class before it builds anything. Rounding is deliberately not offered (a rounded size is a mislabeled row); every pinned sweep size is already a multiple of 4. |

### Presentation

The two classes NEVER merge into one plotted series. Variable rows
carry the real type name in the legend (`sensor_msgs/Image
(variable)`); pod rows carry the fixed-array vocabulary: the workspace
pod legs always, and the ROS 2 pod rows on any figure where the ROS 2
stack itself carries both classes. That scoping is deliberate and is
what makes the legend the class's home rather than the footnote: an
incumbent ROS 2 figure with no image row keeps its legend text unchanged,
while a paired figure names both classes in the LEGEND, which
`--release` keeps (the matched-quantity footnote is an annotation and
is stripped there). Beneath the x-axis, curly braces span contiguous
payload groups sharing a rate label: on quiescent figures the braces
group by the pinned schedule's rate classes (derived from
`bench.quiescent_schedule`, one truth: "IMU class @1 kHz target" over
64 B to 1 KB … "lidar class @10 Hz target" at 16 MB); on fixed100 figures
one brace spans the sweep at the uniform target; backtoback figures
draw none (saturation makes no rate claim). Every brace says `target`
because the schedule is what the sweep ASKED for: the
`CER_BENCH_FORCE_RATE_HZ` diagnostic (§11) overrides it invisibly to
post-processing, so a forced-rate run's figure is a diagnostic, not a
rate claim. A figure carrying both classes states the matched-quantity
rule in its footnote block. The brace helper lives in `plot.py`
(`_draw_axis_braces`); suite-wide adoption on figures that carry no
type-class rows follows the same helper.

## 19. The readiness rule on the rmw_cerulion lane

Before a latency node starts its measured window it waits for the whole
ping to pong to echo chain to be up. On every RMW except one, that wait
is three conditions: a matched subscriber on this node's own kick
publisher, a matched publisher on its own echo subscription, and
count_subscribers("ping") above zero. The first two are endpoint counts
the transport itself carries, so they answer correctly across process
boundaries. The third is a ROS graph query, and rmw_cerulion answers
graph queries from a registry that holds only the calling process's own
endpoints, because cross process endpoint discovery in that
implementation is still a follow up. This harness runs ping, pong and
latency as three separate processes, so on rmw_cerulion that third
condition reads zero forever while the other two read one, the fifteen
second deadline expires, and no cell can produce a sample even though
the data path is healthy and both loan legs are available. Under
RMW_IMPLEMENTATION=rmw_cerulion only, and read from the environment once
at startup so no other lane is touched, the third condition is therefore
replaced by a readiness test made of data rather than of graph metadata:
the two matched conditions are kept exactly as they are, and then the
latency node publishes bounded probe kicks and treats the chain as ready
when the first echo comes back, which is direct proof that frames cross
the whole path and is strictly what the graph query stood in for. The
probe kicks and the echoes they produce are warm up. They are counted in
the kicks_sent receipt because they really were sent, they are never
stamped, never pushed into the sample array and never reach a bin file
or a percentile, and a short settle in probe mode absorbs any straggler
so a slow probe echo cannot be measured after the switch. Both the probe
phase and the surrounding wait are bounded at fifteen seconds each, so a
genuinely dead chain still fails in seconds with a message saying the
graph reported the chain wired while no data crossed it. Every ready
line now ends with a readiness field reading either matched+graph or
matched+probe, so no reader has to guess which rule gated a row.

## 20. Rule G3 on the echo side: payload sized work stays outside the timed window

Section 3 keeps payload sized writes out of the timed window. The source
comments call that rule G3: payload sized work stays outside the timed
window, and the stamp is the last thing written before the publish. The ping
node obeys it by borrowing its loaned message before it stamps, and the
pong node's copy path branch obeys it by preallocating its outbound
message once at construction and never rebuilding it. The pong node's
LOAN branch did not: it called borrow_loaned_message between taking the
ping and publishing the echo, and the comment beside it claimed the eight
byte stamp copy was the only work there. That claim was true only if a
borrow is a pointer handout, and it is not on every RMW. On rmw_cerulion
the loanable path loans an uninitialised shared memory slot, zeroes the
thirty two byte wire header, and then runs the typesupport init function
over the payload; the C++ introspection init writes every member, and
this bench's Pod message body is a fixed byte array, so the borrow
carries a write the size of the payload. Sitting where it sat, that write
was inside the round trip every echo and grew with the message, so part
of what the loan lane reported as transport latency was the harness
initialising its own reply. The pong node now borrows the reply for the
next echo immediately after publishing the current one, and borrows the
first one before it signals ready, so the only work between the take and
the publish is the stamp copy, exactly as on the ping side. The ready
line carries echo_rule, reading prefetched_loan on the loan branch and
preallocated_copy on the other, so a row always says which discipline
produced it. One limit: outside the window means outside under a
paced variant, where the chain is idle between echoes. Under back to back
pacing the chain is saturated and the next ping can arrive while the
borrow is still running, so there the cost moves rather than disappears.
It is never between the take and the publish again either way.

## 21. The echo node refills when nothing is in flight

Section 20 moved the pong node's payload sized borrow out of the window
between the take and the publish, and section 3's rule was then satisfied
in the letter but not in the spirit, because the borrow still ran
immediately after the publish while the echo it had just sent was still
in flight and the reader had not yet been woken. That placement produces
two modes whose separation is exactly one borrow, and the mode a sample
lands in is almost entirely decided by which core the reader wakes onto:
at 4 MiB, 98.1 percent of slow samples had the latency node take on the
core pong had just published from, against 5.2 percent of fast ones, and
the whole of the excess sat in the return leg while the forward leg was
identical between the modes. The pong node therefore arms a deadline
after the drain rather than refilling, one millisecond by default and
overridable through CER_BENCH_PONG_REFILL_DEFER_US, lets the wait expire
on it, and refills there, which is a point at which no sample is
outstanding and no reader is waiting to be placed.

Two alternatives exist and neither is used. Pinning the three nodes to
distinct cores removes the slow mode outright, taking the
4 MiB median from 100.2 to 13.5 microseconds and the 16 MiB ninety ninth
percentile from 839.7 to 33.7, but nothing else in this suite is pinned,
the native chart lines included, so adopting it for one lane would
measure that lane under a posture no other line on the chart shares.
Lowering the commanded rate to ten, twenty or fifty hertz moves the slow
fraction, but the fast mode itself roughly doubles at 4 MiB and rises
elevenfold at 16 MiB, because the cores fall into deeper idle between
kicks and the chart posture holds no C state lock, and the same core
correlation inverts: a lower rate trades one artifact for another
instead of removing either.

The liveness behaviour of the wait is otherwise unchanged at one hundred
milliseconds. If a ping arrives while the pool is empty the node borrows
inline rather than dropping the echo, counts it, and warns at exit naming
the count, because an inline borrow is the thing this design exists to
keep out of the window and a silent one would make the row a lie.
