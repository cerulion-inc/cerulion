# ROS 2 over rmw_cerulion, measured in the native latency harness

This package is the evidence for one line: ROS 2 Jazzy running over
`rmw_cerulion` with loaned messages in both directions, measured by
`benches/latency` in the same harness and the same posture as every other
line on the native round trip chart. `HEADLINE.md` renders the tables. Every
figure below is in `HEADLINE.md`, and every figure there is computed from
files in this package.

## What was measured

Three ROS 2 nodes run as three separate processes in one ROS 2 Jazzy
container with `RMW_IMPLEMENTATION=rmw_cerulion`: a ping node, an echo node
and a latency node. The ping node writes a `CLOCK_MONOTONIC` stamp into the
message as its last act before publishing. The echo node takes the message
and publishes it back. The latency node takes the echo, reads the same clock
and subtracts. One sample is that difference: two transport hops through the
echo node, in nanoseconds. Publishing uses a loaned message and receiving
uses a loaned take on every hop (`loaned=1` on every row), so the payload is
never copied by the transport.

The cell is `jazzy_cerulion_shm_loan_be1_chrt0`, at ten payload sizes from
64 B to 16 MiB. The pacing variant is `fixed100`: one round trip every 10 ms,
100 warm up round trips and then 2,000 measured ones per size. That was
repeated five times (k=5), so each size has 10,000 measured samples. Every
row held the full 100 Hz; none fell down the rate ladder.

## Posture

Stock, the posture of the chart: no CPU idle state lock
(`CER_BENCH_DMA_LOCK=0`), no real time priority (chrt 0), the `performance`
governor, and no CPU pinning. The three nodes are placed by the kernel, like
every other line on the chart. It is one machine, the x86-64 Linux desktop of
the native fixed100 campaign package (same machine hash, `8a84baf25d5d1710`).

## How p50 and p99 are computed

These are the suite's own definitions, implemented in
`benches/latency/compile_csv.py`.

- A percentile is the linear interpolated quantile of the sorted samples,
  truncated to a whole nanosecond.
- The published p50 is the median of the five per-rep p50s, not the p50 of
  the pooled samples, so one unusual rep cannot move it.
- Every other statistic, p99 included, pools all 10,000 samples of a size.
- A percentile is left empty when fewer than 20 pooled samples sit in the
  tail it rests on. With 10,000 samples that blanks p99.9 and nothing else.

`HEADLINE.md` was checked the other way as well: recomputing all ten rows
from the `.bin` samples reproduces the CSV exactly.

## Result

| size | p50 | p99 (pooled) |
|---|---|---|
| 64 B | 10.45 µs | 36.18 µs |
| 1 MiB | 11.12 µs | 37.21 µs |
| 16 MiB | 17.47 µs | 54.38 µs |

The p50 stays between 10.43 µs and 10.65 µs from 64 B through 256 KiB, then
reads 11.12 µs at 1 MiB, 15.05 µs at 4 MiB and 17.47 µs at 16 MiB.

## Caveats a citation must carry

1. **Four inline borrows in 105,050 echoes (0.0038 percent).** The echo node
   keeps one reply loan borrowed ahead of time. Four times, one echo found it
   missing and borrowed inside its own timed window: once each at 4 MiB in
   reps 2, 3 and 4, and once at 1 MiB in rep 5. The node counts these and
   warns at exit. The affected samples are counted in HEADLINE.md; the
   published p50 is a median of rep p50s and is unaffected.
2. **One outlier rep at 64 B.** Rep 2 has a p50 of 18.28 µs; the other four
   sit between 10.28 µs and 10.50 µs. The published p50 is a median of rep
   p50s and is unaffected. The pooled p90 at 64 B (18.50 µs) is inflated by
   that rep.
3. **The echo node defers its reply loan refill by 1 ms.** Borrowing a loaned
   message on `rmw_cerulion` initialises the payload, which is work the size
   of the payload. Done right after a publish, while that echo was still in
   flight, it held the core the reader was about to wake on and produced a
   second, slower mode that was a scheduler placement artifact of the
   harness rather than a property of the transport. The node now refills
   once nothing is in flight. The change is to the loaned echo path on the
   ROS 2 side of the harness, whichever RMW is loaded; the native lines and
   the stock ROS 2 cell, which does not loan, are untouched. METHODOLOGY
   sections 20 and 21 give the full argument and the alternatives that were
   measured and rejected.
4. **Unpinned, like every other line.** Pinning the three nodes to separate
   cores was measured and rejected because no other line on the chart is
   pinned.
5. **Payload fill is outside the timed window** on every node, the suite's
   fill exclusion rule (METHODOLOGY section 3), as on every other line.
6. **`run.json` records `git_dirty: true`** on every invocation. The harness
   sets that flag from `git status` in the checkout it ran from, and this
   package cannot show which paths were uncommitted. What it does pin: the
   container image label equals the recorded commit, and all ten build
   receipts record one `librmw_cerulion.so`
   (sha256 `979ad085fac9a40c6d5fcd1a9b5ed469739826800463b6acc1515fd31959374f`).
7. **One harness, one posture.** Every number here comes from the same
   harness, posture and pacing as the native lines it is drawn beside.

## Checks that ran beside it

- **Acceptance smoke** (`acceptance-smoke/`): one rep at five sizes with the
  echo node's CPU probes on, run before the campaign. Zero inline borrows,
  and p90 over p50 between 1.08 and 1.36.
- **Stock parity spot check** (`stock-parity-spot-check/`): the chart's stock
  ROS 2 cell, `jazzy_stock_rclcpp_chrt0`, one rep at 4 MiB and 16 MiB with the
  same build and posture. It reads 13.05 ms and 102.93 ms against 12.91 ms
  and 103.85 ms in the native fixed100 campaign package (1.01x and 0.99x), so
  the harness change did not disturb the stock lane.

## What is here

| path | contents |
|---|---|
| `HEADLINE.md` | the tables, rendered from the files below |
| `PROVENANCE.txt` | build, image, posture, and what was normalized for publication |
| `jazzy-cerulion-loan-k5/` | the five rep run directory: `run.json`, the compiled CSV, and per rep the raw `.bin` samples (little endian unsigned 64 bit nanoseconds), `.rate` sidecars, node logs, delivery receipts, the build receipt, package versions and the driver's per cell logs |
| `stock-parity-spot-check/` | the same layout for the stock cell, in its own run directory |
| `acceptance-smoke/` | per size: `cell.log`, samples, node log, receipts, and the probe outputs `rec.bin` and `latcpu.i32` |
| `logs/` | the two driver logs and, under `launchers/`, the two scripts as run |

On the bench machine the run directory was named
`8a84baf25d5d1710-2026-09-18-fixed100-g3-deferred`, and that name survives in
the `argv` recorded in `run.json` and in the driver log, which are records of
what ran. G3 is the suite's name for the rule that payload sized work stays
outside the timed window, and "deferred" is the deferred refill of caveat 3.
No manifest or checksum binds the directory name.

## Recompute

```
python3 benches/latency/bench.py compile-csv --variant fixed100 \
    --run-dir docs/benchmarks/results/8a84baf25d5d1710-2026-09-18-fixed100-rmw-cerulion/jazzy-cerulion-loan-k5
```

That is a strict compile: it fails unless all ten sizes are present in every
rep.
