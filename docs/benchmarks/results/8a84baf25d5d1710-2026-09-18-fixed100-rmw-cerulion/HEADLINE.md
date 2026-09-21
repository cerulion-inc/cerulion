# rmw_cerulion fully loaned cell in the native latency harness (k=5)

p50 = median of per-rep p50s; every other percentile pooled across reps (n=10000); spread = per-rep p50 min-max; rate = achieved (fixed100 ladder). Round trip, nanoseconds measured, shown in microseconds.
STOCK posture (CER_BENCH_DMA_LOCK=0, chrt 0), unpinned; governor `performance`; ROS 2 Jazzy container, `RMW_IMPLEMENTATION=rmw_cerulion`, loaned publish and loaned take; harness commit `9f30d2b3b`; sampling n=2000/size/rep, 5 reps; x86-64 Linux desktop, machine hash `8a84baf25d5d1710` (the machine of the native fixed100 campaign package).

## Headline sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | loaned |
|---|---|---|---|---|---|---|
| ROS 2 over rmw_cerulion (loan) | 64 B | 10.45 µs | 36.18 µs | 10.28 µs-18.28 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 1 MiB | 11.12 µs | 37.21 µs | 11.01 µs-11.60 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 16 MiB | 17.47 µs | 54.38 µs | 16.91 µs-18.80 µs | 100 Hz | 1 |

## All sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | loaned |
|---|---|---|---|---|---|---|
| ROS 2 over rmw_cerulion (loan) | 64 B | 10.45 µs | 36.18 µs | 10.28 µs-18.28 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 256 B | 10.58 µs | 45.06 µs | 10.50 µs-11.01 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 1 KiB | 10.43 µs | 36.38 µs | 10.22 µs-10.68 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 4 KiB | 10.52 µs | 26.68 µs | 10.30 µs-10.53 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 16 KiB | 10.49 µs | 23.65 µs | 10.35 µs-10.80 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 64 KiB | 10.65 µs | 53.26 µs | 10.19 µs-10.83 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 256 KiB | 10.58 µs | 37.62 µs | 10.47 µs-10.91 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 1 MiB | 11.12 µs | 37.21 µs | 11.01 µs-11.60 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 4 MiB | 15.05 µs | 50.55 µs | 14.70 µs-15.64 µs | 100 Hz | 1 |
| ROS 2 over rmw_cerulion (loan) | 16 MiB | 17.47 µs | 54.38 µs | 16.91 µs-18.80 µs | 100 Hz | 1 |

## Pooled distribution, all sizes

| size | floor | p25 | p50 (median of rep p50s) | p75 | p90 | p95 | p99 | p99.9 | max |
|---|---|---|---|---|---|---|---|---|---|
| 64 B | 8.25 µs | 10.08 µs | 10.45 µs | 11.18 µs | 18.50 µs | 19.38 µs | 36.18 µs | suppressed | 2431.65 µs |
| 256 B | 8.21 µs | 10.22 µs | 10.58 µs | 11.35 µs | 14.61 µs | 18.57 µs | 45.06 µs | suppressed | 280.09 µs |
| 1 KiB | 8.23 µs | 10.01 µs | 10.43 µs | 10.91 µs | 11.85 µs | 14.41 µs | 36.38 µs | suppressed | 236.54 µs |
| 4 KiB | 6.45 µs | 10.10 µs | 10.52 µs | 10.97 µs | 11.69 µs | 12.38 µs | 26.68 µs | suppressed | 250.02 µs |
| 16 KiB | 8.45 µs | 10.16 µs | 10.49 µs | 10.90 µs | 11.41 µs | 11.95 µs | 23.65 µs | suppressed | 235.89 µs |
| 64 KiB | 8.39 µs | 10.17 µs | 10.65 µs | 11.06 µs | 12.19 µs | 18.13 µs | 53.26 µs | suppressed | 959.34 µs |
| 256 KiB | 8.03 µs | 10.15 µs | 10.58 µs | 11.09 µs | 11.81 µs | 14.09 µs | 37.62 µs | suppressed | 1057.52 µs |
| 1 MiB | 8.28 µs | 10.74 µs | 11.12 µs | 11.84 µs | 12.84 µs | 18.02 µs | 37.21 µs | suppressed | 350.17 µs |
| 4 MiB | 9.86 µs | 12.77 µs | 15.05 µs | 15.84 µs | 16.66 µs | 17.45 µs | 50.55 µs | suppressed | 1127.08 µs |
| 16 MiB | 12.71 µs | 16.17 µs | 17.47 µs | 20.05 µs | 24.07 µs | 28.65 µs | 54.38 µs | suppressed | 1838.37 µs |

p99.9 is suppressed by the suite's own rule: a percentile cell is written empty when fewer than 20 pooled samples sit in the tail it rests on, and 10000 samples leave 10 above p99.9.

## Per-rep p50

| size | rep1 | rep2 | rep3 | rep4 | rep5 |
|---|---|---|---|---|---|
| 64 B | 10.45 µs | 18.28 µs | 10.44 µs | 10.28 µs | 10.50 µs |
| 256 B | 10.58 µs | 11.01 µs | 10.86 µs | 10.55 µs | 10.50 µs |
| 1 KiB | 10.43 µs | 10.68 µs | 10.48 µs | 10.40 µs | 10.22 µs |
| 4 KiB | 10.30 µs | 10.39 µs | 10.52 µs | 10.53 µs | 10.53 µs |
| 16 KiB | 10.43 µs | 10.61 µs | 10.49 µs | 10.80 µs | 10.35 µs |
| 64 KiB | 10.53 µs | 10.65 µs | 10.83 µs | 10.82 µs | 10.19 µs |
| 256 KiB | 10.91 µs | 10.47 µs | 10.58 µs | 10.54 µs | 10.60 µs |
| 1 MiB | 11.09 µs | 11.49 µs | 11.60 µs | 11.01 µs | 11.12 µs |
| 4 MiB | 15.01 µs | 15.64 µs | 15.05 µs | 14.70 µs | 15.22 µs |
| 16 MiB | 17.47 µs | 18.80 µs | 16.91 µs | 17.03 µs | 18.25 µs |

## Delivery accounting and inline borrows

| size | cells | echoes (pong `echoed=`) | inline borrows | unstamped | nonpositive rtt | take failures (latency + pong) |
|---|---|---|---|---|---|---|
| 64 B | 5 | 10505 | 0 | 0 | 0 | 0 |
| 256 B | 5 | 10505 | 0 | 0 | 0 | 0 |
| 1 KiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| 4 KiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| 16 KiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| 64 KiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| 256 KiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| 1 MiB | 5 | 10505 | 1 | 0 | 0 | 0 |
| 4 MiB | 5 | 10505 | 3 | 0 | 0 | 0 |
| 16 MiB | 5 | 10505 | 0 | 0 | 0 | 0 |
| **total** | **50** | **105050** | **4** | | | |

4 inline borrows in 105050 echoes (0.0038 percent), one each in: 1 MiB rep5, 4 MiB rep2, 4 MiB rep3, 4 MiB rep4. An inline borrow is one echo that found the prefetched loan missing and borrowed inside its own timed window; the echo node counts it and warns at exit.

## Acceptance smoke (k=1, 2000 samples per size, same posture, CPU probes on)

| size | p25 | p50 | p90 | p99 | p90/p50 | inline borrows |
|---|---|---|---|---|---|---|
| 64 B | 10.30 µs | 10.79 µs | 12.13 µs | 37.93 µs | 1.12 | 0 |
| 256 KiB | 10.57 µs | 11.03 µs | 12.22 µs | 29.46 µs | 1.11 | 0 |
| 1 MiB | 10.78 µs | 11.30 µs | 12.37 µs | 53.44 µs | 1.10 | 0 |
| 4 MiB | 14.99 µs | 16.99 µs | 18.31 µs | 89.22 µs | 1.08 | 0 |
| 16 MiB | 17.14 µs | 19.11 µs | 25.93 µs | 92.98 µs | 1.36 | 0 |

## Stock parity spot check (rmw_fastrtps_cpp defaults, k=1, same harness, build and posture)

| size | this run p50 (k=1) | native fixed100 campaign p50 (k=5) | ratio | rmw_cerulion loan p50 in this package |
|---|---|---|---|---|
| 4 MiB | 13.05 ms | 12.91 ms | 1.01x | 15.05 µs |
| 16 MiB | 102.93 ms | 103.85 ms | 0.99x | 17.47 µs |

The published column is `results_jazzy_stock_rclcpp_chrt0.csv` in the native fixed100 campaign package (`../8a84baf25d5d1710-2026-09-16-fixed100-heroes/hero-stock/`).
