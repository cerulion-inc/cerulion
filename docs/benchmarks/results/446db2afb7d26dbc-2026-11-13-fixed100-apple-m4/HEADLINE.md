# Apple M4 platform campaign, macOS 27.2.1, fixed100 (k=5)

p50 = median of per-rep p50s. p99 pooled across reps (n=10000). Spread = per-rep p50 min-max. Rate = achieved (fixed100 ladder).
STOCK posture in effect (CER_BENCH_DMA_LOCK unset, so the CPU DMA latency lock is requested, which does nothing outside Linux: 100 of 100 logs read "CPU C-state cap requested but unsupported on this OS", chrt 0). Recorder ON (always-on Flashback, the default). Suite SHA `27a091784b` (Cerulion 2.0.0 source, see PROVENANCE). run.json reads git_dirty: true from invocation 1 on, for the bench lockfile and the run's own output directory (see PROVENANCE). Sampling n=2000/size/rep. Apple M4 laptop (10-core, 24 GB), macOS 27.2.1 arm64.
Free run = `CERULION_EXECUTION_MODE` unset, the default. `--msg-class variable`. No raw iceoryx2, zenoh or ROS 2 cell was run in this campaign.

## Cerulion split (two processes), all sizes

| size | p50 | p99 (pooled) | rep p50 spread | rate |
|---|---|---|---|---|
| 64 B | 9.08 µs | 29.04 µs | 8.96 µs-9.83 µs | 100 Hz |
| 256 B | 9.21 µs | 30.79 µs | 9.04 µs-9.88 µs | 100 Hz |
| 1 KiB | 9.75 µs | 37.33 µs | 8.92 µs-11.17 µs | 100 Hz |
| 4 KiB | 9.17 µs | 33.05 µs | 9.00 µs-9.75 µs | 100 Hz |
| 16 KiB | 9.71 µs | 23.59 µs | 9.17 µs-9.96 µs | 100 Hz |
| 64 KiB | 9.83 µs | 32.00 µs | 9.21 µs-9.92 µs | 100 Hz |
| 256 KiB | 9.04 µs | 23.96 µs | 8.83 µs-9.67 µs | 100 Hz |
| 1 MiB | 9.71 µs | 31.67 µs | 9.14 µs-9.96 µs | 100 Hz |
| 4 MiB | 9.58 µs | 26.59 µs | 9.42 µs-10.19 µs | 100 Hz |
| 16 MiB | 9.54 µs | 31.79 µs | 9.42 µs-10.17 µs | 100 Hz |

## Cerulion mono (one process), all sizes

| size | p50 | p99 (pooled) | rep p50 spread | rate |
|---|---|---|---|---|
| 64 B | 3.75 µs | 16.92 µs | 3.67 µs-3.83 µs | 100 Hz |
| 256 B | 3.67 µs | 14.96 µs | 3.67 µs-3.79 µs | 100 Hz |
| 1 KiB | 3.75 µs | 21.83 µs | 3.58 µs-5.02 µs | 100 Hz |
| 4 KiB | 3.62 µs | 12.00 µs | 3.62 µs-3.67 µs | 100 Hz |
| 16 KiB | 3.62 µs | 11.62 µs | 3.58 µs-3.67 µs | 100 Hz |
| 64 KiB | 3.62 µs | 11.04 µs | 3.62 µs-3.67 µs | 100 Hz |
| 256 KiB | 3.71 µs | 20.42 µs | 3.58 µs-4.58 µs | 100 Hz |
| 1 MiB | 3.62 µs | 21.38 µs | 3.58 µs-5.19 µs | 100 Hz |
| 4 MiB | 3.83 µs | 12.62 µs | 3.75 µs-3.92 µs | 100 Hz |
| 16 MiB | 3.83 µs | 21.67 µs | 3.75 µs-4.29 µs | 100 Hz |

Values are rendered from the CSVs in this directory (nanoseconds, two decimals in µs).
