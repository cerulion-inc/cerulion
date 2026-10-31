# macOS data wake, before and after (k=5)

p50 = median of per-rep p50s; p99 pooled across reps; spread = per-rep p50 min-max.
Quiescent pacing (chrt 0), default settings, no pinning; `--msg-class variable`; Apple M3 Max (16 cores, 128 GiB), macOS 26.0.1 arm64, 24 MHz timebase (41.667 ns per tick).
Before = main `75b487eaa4`. After = `61fb8aac41` (its run.json sha), tree-identical to public commit `a0b4529fe7` on the pull request that merged as `64fc0de94c`. Five reps per cell per arm, the arms alternating within one session per cell. Recorder state not recorded.

## Cerulion split (two processes)

| size | p50 before | p50 after | p99 before (pooled) | p99 after (pooled) | before rep p50 spread | after rep p50 spread |
|---|---|---|---|---|---|---|
| 64 B | 70.17 µs | 10.42 µs | 141.54 µs | 28.21 µs | 69.58 µs-70.96 µs | 10.12 µs-11.42 µs |
| 4 KiB | 77.75 µs | 12.79 µs | 143.79 µs | 29.83 µs | 76.71 µs-78.42 µs | 11.29 µs-16.00 µs |
| 1 MiB | 80.12 µs | 16.14 µs | 152.42 µs | 41.67 µs | 78.98 µs-86.75 µs | 16.00 µs-17.25 µs |

## Cerulion mono (one process)

| size | p50 before | p50 after | p99 before (pooled) | p99 after (pooled) | before rep p50 spread | after rep p50 spread |
|---|---|---|---|---|---|---|
| 64 B | 4.46 µs | 4.50 µs | 10.00 µs | 10.04 µs | 4.29 µs-4.50 µs | 4.25 µs-4.50 µs |
| 4 KiB | 4.46 µs | 4.58 µs | 10.58 µs | 10.62 µs | 4.29 µs-4.54 µs | 4.29 µs-4.75 µs |
| 1 MiB | 7.25 µs | 7.38 µs | 23.88 µs | 24.25 µs | 6.96 µs-7.54 µs | 7.08 µs-7.67 µs |

Values are rendered from the CSVs in `before/` and `after/` (nanoseconds, two decimals in µs).
