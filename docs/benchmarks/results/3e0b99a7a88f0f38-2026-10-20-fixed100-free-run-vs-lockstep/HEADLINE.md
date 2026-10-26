# Free run against lockstep, multi-process, fixed100 (k=6)

p50 = median of per-rep p50s; p99 pooled across reps (n=12000); spread = per-rep p50 min-max; rate = achieved (fixed100 ladder).
STOCK posture (CER_BENCH_DMA_LOCK=0, chrt 0); recorder ON (always-on Flashback, the default); suite SHA `4ff3326d06` with the latency ping change that landed as `bbf276489b` applied; sampling n=2000/size/rep; x86-64 Linux machine (Ubuntu 22.04, kernel 6.8), performance governor.
One build, one machine, reps interleaved (A B, B A, ...). Free run = `CERULION_EXECUTION_MODE` unset, the default. Lockstep = `CERULION_EXECUTION_MODE=lockstep`, the opt out. Each rep's execution mode was asserted from the run before the rep was kept. Every cell held 100 Hz on every rep; no attempt was refused.

## Cerulion split (two processes), all sizes

| size | free run p50 | lockstep p50 | lockstep p50 vs free run | free run p99 (pooled) | lockstep p99 (pooled) | free run rep p50 spread | lockstep rep p50 spread | rate |
|---|---|---|---|---|---|---|---|---|
| 64 B | 4.23 µs | 5.08 µs | +20.0% | 7.60 µs | 10.47 µs | 4.17 µs-4.46 µs | 5.00 µs-5.11 µs | 100 Hz / 100 Hz |
| 256 B | 4.20 µs | 5.03 µs | +19.7% | 6.85 µs | 8.82 µs | 4.19 µs-4.22 µs | 4.94 µs-5.08 µs | 100 Hz / 100 Hz |
| 1 KiB | 4.22 µs | 5.05 µs | +19.7% | 7.58 µs | 8.41 µs | 4.16 µs-4.47 µs | 5.01 µs-5.12 µs | 100 Hz / 100 Hz |
| 4 KiB | 4.21 µs | 5.05 µs | +19.8% | 7.98 µs | 8.83 µs | 4.15 µs-4.24 µs | 5.00 µs-5.09 µs | 100 Hz / 100 Hz |
| 16 KiB | 4.22 µs | 5.09 µs | +20.5% | 8.50 µs | 9.14 µs | 4.20 µs-4.47 µs | 4.99 µs-5.14 µs | 100 Hz / 100 Hz |
| 64 KiB | 4.25 µs | 5.10 µs | +20.0% | 7.62 µs | 9.83 µs | 4.22 µs-4.34 µs | 5.06 µs-5.12 µs | 100 Hz / 100 Hz |
| 256 KiB | 4.33 µs | 5.27 µs | +21.7% | 7.68 µs | 9.87 µs | 4.29 µs-4.39 µs | 5.20 µs-5.42 µs | 100 Hz / 100 Hz |
| 1 MiB | 4.56 µs | 5.44 µs | +19.2% | 9.01 µs | 10.63 µs | 4.49 µs-4.59 µs | 5.42 µs-5.48 µs | 100 Hz / 100 Hz |
| 4 MiB | 4.86 µs | 6.02 µs | +23.9% | 10.10 µs | 11.47 µs | 4.75 µs-4.99 µs | 5.89 µs-6.13 µs | 100 Hz / 100 Hz |
| 16 MiB | 5.22 µs | 7.22 µs | +38.3% | 12.34 µs | 13.64 µs | 5.09 µs-6.82 µs | 7.03 µs-7.42 µs | 100 Hz / 100 Hz |

Free run has the lower p50 and the lower p99 at every size. Values are rendered from the two `results_cerulion_workspace_split_chrt0.csv` files in this package (nanoseconds, two decimals in µs).
