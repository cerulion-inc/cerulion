# fixed100 hero re-measure -- ON pass (k=5)

p50 = median of per-rep p50s; p99 pooled across reps (n=10000); spread = per-rep p50 min-max; rate = achieved (fixed100 ladder).
STOCK posture (CER_BENCH_DMA_LOCK=0, chrt 0); recorder ON (always-on Flashback, the default); suite SHA `a68a2a987`; sampling n=2000/size/rep; x86-64 Linux desktop (24-core Intel Core Ultra 9 285K), performance governor.

## STOCK posture, recorder ON (always-on Flashback, the default) -- headline sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 4.07 µs | 6.61 µs | 3.92 µs-4.09 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 MiB | 4.08 µs | 7.38 µs | 4.05 µs-4.14 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 MiB | 4.43 µs | 10.45 µs | 4.39 µs-4.49 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion mono | 64 B | 2.69 µs | 9.41 µs | 2.66 µs-2.70 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 2.76 µs | 10.74 µs | 2.69 µs-2.77 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 2.80 µs | 14.23 µs | 2.77 µs-2.85 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B | 2.42 µs | 3.02 µs | 1.34 µs-2.50 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | 2.48 µs | 3.07 µs | 2.47 µs-2.78 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | 2.84 µs | 3.21 µs | 2.46 µs-3.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 64 B | 509.67 µs | 831.36 µs | 420.98 µs-608.46 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 1 MiB | 580.63 µs | 833.60 µs | 133.16 µs-653.65 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 16 MiB | 591.34 µs | 800.38 µs | 438.71 µs-619.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 64 B | 311.11 µs | 568.17 µs | 181.09 µs-487.32 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 1 MiB | 11.0 ms | 43.4 ms | 10.9 ms-11.3 ms | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 16 MiB | 103.9 ms | 156.9 ms | 102.9 ms-104.2 ms | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 64 B | 24.64 µs | 31.47 µs | 24.06 µs-28.90 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 1 MiB | 29.79 µs | 36.99 µs | 13.67 µs-34.06 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 16 MiB | 33.41 µs | 37.34 µs | 28.49 µs-34.15 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |

## STOCK posture, recorder ON (always-on Flashback, the default) -- all sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 4.07 µs | 6.61 µs | 3.92 µs-4.09 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 256 B | 3.97 µs | 6.62 µs | 3.92 µs-4.13 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 KiB | 4.01 µs | 7.77 µs | 3.94 µs-4.12 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 4 KiB | 4.01 µs | 6.75 µs | 3.94 µs-4.07 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 KiB | 4.02 µs | 6.45 µs | 3.98 µs-4.13 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 64 KiB | 4.04 µs | 6.76 µs | 4.01 µs-4.10 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 256 KiB | 4.04 µs | 7.63 µs | 3.98 µs-4.19 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 MiB | 4.08 µs | 7.38 µs | 4.05 µs-4.14 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 4 MiB | 4.33 µs | 8.18 µs | 4.24 µs-4.35 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 MiB | 4.43 µs | 10.45 µs | 4.39 µs-4.49 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion mono | 64 B | 2.69 µs | 9.41 µs | 2.66 µs-2.70 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 B | 2.68 µs | 9.51 µs | 2.65 µs-2.74 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 KiB | 2.66 µs | 9.35 µs | 2.65 µs-2.69 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 KiB | 2.68 µs | 9.37 µs | 2.65 µs-2.73 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 KiB | 2.70 µs | 9.71 µs | 2.67 µs-2.78 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 64 KiB | 2.75 µs | 9.94 µs | 2.71 µs-2.86 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 KiB | 2.71 µs | 10.03 µs | 2.69 µs-2.86 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 2.76 µs | 10.74 µs | 2.69 µs-2.77 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 MiB | 2.77 µs | 11.66 µs | 2.73 µs-2.81 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 2.80 µs | 14.23 µs | 2.77 µs-2.85 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B | 2.42 µs | 3.02 µs | 1.34 µs-2.50 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 B | 2.42 µs | 3.06 µs | 2.40 µs-2.88 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 KiB | 2.47 µs | 3.06 µs | 2.42 µs-2.88 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 KiB | 2.42 µs | 6.78 µs | 2.38 µs-2.92 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 KiB | 2.41 µs | 3.06 µs | 2.40 µs-2.85 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 64 KiB | 2.44 µs | 3.06 µs | 1.37 µs-2.85 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 KiB | 2.49 µs | 3.05 µs | 2.44 µs-2.85 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | 2.48 µs | 3.07 µs | 2.47 µs-2.78 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 MiB | 2.83 µs | 3.17 µs | 2.51 µs-2.91 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | 2.84 µs | 3.21 µs | 2.46 µs-3.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 64 B | 509.67 µs | 831.36 µs | 420.98 µs-608.46 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 256 B | 534.70 µs | 741.36 µs | 442.08 µs-577.92 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 1 KiB | 469.11 µs | 823.75 µs | 362.37 µs-654.90 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 4 KiB | 475.26 µs | 742.77 µs | 255.57 µs-546.83 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 16 KiB | 413.06 µs | 652.32 µs | 351.35 µs-503.32 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 64 KiB | 423.94 µs | 755.24 µs | 316.61 µs-601.79 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 256 KiB | 539.09 µs | 757.59 µs | 477.97 µs-561.14 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 1 MiB | 580.63 µs | 833.60 µs | 133.16 µs-653.65 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 4 MiB | 542.95 µs | 800.30 µs | 490.80 µs-589.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| zenoh SHM | 16 MiB | 591.34 µs | 800.38 µs | 438.71 µs-619.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 64 B | 311.11 µs | 568.17 µs | 181.09 µs-487.32 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 256 B | 309.68 µs | 556.15 µs | 75.84 µs-396.63 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 1 KiB | 409.71 µs | 604.25 µs | 186.77 µs-474.27 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 4 KiB | 442.23 µs | 587.25 µs | 271.82 µs-462.62 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 16 KiB | 464.06 µs | 614.18 µs | 360.04 µs-520.61 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 64 KiB | 463.75 µs | 639.07 µs | 433.77 µs-538.29 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 256 KiB | 483.16 µs | 653.17 µs | 363.52 µs-502.49 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 1 MiB | 11.0 ms | 43.4 ms | 10.9 ms-11.3 ms | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 4 MiB | 12.9 ms | 36.0 ms | 4.5 ms-12.9 ms | 100 Hz | n/a (no Cerulion recorder in this cell) |
| stock ROS 2 | 16 MiB | 103.9 ms | 156.9 ms | 102.9 ms-104.2 ms | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 64 B | 24.64 µs | 31.47 µs | 24.06 µs-28.90 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 256 B | 28.33 µs | 30.79 µs | 25.15 µs-28.55 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 1 KiB | 28.48 µs | 31.20 µs | 24.16 µs-29.70 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 4 KiB | 28.39 µs | 31.22 µs | 24.37 µs-29.64 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 16 KiB | 28.90 µs | 32.47 µs | 11.60 µs-30.42 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 64 KiB | 25.85 µs | 31.42 µs | 24.95 µs-29.81 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 256 KiB | 27.73 µs | 42.08 µs | 15.64 µs-29.42 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 1 MiB | 29.79 µs | 36.99 µs | 13.67 µs-34.06 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 4 MiB | 33.63 µs | 36.48 µs | 25.02 µs-34.60 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| ROS 2 composed+IPC | 16 MiB | 33.41 µs | 37.34 µs | 28.49 µs-34.15 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |

## README table rows (docs/PERFORMANCE.md format; p50 with p99 in parentheses) -- recorder ON (always-on Flashback, the default)

| Payload | Cerulion (multi-process, free run enabled) | ROS 2 (defaults) | vs. ROS 2 defaults | Cerulion (single process) | ROS 2 (composed + intra-process) |
|---|---|---|---|---|---|
| 64 B | 4.07 µs (p99 6.61 µs) | 311 µs (p99 568 µs) | 76× | 2.69 µs (p99 9.41 µs) | 24.6 µs (p99 31.5 µs) |
| 256 B | 3.97 µs (p99 6.62 µs) | 310 µs (p99 556 µs) | 78× | 2.68 µs (p99 9.51 µs) | 28.3 µs (p99 30.8 µs) |
| 1 KiB | 4.01 µs (p99 7.77 µs) | 410 µs (p99 604 µs) | 102× | 2.66 µs (p99 9.35 µs) | 28.5 µs (p99 31.2 µs) |
| 4 KiB | 4.01 µs (p99 6.75 µs) | 442 µs (p99 587 µs) | 110× | 2.68 µs (p99 9.37 µs) | 28.4 µs (p99 31.2 µs) |
| 16 KiB | 4.02 µs (p99 6.45 µs) | 464 µs (p99 614 µs) | 116× | 2.70 µs (p99 9.71 µs) | 28.9 µs (p99 32.5 µs) |
| 64 KiB | 4.04 µs (p99 6.76 µs) | 464 µs (p99 639 µs) | 115× | 2.75 µs (p99 9.94 µs) | 25.9 µs (p99 31.4 µs) |
| 256 KiB | 4.04 µs (p99 7.63 µs) | 483 µs (p99 653 µs) | 120× | 2.71 µs (p99 10.0 µs) | 27.7 µs (p99 42.1 µs) |
| 1 MiB | 4.08 µs (p99 7.38 µs) | 11.0 ms (p99 43.4 ms) | 2,687× | 2.76 µs (p99 10.7 µs) | 29.8 µs (p99 37.0 µs) |
| 4 MiB | 4.33 µs (p99 8.18 µs) | 12.9 ms (p99 36.0 ms) | 2,983× | 2.77 µs (p99 11.7 µs) | 33.6 µs (p99 36.5 µs) |
| 16 MiB | 4.43 µs (p99 10.4 µs) | 103.9 ms (p99 156.9 ms) | 23,422× | 2.80 µs (p99 14.2 µs) | 33.4 µs (p99 37.3 µs) |

Ratios are ROS 2 defaults p50 / Cerulion multi-process (free run enabled) p50 from unrounded ns.

Lockstep is the shipped default; the multi-process row was measured with CERULION_EXECUTION_MODE=free_run (see PROVENANCE-TWO-BUILDS.txt).

## Provenance: TWO builds in this package

| rows | build | cerulion binary sha16 |
|---|---|---|
| mono, iox2 floor, zenoh_shm, both jazzy rows | main 68a0d6196a962237f8628cd94c9f93371703e827 | 17d52c9b1552e530 |
| the MULTI-PROCESS row (published) | PR #912 head a68a2a987dcee4945d6620b11b04af039e15a3bd | d7d940c5aec0c63d |

Never cite a single provenance sha for this package.

- `hero-stock-freerun-fixed/` is the PUBLISHED multi-process row (#912 head:
  F1 recorder pin+nice, F2 dead-node walk off the live loop).
- `hero-stock-freerun/` is stage A3: pre-fix, main 68a0d6196, free run +
  recorder ON, post-sweep. LABELED DIAGNOSTIC.
- `hero-stock-lockstep/` is the lockstep split, recorder ON, PRE-sweep,
  rate-mixed (100/50/50/50/100 Hz at 16 MiB). LABELED DIAGNOSTIC.
- The iox2 floor row is `iox2-rerun/` (post-sweep). `hero-stock/`'s iox2 CSV is
  the PRE-sweep diagnostic, kept for comparison.

`hero-stock-freerun-fixed/run.json` carries six invocations: the first
(09:43:36Z) is the aborted first launch's rep 1, stopped at its witness gate by
a false-negative thread-name match, and no data from it is used; the five
invocations from 09:51:28Z are the pooled series, and rep1/ holds the
relaunch's files (its first per-cell log is stamped 09:52:13Z). run.json is not
edited; this note explains it.
