# platform@ba5d75c5c fixed100 platform campaign (Jetson aarch64) -- ON pass (k=5)

p50 = median of per-rep p50s; p99 pooled across reps (n=10000); spread = per-rep p50 min-max; rate = achieved (fixed100 ladder).
STOCK posture (CER_BENCH_DMA_LOCK=0, chrt 0); recorder ON (always-on Flashback, the default); suite SHA `ba5d75c5c`; sampling n=2000/size/rep; NVIDIA Jetson Orin NX developer kit (aarch64, 8-core Arm Cortex-A78AE, 16 GB), Linux for Tegra R36.4.3, schedutil governor.
MISSING cells (no CSV -- unmeasured, reported as such): zenoh_shm_chrt0, jazzy_stock_rclcpp_chrt0, jazzy_composed_ipcon_rclcpp_chrt0

## STOCK posture, recorder ON (always-on Flashback, the default) -- headline sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 25.27 µs | 52.77 µs | 24.90 µs-28.19 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 MiB | 31.82 µs | 65.47 µs | 25.02 µs-34.59 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 MiB | 29.50 µs | 78.53 µs | 25.95 µs-50.08 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion mono | 64 B | 22.34 µs | 39.01 µs | 21.73 µs-23.46 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 24.23 µs | 50.41 µs | 21.63 µs-24.83 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 24.62 µs | 66.11 µs | 22.40 µs-25.70 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B | 1.70 µs | 8.58 µs | 1.28 µs-3.04 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | 3.14 µs | 8.86 µs | 1.41 µs-3.17 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | 3.07 µs | 13.79 µs | 2.98 µs-3.17 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |

## STOCK posture, recorder ON (always-on Flashback, the default) -- all sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 25.27 µs | 52.77 µs | 24.90 µs-28.19 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 256 B | 25.92 µs | 51.75 µs | 24.10 µs-28.26 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 KiB | 28.39 µs | 53.44 µs | 25.50 µs-28.99 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 4 KiB | 27.17 µs | 51.94 µs | 24.93 µs-27.68 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 KiB | 26.62 µs | 51.17 µs | 23.97 µs-29.12 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 64 KiB | 28.69 µs | 55.94 µs | 24.83 µs-30.37 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 256 KiB | 29.89 µs | 60.03 µs | 27.20 µs-31.97 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 1 MiB | 31.82 µs | 65.47 µs | 25.02 µs-34.59 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 4 MiB | 29.68 µs | 72.16 µs | 24.39 µs-43.01 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion split | 16 MiB | 29.50 µs | 78.53 µs | 25.95 µs-50.08 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run ON (CERULION_EXECUTION_MODE=free_run) |
| Cerulion mono | 64 B | 22.34 µs | 39.01 µs | 21.73 µs-23.46 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 B | 22.91 µs | 40.83 µs | 21.76 µs-24.23 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 KiB | 23.27 µs | 34.72 µs | 20.32 µs-24.26 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 KiB | 22.67 µs | 43.68 µs | 20.93 µs-24.39 µs | mixed Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 KiB | 22.85 µs | 37.44 µs | 20.64 µs-24.58 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 64 KiB | 22.62 µs | 38.34 µs | 22.21 µs-25.18 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 KiB | 23.41 µs | 45.73 µs | 23.20 µs-25.63 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 24.23 µs | 50.41 µs | 21.63 µs-24.83 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 MiB | 22.66 µs | 62.12 µs | 22.21 µs-29.25 µs | mixed Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 24.62 µs | 66.11 µs | 22.40 µs-25.70 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B | 1.70 µs | 8.58 µs | 1.28 µs-3.04 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 B | 1.41 µs | 8.26 µs | 1.31 µs-3.01 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 KiB | 3.01 µs | 8.32 µs | 1.34 µs-3.04 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 KiB | 2.75 µs | 8.70 µs | 1.31 µs-3.10 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 KiB | 3.04 µs | 8.77 µs | 1.41 µs-3.10 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 64 KiB | 2.91 µs | 8.54 µs | 1.28 µs-3.10 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 KiB | 1.38 µs | 8.80 µs | 1.31 µs-3.14 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | 3.14 µs | 8.86 µs | 1.41 µs-3.17 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 MiB | 2.82 µs | 8.51 µs | 1.31 µs-3.20 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | 3.07 µs | 13.79 µs | 2.98 µs-3.17 µs | 100 Hz | n/a (no Cerulion recorder in this cell) |

## Rate caveats (fixed100 ladder engaged; the published p50 is the median of per-rep p50s ACROSS rates)

- Cerulion mono @ 4 KiB: CSV achieved_rate_hz=mixed; per rep: rep1 100 Hz (p50 24.39 µs), rep2 100 Hz (p50 20.93 µs), rep3 100 Hz (p50 22.69 µs), rep4 100 Hz (p50 22.66 µs), rep5 50 Hz (p50 23.65 µs)
- Cerulion mono @ 4 MiB: CSV achieved_rate_hz=mixed; per rep: rep1 100 Hz (p50 29.25 µs), rep2 100 Hz (p50 22.46 µs), rep3 100 Hz (p50 22.66 µs), rep4 50 Hz (p50 27.71 µs), rep5 100 Hz (p50 22.21 µs)

Per-rep p50s are shown only where the raw .bin files are present in this copy; otherwise only the per-rep rates are known (from the .rate sidecars).

