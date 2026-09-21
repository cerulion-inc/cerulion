# platform@ba5d75c5c fixed100 platform campaign (macOS) -- ON pass (k=5)

p50 = median of per-rep p50s; p99 pooled across reps (n=10000); spread = per-rep p50 min-max; rate = achieved (fixed100 ladder).
STOCK posture (untuned: no RT scheduling, no governor control on this platform; CER_BENCH_DMA_LOCK=0); recorder ON (always-on Flashback, the default); suite SHA `ba5d75c5c`; sampling n=2000/size/rep; Apple M4 laptop (10-core, 24 GB), macOS 26.
MISSING cells (no CSV -- unmeasured, reported as such): zenoh_shm_chrt0, jazzy_stock_rclcpp_chrt0, jazzy_composed_ipcon_rclcpp_chrt0

## STOCK posture, recorder ON (always-on Flashback, the default) -- headline sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 88.48 µs | 142.17 µs | 87.98 µs-90.46 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 1 MiB | 87.85 µs | 140.92 µs | 84.21 µs-91.33 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 16 MiB | 89.77 µs | 138.42 µs | 89.60 µs-92.71 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion mono | 64 B | 3.46 µs | 19.12 µs | 3.42 µs-4.38 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 3.46 µs | 20.54 µs | 3.38 µs-3.96 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 3.75 µs | 10.04 µs | 3.71 µs-3.88 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B |  |  |  | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |

## FLOOR ROW (raw iceoryx2)

floor (backtoback variant), k=5 x 10 sizes, 50000 pooled samples per size, STOCK:

| size | p50 | p99 (pooled) | rep p50 spread | >100 us |
|---|---|---|---|---|
| 64 B | 0.625 us | 1.458 us | 0.584 us-1.084 us | 0 |
| 256 B | 0.792 us | 1.083 us | 0.458 us-0.833 us | 0 |
| 1 KiB | 0.666 us | 0.917 us | 0.458 us-0.791 us | 0 |
| 4 KiB | 0.584 us | 0.792 us | 0.500 us-0.708 us | 0 |
| 16 KiB | 0.542 us | 0.750 us | 0.500 us-0.667 us | 0 |
| 64 KiB | 0.500 us | 0.667 us | 0.458 us-0.583 us | 0 |
| 256 KiB | 0.459 us | 0.625 us | 0.417 us-0.542 us | 0 |
| 1 MiB | 0.459 us | 0.584 us | 0.416 us-0.500 us | 0 |
| 4 MiB | 0.459 us | 0.584 us | 0.417 us-0.500 us | 0 |
| 16 MiB | 0.459 us | 0.583 us | 0.417 us-0.459 us | 0 |

p50 0.459 us-0.792 us across the whole sweep; worst pooled p99 1.458 us (at 64 B); 0 of 500,000
samples over 100 us. The p50 spread across sizes is 1.73x, but it runs the WRONG WAY for a payload
cost - the SMALLEST payload is the slowest and the curve falls monotonically to 16 MiB, which rules
out an O(n) copy; the 64 B and 256 B rows are the first sizes measured in each invocation and carry
the warm-up. Zero-copy holds: a 16 MiB round trip costs the same 0.459 us as a 256 KiB one.

floor (fixed100, the variant every other row here uses): **not sustained on macOS** - the cell
exhausted the 100/50/20 Hz ladder and no latency was minted. The reason is NOT the transport: a
backtoback run of the SAME binary, with the rate limiter removed, does the round trip in 0.459-0.792 us
p50 (0 of 500,000 samples over 100 us) and finishes all ten sizes x 5 reps in 4 seconds. At ~0.8 us a 100 Hz
slot is ~12,000 round trips wide, so the cell cannot miss slots on transport work. What it misses
them on is the pacer's own `thread::sleep(remaining - 50 us)` (native/src/lib.rs:656-684), which
on macOS overshoots by tens of milliseconds: every slot lands late, the grid index jumps forward,
and the skip counter runs past the slot count ("skipped 6669 of 2100"). The overshoot GREW with the
rung period (~42 ms at 10 ms slots, ~80 ms at 20 ms, ~98 ms at 50 ms) - a sleep-granularity
signature, not a payload one.

The backtoback number is labeled backtoback and is NOT comparable to the fixed100 rows above: it was
not rate-paced at all. The discriminator that established this (one size, one rep) is kept in evidence/ alongside the k=5 curve above.
Evidence: evidence/backtoback-discriminator/ (FINDING.txt, the run log, run.json, percentiles).

## STOCK posture, recorder ON (always-on Flashback, the default) -- all sizes

| cell | size | p50 | p99 (pooled) | rep p50 spread | rate | recorder |
|---|---|---|---|---|---|---|
| Cerulion split | 64 B | 88.48 µs | 142.17 µs | 87.98 µs-90.46 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 256 B | 89.23 µs | 140.54 µs | 84.79 µs-91.21 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 1 KiB | 89.38 µs | 139.88 µs | 86.40 µs-90.58 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 4 KiB | 89.42 µs | 139.54 µs | 85.81 µs-90.48 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 16 KiB | 87.06 µs | 140.12 µs | 86.06 µs-89.58 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 64 KiB | 87.38 µs | 140.38 µs | 87.27 µs-88.77 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 256 KiB | 87.94 µs | 140.88 µs | 87.46 µs-90.31 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 1 MiB | 87.85 µs | 140.92 µs | 84.21 µs-91.33 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 4 MiB | 92.90 µs | 139.46 µs | 91.15 µs-94.12 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion split | 16 MiB | 89.77 µs | 138.42 µs | 89.60 µs-92.71 µs | 100 Hz | ON (always-on Flashback default; env untouched); free run on, degraded park, no recorder placement witness on macOS |
| Cerulion mono | 64 B | 3.46 µs | 19.12 µs | 3.42 µs-4.38 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 B | 3.46 µs | 13.33 µs | 3.42 µs-3.46 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 KiB | 3.46 µs | 11.75 µs | 3.42 µs-3.50 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 KiB | 3.38 µs | 7.79 µs | 3.38 µs-3.46 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 KiB | 3.46 µs | 11.83 µs | 3.38 µs-3.50 µs | mixed Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 64 KiB | 3.38 µs | 7.46 µs | 3.38 µs-3.42 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 256 KiB | 3.38 µs | 21.42 µs | 3.38 µs-4.42 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 1 MiB | 3.46 µs | 20.54 µs | 3.38 µs-3.96 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 4 MiB | 3.58 µs | 10.75 µs | 3.58 µs-3.67 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| Cerulion mono | 16 MiB | 3.75 µs | 10.04 µs | 3.71 µs-3.88 µs | 100 Hz | ON (always-on Flashback default; env untouched) |
| raw iceoryx2 | 64 B |  |  |  | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 B |  |  |  | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 KiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 KiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 KiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 64 KiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 256 KiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 1 MiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 4 MiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |
| raw iceoryx2 | 16 MiB | (not minted) | | | not sustained (fixed100) - see FLOOR below | n/a (no Cerulion recorder in this cell) |

## Rate caveats (fixed100 ladder engaged; the published p50 is the median of per-rep p50s ACROSS rates)

- Cerulion mono @ 16 KiB: CSV achieved_rate_hz=mixed; per rep: rep1 50 Hz, rep2 100 Hz, rep3 100 Hz, rep4 100 Hz, rep5 100 Hz
- raw iceoryx2 @ 64 B: CSV achieved_rate_hz=did_not_sustain; per rep: rep1 did_not_sustain Hz, rep2 no sidecar Hz, rep3 no sidecar Hz, rep4 no sidecar Hz, rep5 no sidecar Hz
- raw iceoryx2 @ 256 B: CSV achieved_rate_hz=did_not_sustain; per rep: rep1 did_not_sustain Hz, rep2 no sidecar Hz, rep3 no sidecar Hz, rep4 no sidecar Hz, rep5 no sidecar Hz

Per-rep p50s are shown only where the raw .bin files are present in this copy; otherwise only the per-rep rates are known (from the .rate sidecars).

