//! Shared helpers for the round-trip latency bench binaries
//! (`benches/latency/native`).
//!
//! ONE tree, THREE pacing modes selected by `CER_BENCH_PACING` — this
//! replaces the duplicated `benches/cerulion_round_trip` (back-to-back)
//! and `benches/cerulion_round_trip_quiescent` sibling folders:
//!
//! - **`quiescent`** (default, the sensor-rate REALISM mode) — every
//!   iteration is paced to a per-payload target rate
//!   ([`quiescent_schedule`]) via [`RateLimiter`], modelling a robot
//!   whose topics tick at sensor rates with idle gaps between messages.
//!   This measures the latency a robotics user actually observes,
//!   wake-up cost included.
//! - **`fixed100`** (the uniform-rate mode) —
//!   ONE target rate, [`FIXED100_RATE_HZ`] = 100 Hz, at EVERY payload
//!   size ([`FIXED100_TOTAL`] = 2100 / [`FIXED100_WARMUP`] = 100 →
//!   2000 measured, tail-resolved). The field convention for a payload
//!   sweep (see `METHODOLOGY.md` §17): a
//!   size-dependent schedule conflates payload-flatness with the
//!   idleness tax the rate axis carries (~2.2× p50 on box-jetson), so
//!   the uniform rate makes that tax a CONSTANT across sizes. A size
//!   that cannot SUSTAIN the target steps down the fallback ladder
//!   ([`fixed100_ladder`]: 100→50→20→sensor-rate floor) and RE-RUNS;
//!   the ACHIEVED rate is recorded in a `.rate` sidecar next to the
//!   `.bin` ([`dump_achieved_rate`]) so no mixed-rate line is ever
//!   silent. An exhausted ladder mints NO latency — a
//!   `did_not_sustain` sidecar and no `.bin` ([`dump_did_not_sustain`]).
//! - **`backtoback`** (SECONDARY) — uncapped tight ping-pong, the classic
//!   middleware saturation shape: as soon as an echo lands the next ping
//!   goes out. Keeps caches and CPU governors hot, so it understates the
//!   quiescent wake-up cost — useful as a floor/saturation cross-check,
//!   never as the headline.
//!
//! Exposes:
//! - [`PacingMode`] / [`bench_plan`] / [`BenchPlan`] — the per-payload
//!   iteration plan (pacer + measured/warmup counts) for the active mode.
//! - [`run_sized_bench`] — the shared per-size driver every bin calls;
//!   owns the fixed100 fallback ladder + sidecar writes in ONE place.
//! - [`RateLimiter`] — hybrid sleep+busywait pacer (paced modes); counts
//!   the grid slots it SKIPS ([`RateLimiter::slots_skipped`]) — the
//!   fixed100 sustain observable.
//! - [`quiescent_schedule`] / [`fixed100_ladder`] — the pinned schedules.
//! - [`dump_raw_samples`] — raw-sample `.bin` dump; the bench binaries'
//!   main output (percentiles are computed offline). fixed100 adds the
//!   [`dump_achieved_rate`] / [`dump_did_not_sustain`] `.rate` sidecar.
//! - [`wall_ns`] — the shared wall-clock (`CLOCK_MONOTONIC`) ns stamp.
//! - [`zenoh_shm_config`] — shared zenoh `Config` for the ping/pong split.
//!
//! Every binary acquires the CPU DMA-latency lock via [`acquire_dma_lock`]
//! for the duration of the run (C-state pin; no-op on non-Linux) and fails
//! fast if `/dev/cpu_dma_latency` exists but is not writable — without the
//! lock, C-state exit latency contaminates p99/p99.9 tails.
//! `CER_BENCH_ALLOW_NO_DMA_LOCK=1` downgrades the failure to a loud warning
//! for hosts where root access is unavailable; such runs are NOT citable
//! for tail percentiles (same escape-hatch pattern as
//! `CER_BENCH_ALLOW_UNVERIFIED_SHM` on the ROS 2 side).
//!
//! ## Env contract (shared with `ros2/run_bench.sh`,
//! `workspace/run_workspace.sh`, and `bench.py`)
//!
//! | var | meaning |
//! |---|---|
//! | `CER_BENCH_RAW_DUMP_DIR` | REQUIRED — directory the `.bin` dumps land in |
//! | `CER_BENCH_RAW_NAME` | REQUIRED — raw_prefix; files are `${NAME}_<payloadbytes>.bin` (fixed100 also writes a `${NAME}_<payloadbytes>.rate` achieved-rate sidecar) |
//! | `CER_BENCH_PACING` | `quiescent` (default), `fixed100`, or `backtoback` |
//! | `CER_BENCH_SMOKE_N` | smoke override of (total, warmup) keeping pacing |
//! | `CER_BENCH_TARGET_SAMPLES` | MEASURED samples — the suite-wide contract (every component treats it as measured, i.e. total − warmup). Consulted here only in backtoback mode (default 10000); quiescent counts come from the pinned schedule |
//! | `CER_BENCH_WARMUP` | backtoback warmup iterations (default 1000) |
//! | `CER_BENCH_TARGET_RATE_HZ` | rate for [`RateLimiter::from_env`] (per-payload, one-size-per-invocation callers) |
//! | `CER_BENCH_ALLOW_NO_DMA_LOCK` | `1` = run without the C-state lock when the device is unwritable (loud; tails not citable) — see [`acquire_dma_lock`] |
//! | `CER_BENCH_DMA_LOCK` | `1` (default) = acquire the lock; `0` = **stock posture**: skip the lock entirely even when the device is writable (loud; the untuned-host shape). The SAME knob bench.py + run_workspace.sh honor — see [`acquire_dma_lock`] |

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Pacing mode
// ---------------------------------------------------------------------------

/// Which pacing discipline the bench loop runs under. Read from
/// `CER_BENCH_PACING` — see the module docs for the two modes' semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacingMode {
    /// Rate-limited to the per-payload [`quiescent_schedule`] (default).
    Quiescent,
    /// Rate-limited to ONE uniform target ([`FIXED100_RATE_HZ`] = 100 Hz)
    /// at every payload size, with the [`fixed100_ladder`] fallback for a
    /// size that cannot sustain it.
    Fixed100,
    /// Uncapped tight loop — the saturation shape.
    Backtoback,
}

impl PacingMode {
    /// Read `CER_BENCH_PACING`. Absent → `Quiescent` (the primary mode).
    /// Anything other than exactly `"quiescent"` / `"fixed100"` /
    /// `"backtoback"` panics — a typo'd mode must never silently fall
    /// back to a default and dump samples labelled with the wrong pacing.
    pub fn from_env() -> Self {
        match env::var("CER_BENCH_PACING") {
            Err(env::VarError::NotPresent) => PacingMode::Quiescent,
            Err(e) => panic!("CER_BENCH_PACING is set but not valid UTF-8: {e}"),
            Ok(v) => match v.as_str() {
                "quiescent" => PacingMode::Quiescent,
                "fixed100" => PacingMode::Fixed100,
                "backtoback" => PacingMode::Backtoback,
                other => panic!(
                    "CER_BENCH_PACING={other:?} is not a pacing mode — valid values: \
                     \"quiescent\" (default), \"fixed100\", \"backtoback\""
                ),
            },
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            PacingMode::Quiescent => "quiescent",
            PacingMode::Fixed100 => "fixed100",
            PacingMode::Backtoback => "backtoback",
        }
    }
}

/// Per-payload iteration plan for the active [`PacingMode`]: how many
/// warmup and measured iterations to run, and how to pace them.
///
/// Built by [`bench_plan`]. The bench binaries call [`BenchPlan::wait_next`]
/// at the top of every iteration (warmup and measured alike); in
/// `backtoback` mode it returns immediately.
pub struct BenchPlan {
    pub mode: PacingMode,
    /// Target pacing rate — `Some` in quiescent mode, `None` in backtoback.
    pub rate_hz: Option<u64>,
    /// Measured iterations (== samples dumped to the `.bin`).
    pub measured: u64,
    /// Warmup iterations (discarded, not dumped).
    pub warmup: u64,
    limiter: Option<RateLimiter>,
}

impl BenchPlan {
    /// Block until the next iteration may start. Quiescent/fixed100:
    /// paces to the plan rate via [`RateLimiter`]. Backtoback: returns
    /// immediately.
    pub fn wait_next(&mut self) {
        if let Some(l) = &mut self.limiter {
            l.wait_next();
        }
    }

    /// One-line per-size progress summary, identical across all bench
    /// binaries so sweep logs read uniformly in every pacing mode.
    pub fn summary(&self) -> String {
        match self.rate_hz {
            Some(hz) => format!(
                "pacing={} rate={hz:>4} Hz measured={:>5} (warmup={})",
                self.mode.as_str(),
                self.measured,
                self.warmup
            ),
            None => format!(
                "pacing=backtoback measured={:>5} (warmup={})",
                self.measured, self.warmup
            ),
        }
    }

    /// Grid slots the pacer had to SKIP because a deadline was already
    /// past when `wait_next` ran (see [`RateLimiter`]'s missed-slot
    /// policy). 0 for backtoback (no pacer). This is the direct
    /// observable for "the loop could not fire at the target rate": a
    /// blocking round-trip loop never loses a sample, so an unsustainable
    /// rate manifests ONLY as skipped slots.
    pub fn slots_skipped(&self) -> u64 {
        self.limiter.as_ref().map_or(0, |l| l.slots_skipped())
    }

    /// fixed100 sustain verdict for the run this plan just paced:
    /// skipped slots within [`FIXED100_SKIP_TOLERANCE_PERCENT`] of the
    /// total (warmup + measured) iterations. See [`fixed100_sustained`].
    pub fn sustained_target_rate(&self) -> bool {
        fixed100_sustained(self.slots_skipped(), self.warmup + self.measured)
    }
}

/// Build the per-payload [`BenchPlan`] for the active `CER_BENCH_PACING`.
///
/// - `quiescent`: (rate, total, warmup) from [`quiescent_schedule`];
///   measured = total − warmup. `CER_BENCH_TARGET_SAMPLES`/`CER_BENCH_WARMUP`
///   are deliberately NOT consulted here — the quiescent counts are the
///   pinned schedule (or the smoke override), matching the retired
///   quiescent tree byte-for-byte.
/// - `backtoback`: measured = `CER_BENCH_TARGET_SAMPLES` (default 10 000),
///   warmup = `CER_BENCH_WARMUP` (default 1 000), no pacer. The
///   `CER_BENCH_SMOKE_N` override wins over both (same (total, warmup)
///   shape as quiescent smoke, pacing preserved — i.e. still uncapped).
pub fn bench_plan(payload_size: usize) -> BenchPlan {
    let mode = PacingMode::from_env();
    match mode {
        PacingMode::Quiescent => {
            let (rate_hz, total, warmup) = quiescent_schedule(payload_size);
            assert!(
                total > warmup,
                "quiescent schedule for payload {payload_size}: total ({total}) must exceed warmup ({warmup})"
            );
            BenchPlan {
                mode,
                rate_hz: Some(rate_hz),
                measured: total - warmup,
                warmup,
                limiter: Some(RateLimiter::new(rate_hz)),
            }
        }
        // fixed100: the plan at the TARGET rate. Ladder-driving callers
        // ([`run_sized_bench`]) build fallback-rung plans via
        // [`fixed100_plan`] directly.
        PacingMode::Fixed100 => fixed100_plan(FIXED100_RATE_HZ),
        PacingMode::Backtoback => {
            let (measured, warmup) = if let Some((total, warmup)) = smoke_override() {
                (total - warmup, warmup)
            } else {
                let measured = env_u64("CER_BENCH_TARGET_SAMPLES", 10_000);
                let warmup = env_u64("CER_BENCH_WARMUP", 1_000);
                assert!(
                    measured >= 2,
                    "CER_BENCH_TARGET_SAMPLES must be >= 2 — offline percentile \
                     extraction (read_p50_ns) needs at least 2 samples"
                );
                // Upper bound too: `measured` reaches `Vec::with_capacity`
                // in every bin, so an absurd value aborts the process
                // instead of refusing the run. Same rule and ceiling as
                // the ROS 2 nodes' common.hpp::check_sample_budget and the
                // workspace latency node, so all three stacks refuse the
                // same inputs (100e6 samples = 800 MB of u64).
                assert!(
                    warmup <= MAX_TOTAL_SAMPLES && measured <= MAX_TOTAL_SAMPLES - warmup,
                    "CER_BENCH_TARGET_SAMPLES={measured} + CER_BENCH_WARMUP=\
                     {warmup} exceeds the {MAX_TOTAL_SAMPLES}-sample ceiling \
                     — refusing to run a measurement whose capacity would \
                     wrap or exhaust memory"
                );
                (measured, warmup)
            };
            BenchPlan {
                mode,
                rate_hz: None,
                measured,
                warmup,
                limiter: None,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// fixed100 — the uniform-rate mode
// ---------------------------------------------------------------------------

/// fixed100 uniform target rate — 100 Hz at EVERY payload size (the
/// field-modal rate for a payload sweep; Part 2 of the
/// shielding findings, archived internally). **Lockstep in
/// four places** with [`FIXED100_TOTAL`] /
/// [`FIXED100_WARMUP`]: this file, `../bench.py` (`fixed100_schedule`),
/// `../ros2/run_bench.sh` (the fixed100 arm), and
/// `../workspace/run_workspace.sh` (`FIXED100_*`).
pub const FIXED100_RATE_HZ: u64 = 100;
/// fixed100 total iterations per size (uniform; measured = total − warmup
/// = 2000 — tail-resolved: a single-rep p99 rests on 20 exceedances).
pub const FIXED100_TOTAL: u64 = 2_100;
/// fixed100 warmup iterations per size (discarded, not dumped).
pub const FIXED100_WARMUP: u64 = 100;

/// Sustain tolerance: a fixed100 run counts as SUSTAINED when the pacer
/// skipped at most this percentage of its total (warmup + measured) grid
/// slots. 1 % of 2100 iterations = 21 slots ≈ 210 ms of cumulative stall
/// at 100 Hz — generous for OS jitter on a bench machine, while a rate the
/// transport genuinely cannot hold (round trip > period) skips nearly
/// EVERY slot and lands orders of magnitude past it.
pub const FIXED100_SKIP_TOLERANCE_PERCENT: u64 = 1;

/// Pure sustain verdict — see [`FIXED100_SKIP_TOLERANCE_PERCENT`].
pub fn fixed100_sustained(slots_skipped: u64, total_iterations: u64) -> bool {
    slots_skipped * 100 <= total_iterations * FIXED100_SKIP_TOLERANCE_PERCENT
}

/// The fixed100 fallback ladder for one payload size: the 100 Hz target,
/// then 50 Hz, then 20 Hz, then the size's SENSOR rate (its
/// [`quiescent_schedule`] rate) when that sits BELOW the 20 Hz rung —
/// strictly descending, so today only 16 MiB (sensor 10 Hz) gains a 4th
/// rung. A size that cannot sustain a rung is RE-RUN at the next one
/// (fresh warmup + measured); the counts stay [`FIXED100_TOTAL`] /
/// [`FIXED100_WARMUP`] at every rung so the per-size measured count
/// (2000) is rate-independent. **Lockstep in three places:** this
/// function, `../bench.py::fixed100_ladder` (drives the ROS 2 cells),
/// and `../workspace/run_workspace.sh::ladder_for`.
pub fn fixed100_ladder(payload_size: usize) -> Vec<u64> {
    let mut rungs = vec![FIXED100_RATE_HZ, 50, 20];
    let (sensor_rate, _total, _warmup) = quiescent_schedule(payload_size);
    if sensor_rate < *rungs.last().expect("ladder is non-empty") {
        rungs.push(sensor_rate);
    }
    rungs
}

/// Build a fixed100 [`BenchPlan`] at one ladder rung. Counts are the
/// uniform [`FIXED100_TOTAL`] / [`FIXED100_WARMUP`] (smoke-overridable —
/// `CER_BENCH_SMOKE_N` replaces (total, warmup) with (N, max(N/10, 1)),
/// same rule as the quiescent schedule).
pub fn fixed100_plan(rate_hz: u64) -> BenchPlan {
    let (total, warmup) = smoke_override().unwrap_or((FIXED100_TOTAL, FIXED100_WARMUP));
    assert!(
        total > warmup,
        "fixed100 counts: total ({total}) must exceed warmup ({warmup})"
    );
    BenchPlan {
        mode: PacingMode::Fixed100,
        rate_hz: Some(rate_hz),
        measured: total - warmup,
        warmup,
        limiter: Some(RateLimiter::new(rate_hz)),
    }
}

/// Drive ONE payload size of a bench binary under the active pacing mode
/// — the shared per-size driver both native bins call, so the fixed100
/// fallback ladder lives in exactly one place.
///
/// `run` executes one full (warmup + measured) pass under the given plan
/// and returns the measured samples; it must call `plan.wait_next()` at
/// the top of every iteration. Non-fixed100 modes: one pass, dump, done
/// (any stale `.rate` sidecar is removed so a fixed100 leftover can never
/// mislabel this run's rows). fixed100: walk [`fixed100_ladder`]; the
/// first SUSTAINED rung ([`BenchPlan::sustained_target_rate`]) dumps its
/// samples plus a `.rate` sidecar recording the ACHIEVED rate (a fallback
/// rung is announced loudly — never a silent mixed-rate line); an
/// exhausted ladder dumps NO `.bin` and a `did_not_sustain` sidecar —
/// a latency number is never minted from a run that could not hold any
/// rung (Principle #13).
pub fn run_sized_bench<F>(payload_size: usize, mut run: F)
where
    F: FnMut(&mut BenchPlan) -> Vec<u64>,
{
    if PacingMode::from_env() != PacingMode::Fixed100 {
        let _ = std::fs::remove_file(rate_sidecar_path(payload_size));
        let mut plan = bench_plan(payload_size);
        eprint!("  payload={payload_size:>10} B  {} ... ", plan.summary());
        let samples = run(&mut plan);
        let n = samples.len();
        // The native lane's counterpart to the two runners' delivery-
        // receipt gates. It emits NO receipts and needs none: there is no
        // ping/pong/latency chain to corroborate — one process owns both
        // ends of the round trip, and a sample exists only because a
        // reply came back, so the count IS the accounting. What the
        // receipt gates catch elsewhere (a set of counters that cannot
        // describe the .bin) reduces here to one comparison, and it was
        // unchecked: `run` returning short still dumped whatever it had,
        // and every downstream reader keys off the file.
        let want = plan.measured as usize;
        assert_eq!(
            n, want,
            "native bench produced {n} samples but the plan measured {want} \
             — refusing to dump a .bin the plan cannot describe"
        );
        dump_raw_samples(&samples, payload_size);
        eprintln!("dumped {n} samples");
        return;
    }

    let ladder = fixed100_ladder(payload_size);
    for &rung in &ladder {
        let mut plan = fixed100_plan(rung);
        eprint!("  payload={payload_size:>10} B  {} ... ", plan.summary());
        let samples = run(&mut plan);
        let skipped = plan.slots_skipped();
        if plan.sustained_target_rate() {
            let n = samples.len();
            dump_raw_samples(&samples, payload_size);
            dump_achieved_rate(payload_size, rung, &ladder);
            if rung == FIXED100_RATE_HZ {
                eprintln!("dumped {n} samples");
            } else {
                eprintln!(
                    "dumped {n} samples — FALLBACK: achieved {rung} Hz, not the \
                     {FIXED100_RATE_HZ} Hz target (recorded in the .rate sidecar; \
                     plots annotate the point)"
                );
            }
            return;
        }
        eprintln!(
            "NOT SUSTAINED at {rung} Hz — skipped {skipped} of {} grid slots \
             (> {FIXED100_SKIP_TOLERANCE_PERCENT}%); stepping down the ladder",
            plan.warmup + plan.measured
        );
    }
    // Order matters, and the safe order is verdict FIRST. A REUSED output
    // dir can still hold a `.bin` from an earlier run of this same size,
    // and `did_not_sustain` means NO latency was minted — so a surviving
    // `.bin` would be read as this run's samples. Both sibling ladder
    // implementations delete it (`run_workspace.sh`'s exhaustion arm,
    // `bench.py::run_ros2_cell_payload`); this is the third.
    //
    // But the removal can fail, and where it does the sidecar must ALREADY
    // be on disk. Removing first and panicking on failure leaves the
    // earlier run's `.bin` beside the earlier run's `.rate` — a mutually
    // CONSISTENT pair with no exhaustion marker at all, which compile_csv
    // reads as a healthy cell and ships as this run's percentiles, silently.
    // Writing the verdict first means a failed removal leaves the
    // CONTRADICTORY pair instead, which compile_csv refuses loudly per rep.
    // Fail closed: the worse artifact is the one nothing can detect.
    dump_did_not_sustain(payload_size, &ladder);
    let stale = raw_dump_path(payload_size);
    match std::fs::remove_file(&stale) {
        Ok(()) => eprintln!(
            "  removed a stale {} from an earlier run of this size",
            stale.display()
        ),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => panic!(
            "fixed100 ladder exhausted at payload {payload_size}; the \
             did_not_sustain sidecar is written, but the stale {} could not \
             be removed: {e}. That pair is contradictory and compile_csv \
             refuses it — remove the file and re-run rather than compiling \
             this cell.",
            stale.display()
        ),
    }
    eprintln!(
        "  payload={payload_size:>10} B  DID NOT SUSTAIN the fixed100 ladder \
         {ladder:?} Hz — NO latency minted (did_not_sustain sidecar written; \
         the row renders empty)"
    );
}

/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${size}.rate` — the
/// fixed100 achieved-rate sidecar beside the `.bin`. First line: the
/// achieved integer Hz, or the literal `did_not_sustain`; later `#` lines
/// are provenance (readers consume the first line only —
/// `compile_csv.py` folds it into the `achieved_rate_hz` CSV column).
fn rate_sidecar_path(payload_size: usize) -> PathBuf {
    let dir = env::var("CER_BENCH_RAW_DUMP_DIR")
        .expect("CER_BENCH_RAW_DUMP_DIR must be set — sidecars land beside the .bin dumps");
    let name = env::var("CER_BENCH_RAW_NAME")
        .expect("CER_BENCH_RAW_NAME must be set — sidecars land beside the .bin dumps");
    PathBuf::from(dir).join(format!("{name}_{payload_size}.rate"))
}

fn write_rate_sidecar(payload_size: usize, first_line: &str, provenance: &str) {
    let path = rate_sidecar_path(payload_size);
    let body = format!("{first_line}\n# {provenance}\n");
    std::fs::write(&path, body)
        .unwrap_or_else(|e| panic!("write rate sidecar {}: {e}", path.display()));
}

/// Record the rate a fixed100 size ACTUALLY ran at (the sustained rung).
pub fn dump_achieved_rate(payload_size: usize, achieved_hz: u64, ladder: &[u64]) {
    write_rate_sidecar(
        payload_size,
        &achieved_hz.to_string(),
        &format!(
            "fixed100 achieved-rate sidecar: target={FIXED100_RATE_HZ}Hz \
             ladder={ladder:?}Hz"
        ),
    );
}

/// Record a fixed100 ladder exhaustion — the size ran at NO rung, so no
/// `.bin` exists and downstream renders the row as `did not sustain`.
pub fn dump_did_not_sustain(payload_size: usize, ladder: &[u64]) {
    write_rate_sidecar(
        payload_size,
        "did_not_sustain",
        &format!(
            "fixed100 ladder exhausted: target={FIXED100_RATE_HZ}Hz \
             ladder={ladder:?}Hz — no rung sustained within \
             {FIXED100_SKIP_TOLERANCE_PERCENT}% skipped slots; no latency minted"
        ),
    );
}

/// Parse an optional non-negative integer env var with a default.
/// Loud on garbage — a mistyped count must not silently become the default.
fn env_u64(name: &str, default: u64) -> u64 {
    match env::var(name) {
        Err(env::VarError::NotPresent) => default,
        Err(e) => panic!("{name} is set but not valid UTF-8: {e}"),
        Ok(v) => v
            .parse()
            .unwrap_or_else(|_| panic!("{name} must be a non-negative integer, got {v:?}")),
    }
}

/// `CER_BENCH_SMOKE_N` override: replaces the per-payload
/// (total, warmup) with `(N, max(N/10, 1))` while the PACING stays on
/// whatever mode is active — `bench.py smoke` uses this to run a low-n
/// sanity pass without changing the pacing characteristics.
/// Ceiling on any env-supplied sample budget: 100e6 u64 samples is 800 MB,
/// four orders of magnitude above the widest pinned schedule. Shared by the
/// smoke override and the back-to-back env arm so one edit moves both, and
/// deliberately the same number as the ROS 2 nodes'
/// `common.hpp::check_sample_budget` and the workspace latency node — all
/// three stacks refuse the same inputs.
const MAX_TOTAL_SAMPLES: u64 = 100_000_000;

fn smoke_override() -> Option<(u64, u64)> {
    // Three arms, not `.ok()?`: `var` folds a present-but-non-UTF-8
    // value into the same `Err` as absence, so `CER_BENCH_SMOKE_N=$'\xff'`
    // silently ran the FULL schedule under a smoke invocation — 55_000
    // samples where 8 were asked for, with no complaint. Every other
    // refusal-deciding read in this file already separates the two
    // (PacingMode::from_env, env_u64, sweep_payload_sizes); this one did
    // not, and it is the knob that decides the sample counts.
    let v = match env::var("CER_BENCH_SMOKE_N") {
        Err(env::VarError::NotPresent) => return None,
        Err(e) => panic!("CER_BENCH_SMOKE_N is set but not valid UTF-8: {e}"),
        Ok(v) => v,
    };
    let n: u64 = v
        .parse()
        .expect("CER_BENCH_SMOKE_N must be a positive integer");
    // n <= 2 leaves only 1 measurement sample after warmup (8 bytes);
    // read_p50_ns requires >= 2 samples (16 bytes). Minimum safe value is 3.
    assert!(
        n > 2,
        "CER_BENCH_SMOKE_N must be > 2 (minimum safe value is 3)"
    );
    // Ceiling as well as floor, and HERE rather than at the call sites:
    // this override wins in EVERY pacing arm (quiescent, fixed100 and
    // backtoback all consult it first), and its total reaches
    // `Vec::with_capacity(plan.measured)` in every bin. Guarding only the
    // env arm would leave `CER_BENCH_SMOKE_N=100000000000` aborting the
    // process instead of refusing the run. Same ceiling as the ROS 2
    // nodes' check_sample_budget and the workspace latency node.
    assert!(
        n <= MAX_TOTAL_SAMPLES,
        "CER_BENCH_SMOKE_N={n} exceeds the {MAX_TOTAL_SAMPLES}-sample \
         ceiling — refusing to run a measurement whose capacity would \
         exhaust memory"
    );
    Some((n, (n / 10).max(1)))
}

// ---------------------------------------------------------------------------
// Rate limiter (quiescent pacing)
// ---------------------------------------------------------------------------

/// Rate limiter — paces iteration to a target Hz with hybrid sleep+busywait.
///
/// Deadlines live on a fixed epoch grid: the first call to `wait_next`
/// captures the `t0` epoch (so bootstrap variability doesn't drift the
/// schedule) and iteration `k` fires at `t0 + period * k`.
///
/// **Missed-slot policy — suite-wide:** when a deadline is
/// already in the past (the previous round trip overran one or more
/// slots), `wait_next` advances the iteration index to the next FUTURE
/// slot on the `t0` grid — `ceil(elapsed / period)` — instead of
/// returning immediately. A k-period stall therefore never produces k
/// back-to-back catch-up iterations (which would enter the quiescent
/// sample population as saturation-mode samples, without the wake-up
/// cost quiescent pacing exists to include), and anchoring on the `t0`
/// grid preserves phase (no fire-time re-anchor rate droop). This is
/// the SAME skip policy the ROS 2 cells apply to kick pacing (the
/// fix: `latency_node_rcl.cpp`'s `next_kick = now + period` and
/// `latency_node.cpp`'s reliance on `rcl_timer`'s whole-period clamp)
/// — one policy across every paced line in the suite. Note the C++
/// `RateLimiter` in `../ros2/ros2_rtt_bench/src/common.hpp` is NOT the
/// live ROS 2 pacing mechanism (kick pacing is timer/deadline-based);
/// this Rust pacer is the only `RateLimiter` on a measured path.
///
/// Hybrid: coarse sleep until ~50 µs short of deadline (avoids the
/// `clock_nanosleep` ~50 µs accuracy floor dominating measurements),
/// then busy-wait the rest. CPU cost: ~5% at 1 kHz, negligible at 10 Hz.
pub struct RateLimiter {
    rate_hz: u64,
    period: Duration,
    iter: u64,
    t0: Option<Instant>,
    /// Grid slots skipped by the missed-slot policy across the limiter's
    /// lifetime — the fixed100 sustain observable ([`fixed100_sustained`]).
    slots_skipped: u64,
}

/// Ceiling on any target rate, in Hz: above 1 GHz the wall period
/// `1_000_000_000 / rate_hz` floors to ZERO nanoseconds, and a zero period
/// gates nothing — the run free-runs while every artifact still carries the
/// REQUESTED rate as its label (the mislabeled-figure hazard the wall-grid
/// gate exists to remove). The workspace ping node and the three ROS 2
/// nodes refuse the same input for the same reason.
const MAX_RATE_HZ: u64 = 1_000_000_000;

impl RateLimiter {
    /// Construct directly with a known rate. The native bench binaries
    /// loop over multiple payload sizes within a single invocation
    /// (unlike the ROS2 binaries, which are one-size-per-invocation), so
    /// [`bench_plan`] looks the rate up via [`quiescent_schedule`] per
    /// size and constructs a fresh `RateLimiter` per payload.
    pub fn new(rate_hz: u64) -> Self {
        assert!(rate_hz > 0, "RateLimiter::new: rate_hz must be > 0");
        // A rate above 1 GHz makes `1e9 / rate_hz` floor to a ZERO
        // nanosecond period. `next_grid_slot` does panic on that — with a
        // bare "attempt to divide by zero", one wait_next() into the run
        // and naming nothing. Refuse here instead, where the offending
        // value is in hand, so the message says which knob to fix.
        assert!(
            rate_hz <= MAX_RATE_HZ,
            "RateLimiter::new: rate_hz {rate_hz} exceeds {MAX_RATE_HZ} Hz — \
             the wall period 1e9/rate_hz would floor to 0 ns and the grid \
             would gate nothing while the run still carries the requested \
             rate as its label"
        );
        Self {
            rate_hz,
            period: Duration::from_nanos(1_000_000_000 / rate_hz),
            iter: 0,
            t0: None,
            slots_skipped: 0,
        }
    }

    /// Read `CER_BENCH_TARGET_RATE_HZ` from env. Panics if unset or 0 —
    /// used by one-payload-per-invocation callers which inherit the rate
    /// from the runner's per-payload schedule.
    pub fn from_env() -> Self {
        let rate_hz: u64 = env::var("CER_BENCH_TARGET_RATE_HZ")
            .expect("CER_BENCH_TARGET_RATE_HZ must be set for quiescent bench")
            .parse()
            .expect("CER_BENCH_TARGET_RATE_HZ must be a positive integer");
        Self::new(rate_hz)
    }

    pub fn rate_hz(&self) -> u64 {
        self.rate_hz
    }

    /// Total grid slots skipped so far (see the missed-slot policy in the
    /// type docs). Each skipped slot is a scheduled fire the loop was too
    /// late for — a direct measure of "could not run at the target rate".
    pub fn slots_skipped(&self) -> u64 {
        self.slots_skipped
    }

    /// Block until the next iteration's tick. Call after recording the
    /// previous sample, before issuing the next round-trip. The first
    /// call captures `t0` and waits until the grid's first slot
    /// (`t0 + period`).
    ///
    /// Missed-slot policy (see the type docs): a deadline already in
    /// the past advances `iter` to the next future `t0`-grid slot
    /// (`ceil(elapsed / period)`) rather than returning immediately —
    /// no post-stall burst, phase preserved.
    pub fn wait_next(&mut self) {
        let t0 = *self.t0.get_or_insert_with(Instant::now);
        self.iter += 1;
        let on_time_iter = self.iter;
        let now = Instant::now();
        // Grid-anchored skip: if the previous round trip
        // overran this slot's deadline, jump to the next slot still in
        // the future instead of firing immediately — firing immediately
        // would run the backlog back-to-back and admit saturation-mode
        // samples into the quiescent population (the ROS 2 cells' F1.6
        // skip policy, mirrored here). Policy arithmetic is the pure,
        // oracle-tested [`next_grid_slot`].
        self.iter = next_grid_slot(
            self.iter,
            now.duration_since(t0).as_nanos(),
            self.period.as_nanos(),
        );
        // Every slot the policy jumped over is a scheduled fire the loop
        // missed — the fixed100 sustain observable.
        self.slots_skipped += self.iter - on_time_iter;
        let deadline = t0 + self.period * (self.iter as u32);
        if let Some(remaining) = deadline.checked_duration_since(now) {
            if remaining > Duration::from_micros(100) {
                thread::sleep(remaining - Duration::from_micros(50));
            }
            while Instant::now() < deadline {
                std::hint::spin_loop();
            }
        }
    }
}

/// Pure half of the grid-anchored missed-slot skip (see
/// [`RateLimiter`]'s type docs for the suite-wide policy statement).
///
/// `iter` is the schedule index whose deadline (`t0 + period * iter`) the
/// caller is about to wait for; `elapsed_ns` is `now − t0`. If that
/// deadline is still in the future (or exactly now), the index is kept;
/// if it is already past, the returned index is the next FUTURE slot on
/// the `t0` grid — `ceil(elapsed / period)`. Both arms collapse into one
/// expression because `elapsed <= iter * period ⇒ ceil(elapsed / period)
/// <= iter`, so the `max` keeps on-time indices unchanged AND guarantees
/// the schedule index never moves backward.
///
/// `period_ns == 0` (a rate above 1 GHz) is unreachable from production:
/// [`RateLimiter::new`] refuses any rate above [`MAX_RATE_HZ`] before one
/// can be built. Were it ever reached, `div_ceil` panics rather than
/// silently misdistributing slots.
fn next_grid_slot(iter: u64, elapsed_ns: u128, period_ns: u128) -> u64 {
    iter.max(elapsed_ns.div_ceil(period_ns) as u64)
}

// ---------------------------------------------------------------------------
// Quiescent schedule
// ---------------------------------------------------------------------------

/// Per-payload (rate, samples, warmup) schedule for the quiescent mode.
///
/// Returns `(rate_hz, total_iterations, warmup_iterations)`. Sized for a
/// ~60 s measurement window at typical robotics sensor rates — 1 kHz for
/// control-loop-sized payloads (≤1 KB), 30 Hz for HD-camera-sized payloads
/// (~4 MB), 10 Hz for full lidar scans (16 MB) — EXCEPT the top two sizes,
/// whose windows are extended (~70 s at 4 MiB, ~205 s at 16 MiB) so the
/// measured count reaches 2 000 and a single-rep p99 is citable at EVERY
/// size (the ≥ 20-tail-exceedance rule; tail-resolved schedule).
///
/// **MUST stay in lockstep in FOUR places:** this function,
/// `../workspace/run_workspace.sh::schedule_for`,
/// `../ros2/run_bench.sh`'s per-payload case statement, and
/// `../bench.py::quiescent_schedule`. All four must produce identical
/// per-payload MEASURED counts (total − warmup) so cross-stack CSV rows
/// are apples-to-apples comparable.
///
/// **Smoke-gate override:** when `CER_BENCH_SMOKE_N` is set, the
/// per-payload (total, warmup) is replaced with `(N, max(N/10, 1))` while
/// the rate stays on the schedule — `bench.py smoke` uses this to run a
/// low-n sanity pass without changing the pacing characteristics. The
/// mirror override on the ROS2 side is `TARGET_SAMPLES`/`WARMUP` in
/// `run_bench.sh` (consumed by `bench.py smoke` via `docker run -e`).
pub fn quiescent_schedule(payload_size: usize) -> (u64, u64, u64) {
    let (rate_hz, total, warmup) = match payload_size {
        64 | 256 | 1024 => (1000, 60_000, 5_000),
        4_096 | 16_384 => (500, 30_000, 2_500),
        65_536 => (200, 12_000, 1_000),
        262_144 => (100, 6_000, 500),
        1_048_576 => (60, 3_600, 300),
        4_194_304 => (30, 2_100, 100),
        16_777_216 => (10, 2_050, 50),
        // Fallback for any out-of-table size: 10 Hz × 60 s = 600 samples.
        _ => (10, 600, 50),
    };
    if let Some((total, warmup)) = smoke_override() {
        return (rate_hz, total, warmup);
    }
    (rate_hz, total, warmup)
}

// ---------------------------------------------------------------------------
// Payload sweep
// ---------------------------------------------------------------------------

/// The pinned 10-size payload sweep (bytes) — mirrors
/// `bench.py::PAYLOAD_SIZES`.
pub const PAYLOAD_SIZES: &[usize] = &[
    64,
    256,
    1024,
    4 * 1024,
    16 * 1024,
    64 * 1024,
    256 * 1024,
    1024 * 1024,
    4 * 1024 * 1024,
    16 * 1024 * 1024,
];

/// The payload sizes this invocation sweeps: the pinned [`PAYLOAD_SIZES`]
/// by default, restricted by `CER_BENCH_PAYLOAD_SIZES` (whitespace- or
/// comma-separated byte counts) when set — the suite-wide env contract
/// (METHODOLOGY §11), used for partial re-sweeps (e.g. the 2026-08-13
/// tail-resolved 4/16 MiB re-run). Panics LOUDLY on garbage or an empty
/// restriction — a typo must never silently produce a full (or empty)
/// sweep under a partial-sweep label.
pub fn sweep_payload_sizes() -> Vec<usize> {
    match env::var("CER_BENCH_PAYLOAD_SIZES") {
        Err(env::VarError::NotPresent) => PAYLOAD_SIZES.to_vec(),
        Err(e) => panic!("CER_BENCH_PAYLOAD_SIZES is set but not valid UTF-8: {e}"),
        Ok(v) => {
            let sizes: Vec<usize> = v
                .split(|c: char| c.is_whitespace() || c == ',')
                .filter(|s| !s.is_empty())
                .map(|s| {
                    s.parse().unwrap_or_else(|_| {
                        panic!(
                            "CER_BENCH_PAYLOAD_SIZES must be whitespace/comma-separated \
                             byte counts, got {s:?} in {v:?}"
                        )
                    })
                })
                .collect();
            assert!(
                !sizes.is_empty(),
                "CER_BENCH_PAYLOAD_SIZES is set but empty — unset it for the pinned \
                 10-size sweep or name the sizes to run"
            );
            // Membership, not just numeracy. ONE INVENTORY across all
            // three stacks: every cell must be
            // comparable, parity and plots key on one size list, and a size
            // nobody can compare across stacks is refused rather than
            // measured. The other three mirrors of this override
            // (bench.py::ambient_payload_restriction, ros2/run_bench.sh,
            // workspace/run_workspace.sh) refuse the same set — the limit
            // is the CAMPAIGN's, not any one runner's, so it is
            // deliberately stricter than what a given transport could
            // carry on its own.
            for size in &sizes {
                assert!(
                    PAYLOAD_SIZES.contains(size),
                    "CER_BENCH_PAYLOAD_SIZES names {size}, which is not one of the \
                     suite's TEN PINNED SIZES {PAYLOAD_SIZES:?} — no other stack \
                     can be compared against it, so the row would answer a \
                     question nobody else answers. One inventory across all three \
                     stacks; see METHODOLOGY \
                     'ten pinned sizes'."
                );
            }
            sizes
        }
    }
}

// ---------------------------------------------------------------------------
// Raw sample dump
// ---------------------------------------------------------------------------

/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${size}.bin` — the
/// raw sample dump. Shared by the writer and by the fixed100 exhaustion
/// path, which must DELETE this file: the two outcomes are exclusive, so
/// the artifact of the one that did not happen must not survive.
fn raw_dump_path(payload_size: usize) -> PathBuf {
    let dir = env::var("CER_BENCH_RAW_DUMP_DIR").expect(
        "CER_BENCH_RAW_DUMP_DIR must be set — bench writes raw .bin only, no inline percentiles",
    );
    let name = env::var("CER_BENCH_RAW_NAME").expect(
        "CER_BENCH_RAW_NAME must be set — bench writes raw .bin only, no inline percentiles",
    );
    PathBuf::from(dir).join(format!("{name}_{payload_size}.bin"))
}

/// Write round-trip latency samples (LE u64 ns) to
/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${payload_size}.bin`.
///
/// The bench binary's only output is this raw sample stream — all
/// percentile / ECDF / HDR analysis happens post-hoc via
/// `compile_csv.py` (Python), per the suite principle "raw per-sample
/// dumps so analysis is one Python call away, not a re-run."
///
/// Panics if the env vars aren't set — fail-fast is correct here:
/// silently producing nothing means a sweep just lost its output.
pub fn dump_raw_samples(samples: &[u64], payload_size: usize) {
    let path = raw_dump_path(payload_size);
    let mut buf = Vec::with_capacity(samples.len() * 8);
    for &s in samples {
        buf.extend_from_slice(&s.to_le_bytes());
    }
    let mut f = File::create(&path)
        .unwrap_or_else(|e| panic!("dump_raw_samples: create {}: {e}", path.display()));
    f.write_all(&buf)
        .unwrap_or_else(|e| panic!("dump_raw_samples: write {}: {e}", path.display()));
}

// ---------------------------------------------------------------------------
// Wall clock
// ---------------------------------------------------------------------------

/// Acquire the CPU DMA-latency lock (C-state pin) — or refuse to run.
///
/// The lock is what keeps C-state exit latency out of the measured tails
/// (the May-2026 campaign measured 30–59% p50 inflation and multi-µs p99
/// spikes without it), so a failed acquisition is a hard, loud failure by
/// default: silently measuring without it produces tails that look like
/// middleware jitter but are the OS.
///
/// `CER_BENCH_ALLOW_NO_DMA_LOCK=1` downgrades to a loud warning and
/// returns `None` — for hosts where `/dev/cpu_dma_latency` cannot be made
/// writable. Such runs are NOT citable for p99/p99.9.
///
/// `CER_BENCH_DMA_LOCK=0` (the suite-wide STOCK-posture knob — the same
/// variable bench.py and run_workspace.sh honor) skips the lock entirely,
/// even when the device is writable: the deliberate untuned-host shape,
/// announced loudly. Distinct from `CER_BENCH_ALLOW_NO_DMA_LOCK`, which
/// only tolerates a FAILED acquisition. Any value other than `0`/`1` is
/// an env-contract violation (loud panic — a silently misread posture is
/// a mislabeled-figure hazard).
///
/// Prints `cpu_dma_lock: ok` / `cpu_dma_lock: SKIPPED` to stderr so runner
/// logs record which regime a `.bin` was captured under.
pub fn acquire_dma_lock() -> Option<cerulion_core::dma_lock::CpuDmaLock> {
    match std::env::var("CER_BENCH_DMA_LOCK") {
        Ok(v) if v == "0" => {
            // The CONTRACT (METHODOLOGY §11) says a contradictory
            // CERULION_CPU_DMA_LOCK export is REFUSED. Only bench.py
            // enforced that; a direct `cargo run` of a bench bin skipped
            // the lock and said nothing about the contradiction. Refuse
            // here too — asking for the stock posture while also exporting
            // the cap var is a declaration neither half of which can be
            // honoured, and guessing is the mislabeled-figure hazard the
            // knob exists to remove. No measured behaviour changes: these
            // bins never read the cap var (it is the CLI's), and bench.py
            // exports it only when the posture is tuned.
            // `var_os`, not `var`: the check is about PRESENCE, and
            // `var` folds a present-but-non-UTF-8 value into the same
            // `Err` as absence — so on Unix `CERULION_CPU_DMA_LOCK=$'\xff'`
            // skipped the refusal entirely and the run proceeded under a
            // posture it had just been told was contradictory. The other
            // two enforcement points already test presence (`is not None`
            // in bench.py, `${VAR+set}` in run_workspace.sh); this one
            // tested decodability. Non-Unicode values are reported
            // lossily rather than dropped — the operator needs to see
            // WHICH variable, and its exact bytes are not what is wrong.
            if let Some(cap) = std::env::var_os("CERULION_CPU_DMA_LOCK") {
                panic!(
                    "CER_BENCH_DMA_LOCK=0 (stock posture) but \
                     CERULION_CPU_DMA_LOCK is also exported (='{}') — \
                     contradictory C-state posture; unset one. A stock run \
                     must not inherit the cap var.",
                    cap.to_string_lossy()
                );
            }
            eprintln!(
                "cpu_dma_lock: SKIPPED (CER_BENCH_DMA_LOCK=0 — stock \
                 posture) — C-state exits unmanaged; tails NOT comparable \
                 to capped runs."
            );
            return None;
        }
        Ok(v) if v == "1" => {}
        Err(std::env::VarError::NotPresent) => {}
        Ok(other) => panic!(
            "CER_BENCH_DMA_LOCK must be 0 or 1 (got '{other}') — refusing \
             to guess the C-state posture"
        ),
        Err(e) => panic!("CER_BENCH_DMA_LOCK unreadable: {e}"),
    }
    match cerulion_core::dma_lock::cpu_dma_lock() {
        Ok(lock) => {
            eprintln!("cpu_dma_lock: ok");
            Some(lock)
        }
        Err(e) => {
            let allowed = std::env::var("CER_BENCH_ALLOW_NO_DMA_LOCK")
                .map(|v| v == "1")
                .unwrap_or(false);
            if allowed {
                eprintln!(
                    "cpu_dma_lock: SKIPPED (CER_BENCH_ALLOW_NO_DMA_LOCK=1) — {e}. \
                     p99/p99.9 tails from this run are NOT citable (C-state \
                     exit latency is unmanaged)."
                );
                None
            } else {
                panic!(
                    "cpu_dma_lock failed: {e}\n\
                     fix: make /dev/cpu_dma_latency writable — \
                     `sudo chmod 666 /dev/cpu_dma_latency` (until reboot) or a udev rule \
                     `KERNEL==\"cpu_dma_latency\", MODE=\"0666\"`;\n\
                     or set CER_BENCH_ALLOW_NO_DMA_LOCK=1 to run anyway \
                     (loudly; tails not citable)."
                );
            }
        }
    }
}

/// Wall-clock nanosecond stamp — `CLOCK_MONOTONIC`, via
/// `cerulion_core::clock::RealClock`.
///
/// This is the SAME clock the workspace bench nodes stamp into messages
/// under `CER_BENCH_WALL_STAMP=1` (`self.real_ns()` inside a
/// `#[cerulion_node]` tick), so a one-way latency computed as
/// `wall_ns() − stamp` at a receiver on the same host compares two
/// readings of one clock. NOT comparable across hosts (monotonic clocks
/// are per-boot, per-machine).
pub fn wall_ns() -> u64 {
    use cerulion_core::clock::{Clock, RealClock};
    RealClock.now_ns()
}

// ---------------------------------------------------------------------------
// zenoh config
// ---------------------------------------------------------------------------

/// The loopback endpoint the zenoh ping/pong pair meets on.
///
/// Shared with the initiator's readiness poll so the address it probes
/// cannot drift from the one the pong actually listens on.
pub const ZENOH_BENCH_ENDPOINT: &str = "127.0.0.1:7447";

/// Build the zenoh `Config` used by both bench processes.
///
/// `listen_endpoint = true` makes this peer bind `tcp/{ZENOH_BENCH_ENDPOINT}`
/// (see the const) and wait for the other peer to dial in.
/// `listen_endpoint = false` connects to that same address. One TCP
/// connection in total, bidirectional.
///
/// SHM transport tuning (matches zenoh's recommended setup for low-latency
/// SHM ping-pong, with two corrections to the defaults):
///
/// - `mode: "init"` — pre-initialises the SHM subsystem at session open
///   instead of lazily on first use, so the first iteration of the timing
///   loop doesn't pay the SHM-setup latency.
/// - `transport_optimization.message_size_threshold: 0` — the default 3 KB
///   threshold would silently send our 64 B / 1 KB / 4 KB messages over
///   TCP loopback; we want SHM at every payload size.
/// - `pool_size: 64 MiB` — must comfortably fit our 16 MiB largest payload
///   with room for ping + pong in flight.
///
/// Multicast and gossip scouting are disabled so the bench is deterministic
/// (no chance of discovering a stale zenoh peer on the host).
pub fn zenoh_shm_config(listen_endpoint: bool) -> zenoh::Config {
    let key = if listen_endpoint { "listen" } else { "connect" };
    let endpoints_block = format!(r#"{key}: {{ endpoints: ["tcp/{ZENOH_BENCH_ENDPOINT}"] }},"#);

    let json = format!(
        r#"{{
            mode: "peer",
            {endpoints_block}
            scouting: {{
                multicast: {{ enabled: false }},
                gossip: {{ enabled: false }},
            }},
            transport: {{
                shared_memory: {{
                    enabled: true,
                    mode: "init",
                    transport_optimization: {{
                        enabled: true,
                        pool_size: 67108864,
                        message_size_threshold: 0,
                    }},
                }},
            }},
        }}"#
    );

    zenoh::Config::from_json5(&json).expect("parse zenoh SHM bench config")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The missed-slot policy's arithmetic, pinned by
    /// oracle vectors on the pure [`next_grid_slot`] (period = 1000 ns
    /// throughout, so the grid math is readable). Kills the no-skip
    /// revert (stall vectors), an off-by-one over-skip (the exact-ceil
    /// pins, including a stall landing exactly ON a grid point), and any
    /// backward movement of the schedule index. `wait_next` routes every
    /// call through this function unconditionally, so the call site has
    /// no branch left to mutate.
    #[test]
    fn next_grid_slot_skips_to_the_next_future_slot_and_never_bursts() {
        // On time (deadline in the future): index unchanged.
        assert_eq!(next_grid_slot(3, 2_500, 1_000), 3);
        // Exactly AT the deadline: fire this slot, no skip.
        assert_eq!(next_grid_slot(3, 3_000, 1_000), 3);
        // 1 ns late: this slot is gone — next future slot, not a
        // fire-now burst (a fire-now policy would keep 3).
        assert_eq!(next_grid_slot(3, 3_001, 1_000), 4);
        // Multi-period stall (the burst-test shape): waiting for slot 2
        // with 4.5 periods elapsed lands on slot 5 — slots 2..4 are
        // SKIPPED, phase preserved on the t0 grid.
        assert_eq!(next_grid_slot(2, 4_500, 1_000), 5);
        // A stall ending exactly ON a grid point fires AT that point —
        // pins that the skip never over-shoots by one.
        assert_eq!(next_grid_slot(2, 4_000, 1_000), 4);
        // The schedule index never moves backward.
        assert_eq!(next_grid_slot(7, 1, 1_000), 7);
    }

    /// Behavioral pin for the missed-slot policy: after a stall spanning multiple
    /// grid slots, `wait_next` must SKIP the missed slots and wait for
    /// the next FUTURE `t0`-grid slot — never run the backlog
    /// back-to-back (a burst admits saturation-mode samples
    /// into the quiescent population). Oracle: consecutive `wait_next`
    /// returns after the stall stay >= period/2 apart; without the skip they are
    /// ~2 µs apart (reverting the skip fails here).
    /// A gap only shrinks below period/2 if the EARLIER return was
    /// delayed >= 20 ms past its deadline (wall-undershoot class) — the
    /// 40 ms period keeps that margin wide; runner load otherwise only
    /// LENGTHENS the gaps.
    #[test]
    fn a_stall_skips_missed_slots_instead_of_bursting() {
        let period = Duration::from_millis(40);
        let mut rl = RateLimiter::new(25); // 40 ms grid
        rl.wait_next(); // captures t0, waits for slot 1

        // Stall ~3.5 periods — slots 2, 3 and 4 are now in the past.
        thread::sleep(period * 7 / 2);

        let mut returns = Vec::new();
        for _ in 0..3 {
            rl.wait_next();
            returns.push(Instant::now());
        }
        for pair in returns.windows(2) {
            let gap = pair[1].duration_since(pair[0]);
            assert!(
                gap >= period / 2,
                "post-stall wait_next returns are {gap:?} apart — a \
                 catch-up burst (missed slots were not skipped)"
            );
        }

        // The skip COUNTER records the missed slots
        // the policy jumped over — a ~3.5-period stall skips at least 2
        // slots (timing gives 2–4 depending on where the wakeup lands).
        assert!(
            rl.slots_skipped() >= 2,
            "a 3.5-period stall must record >= 2 skipped slots, got {}",
            rl.slots_skipped()
        );
    }

    /// An on-schedule loop records ZERO skipped slots
    /// (the sustain verdict's negative control — a counter that ticks on
    /// healthy pacing would push every cell down the ladder).
    #[test]
    fn an_on_schedule_loop_skips_no_slots() {
        let mut rl = RateLimiter::new(50); // 20 ms grid — trivially holdable
        for _ in 0..10 {
            rl.wait_next();
        }
        assert_eq!(rl.slots_skipped(), 0);
    }

    /// The rate ceiling, pinned on both sides. Two arms, because they kill
    /// different variants: deleting the guard is caught only by the refusal,
    /// and widening `<=` to `<` only by the accept — the accept arm is also
    /// what stops a variant that moved the ceiling DOWN to 1 from passing.
    ///
    /// The value pin is separate and explicit. Without it the pair fixes
    /// only a RANGE: the load-bearing assertion is `1e9 / MAX == 1`, which
    /// holds for anything in `500_000_001..=1_000_000_000`, so a drift to
    /// `999_999_999` survives both arms. That range is enough for the
    /// property the guard exists for — every value in it still yields a
    /// period of at least 1 ns — but the constant is the number the other
    /// four pacers are written against, so it is pinned as a number too.
    #[test]
    fn a_rate_at_the_ceiling_is_accepted_with_a_nonzero_period() {
        assert_eq!(MAX_RATE_HZ, 1_000_000_000);
        let rl = RateLimiter::new(MAX_RATE_HZ);
        assert_eq!(rl.rate_hz(), MAX_RATE_HZ);
        assert_eq!(rl.period, Duration::from_nanos(1));
    }

    #[test]
    #[should_panic(expected = "exceeds")]
    fn a_rate_one_past_the_ceiling_is_refused() {
        let _ = RateLimiter::new(MAX_RATE_HZ + 1);
    }

    /// The fixed100 ladder oracle: 100 → 50 → 20, plus the size's
    /// sensor rate ONLY when it sits below the 20 Hz rung (strictly
    /// descending — today just 16 MiB @ 10 Hz, and the out-of-table
    /// fallback whose sensor rate is also 10 Hz).
    #[test]
    fn fixed100_ladder_appends_the_sensor_floor_only_below_20hz() {
        assert_eq!(fixed100_ladder(64), vec![100, 50, 20]); // sensor 1000
        assert_eq!(fixed100_ladder(262_144), vec![100, 50, 20]); // sensor 100
        assert_eq!(fixed100_ladder(4_194_304), vec![100, 50, 20]); // sensor 30
        assert_eq!(fixed100_ladder(16_777_216), vec![100, 50, 20, 10]); // sensor 10
        assert_eq!(fixed100_ladder(12_345), vec![100, 50, 20, 10]); // fallback row
    }

    /// The fixed100 sustain threshold, pinned on both sides at the
    /// shipped counts (2100 total → 21-slot budget at 1 %).
    #[test]
    fn fixed100_sustain_threshold_is_pinned_on_both_sides() {
        assert!(fixed100_sustained(0, FIXED100_TOTAL));
        assert!(fixed100_sustained(21, FIXED100_TOTAL)); // exactly AT 1 %
        assert!(!fixed100_sustained(22, FIXED100_TOTAL)); // one past it

        // An unsustainable rate skips nearly every slot — far past the gate.
        assert!(!fixed100_sustained(FIXED100_TOTAL, FIXED100_TOTAL));
    }

    /// The fixed100 counts: uniform (2100, 100) at every size, and
    /// the plan derives measured = 2000 at every rung (the CSV
    /// exact-count contract is rate-independent).
    #[test]
    fn fixed100_plan_counts_are_uniform_and_measured_is_2000() {
        for rung in fixed100_ladder(16_777_216) {
            let plan = fixed100_plan(rung);
            assert_eq!(plan.rate_hz, Some(rung));
            assert_eq!(plan.measured, FIXED100_TOTAL - FIXED100_WARMUP);
            assert_eq!(plan.measured, 2_000);
            assert_eq!(plan.warmup, FIXED100_WARMUP);
        }
    }
}
