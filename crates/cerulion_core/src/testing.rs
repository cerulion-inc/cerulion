// SPDX-License-Identifier: AGPL-3.0-only
//! Test helpers gated behind `#[cfg(any(test, feature = "test-helpers"))]`.
//!
//! Per-test iceoryx2 SHM root. Production iceoryx2 places its
//! on-disk bookkeeping at a process-shared root path (default
//! `/tmp/iceoryx2/`); parallel test threads in the same `cargo
//! test` process clash on this region. This module gives each
//! test its own iceoryx2 `Config` with a unique `prefix` so
//! iceoryx tests can run in parallel without conflicts (the
//! `--test-threads=1` rule documented in AGENTS.md is the
//! fallback for tests that haven't migrated yet).
//!
//! Usage in a test:
//!
//! ```no_run
//! # #[cfg(feature = "test-helpers")] {
//! use cerulion_core::testing::iceoryx_test_config;
//! use cerulion_core::{TransportManager, TransportConfig, VirtualClock};
//! use std::sync::Arc;
//!
//! let ix_config = iceoryx_test_config();
//! let config = TransportConfig {
//!     node_name: "test_node".into(),
//!     clock: Arc::new(VirtualClock::new()),
//!     subscriber_buffer_size: 4,
//!     network: None,
//! };
//! let mgr = TransportManager::init_for_test(config, ix_config).unwrap();
//! // ... run the test against `mgr` ...
//! # }
//! ```

/// Construct an iceoryx2 `Config` with a unique per-process
/// prefix and a test-only root directory.
///
/// Wraps `iceoryx2::testing::generate_isolated_config()` (which
/// is always public in iceoryx2 0.9 — no feature gate needed).
/// That helper sets `config.global.root_path` to a shared test
/// directory AND a per-call unique `prefix` so the on-disk
/// resources for this Cerulion test never collide with another
/// test's resources. The prefix is generated from
/// `UniqueSystemId` so it is unique even across threads in the
/// same process.
///
/// The returned Config is typically passed to
/// `TransportManager::init_for_test` to create a fresh
/// `TransportManager` rooted at the isolated config.
pub fn iceoryx_test_config() -> iceoryx2::config::Config {
    iceoryx2::testing::generate_isolated_config()
}

/// Environment variable carrying [`IsolatedRoot::root`] to a child process.
pub const CHILD_IOX2_ROOT_ENV: &str = "CER_TEST_IOX2_ROOT";
/// Environment variable carrying [`IsolatedRoot::prefix`] to a child process.
pub const CHILD_IOX2_PREFIX_ENV: &str = "CER_TEST_IOX2_PREFIX";

/// An iceoryx2 root path and prefix a PARENT and a CHILD process can both name.
///
/// [`iceoryx_test_config`] mints its root and prefix internally, which is right
/// for a single-process test and useless for one that has to hand the same
/// namespace to a child. This carries both halves explicitly and removes the
/// directory on drop, so a failing run leaves nothing behind.
pub struct IsolatedRoot {
    dir: std::path::PathBuf,
    /// The `global.root_path` both processes configure, with its trailing slash.
    pub root: String,
    /// The `global.prefix` both processes configure.
    pub prefix: String,
}

impl IsolatedRoot {
    /// Mint a fresh root under the iceoryx2 test directory, tagged so a leftover
    /// is attributable to the test that made it.
    pub fn mint(tag: &str) -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .expect("system clock before the epoch")
            .as_nanos();
        let dir = std::path::PathBuf::from(format!(
            "/tmp/iceoryx2/{tag}_{}_{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).expect("create the isolated iceoryx2 root");
        let root = format!("{}/", dir.display());
        let prefix = format!("{tag}{}_", std::process::id());
        Self { dir, root, prefix }
    }

    /// The iceoryx2 `Config` for this root, for either process.
    pub fn config(&self) -> iceoryx2::config::Config {
        iceoryx2::config::Config::default().pipe_root(&self.root, &self.prefix)
    }
}

impl Drop for IsolatedRoot {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// Apply a root path and prefix to an iceoryx2 `Config`, as a method so the
/// builder above reads in one expression.
trait ConfigRoot {
    fn pipe_root(self, root: &str, prefix: &str) -> Self;
}

impl ConfigRoot for iceoryx2::config::Config {
    fn pipe_root(mut self, root: &str, prefix: &str) -> Self {
        use iceoryx2::prelude::{FileName, Path as IoxPath, SemanticString};
        self.global
            .set_root_path(&IoxPath::new(root.as_bytes()).expect("iceoryx2 root path"));
        self.global.prefix = FileName::new(prefix.as_bytes()).expect("iceoryx2 prefix");
        self
    }
}

/// Run an `#[ignore]`d test in THIS binary as a child process over `root`, wait
/// for it to print `ready_marker`, then KILL it.
///
/// The child is killed with `SIGKILL`, so nothing it owns is dropped and
/// nothing it registered is deregistered: its iceoryx2 ports stay in the
/// services' dynamic configs until a dead-node sweep reaps them. That is the
/// one condition under iceoryx2 0.10 in which a notify reaches FEWER listeners
/// than the service reports, because the notifier's send to a socket with no
/// live reader is refused and the connection is dropped. (Saturating a live
/// listener no longer does it: 0.10 swallows a full doorbell and a notify into
/// an already-notified state skips the send, so both return success.)
///
/// `settle` is time given to the namespace after the kill before the caller
/// reads anything, because the child's death is asynchronous.
///
/// Panics with the child's captured output if it never reports ready, which is
/// the only way this can fail silently.
pub fn kill_child_holding_ports(
    child_test_name: &str,
    root: &IsolatedRoot,
    ready_marker: &str,
    settle: core::time::Duration,
) {
    use std::io::{BufRead, BufReader};
    let exe = std::env::current_exe().expect("the running test binary");
    let mut child = std::process::Command::new(exe)
        .args([
            "--exact",
            child_test_name,
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_IOX2_ROOT_ENV, &root.root)
        .env(CHILD_IOX2_PREFIX_ENV, &root.prefix)
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap_or_else(|e| panic!("spawn the child test `{child_test_name}`: {e}"));

    let stdout = child.stdout.take().expect("child stdout is piped");
    let mut reader = BufReader::new(stdout);
    let mut seen = String::new();
    let mut ready = false;
    // Bounded: a child that dies before printing closes the pipe, which ends
    // the loop at once rather than hanging.
    for _ in 0..4096 {
        let mut line = String::new();
        match reader.read_line(&mut line) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
        seen.push_str(&line);
        if line.contains(ready_marker) {
            ready = true;
            break;
        }
    }
    if !ready {
        let _ = child.kill();
        let out = child.wait_with_output().ok();
        let stderr = out
            .map(|o| String::from_utf8_lossy(&o.stderr).into_owned())
            .unwrap_or_default();
        panic!(
            "the child test `{child_test_name}` never printed `{ready_marker}`, so the \
             condition under test was never established.\n--- child stdout ---\n{seen}\n\
             --- child stderr ---\n{stderr}"
        );
    }

    // SIGKILL: no unwinding, no Drop, no deregistration.
    child.kill().expect("kill the child test");
    let _ = child.wait();
    std::thread::sleep(settle);
}

/// The config a CHILD spawned by [`kill_child_holding_ports`] must use, read
/// back out of its environment. `None` when the variables are absent, which is
/// how an `#[ignore]`d child body detects that it was run directly rather than
/// by its parent and returns without doing anything.
pub fn child_iceoryx_config() -> Option<iceoryx2::config::Config> {
    let root = std::env::var(CHILD_IOX2_ROOT_ENV).ok()?;
    let prefix = std::env::var(CHILD_IOX2_PREFIX_ENV).ok()?;
    Some(iceoryx2::config::Config::default().pipe_root(&root, &prefix))
}

/// Re-export of
/// [`crate::spill_fault_injection`] for source compatibility with
/// test code that imports via `cerulion_core::testing::...`.
///
/// The module itself moved to the crate root and is now always-on
/// (codegen needs to reference it from crates that don't enable
/// the `test-helpers` feature). See the module's own doc for the
/// soundness argument.
pub use crate::spill_fault_injection;

/// Per-size FLOOR (min over iters): the uncontended floor latency, robust to
/// CI-VM jitter (noise only adds latency, never lowers the floor — see
/// `flat_latency_test`'s `FLATNESS_MAX` rationale). Returns `f64::MAX` for an
/// empty slice so a missing measurement can't masquerade as a 0 floor.
pub fn floor_ns(samples: &[f64]) -> f64 {
    samples.iter().copied().fold(f64::MAX, f64::min)
}

/// Result of [`drop_one_outlier_robust_ratio`]. Carries the `(key, floor)`
/// tuples so callers can print WHICH size was min / dropped / 2nd-highest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobustFlatness {
    /// (key, floor) of the minimum floor (the ratio denominator).
    pub min: (usize, f64),
    /// (key, floor) of the 2nd-HIGHEST floor (the ratio numerator after the
    /// single worst outlier is discarded).
    pub robust_max: (usize, f64),
    /// (key, floor) of the single worst floor, DISCARDED (the dropped outlier).
    pub dropped: (usize, f64),
    /// `robust_max.1 / min.1`, or `f64::INFINITY` if `min.1 <= 0`.
    pub ratio: f64,
}

/// Drop-one-outlier robust flatness ratio: sort the per-`(key, floor)`
/// pairs by floor, DISCARD the single worst (tolerates ONE VM-stalled per-size
/// window at any position), and compare the 2nd-highest floor to the minimum.
/// A real O(n) copy inflates MULTIPLE sizes monotonically, so dropping one
/// still leaves a huge ratio. Panics if fewer than 3 entries (the gate is
/// meaningless with fewer — both call sites sweep >= 5 sizes).
pub fn drop_one_outlier_robust_ratio(floors: &[(usize, f64)]) -> RobustFlatness {
    assert!(
        floors.len() >= 3,
        "drop-one-outlier robust ratio needs >= 3 sizes, got {}",
        floors.len()
    );
    let mut sorted = floors.to_vec();
    sorted.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("floor is finite"));
    let min = sorted[0];
    let dropped = sorted[sorted.len() - 1];
    let robust_max = sorted[sorted.len() - 2];
    let ratio = if min.1 > 0.0 {
        robust_max.1 / min.1
    } else {
        f64::INFINITY
    };
    RobustFlatness {
        min,
        robust_max,
        dropped,
        ratio,
    }
}

// ---------------------------------------------------------------------------
// Window-health retry + uniform-stall discriminator (pure helpers).
//
// The drop-one-outlier ratio tolerates exactly ONE VM-stalled per-size
// window. A VM stall can span FOUR of five sizes (measured on a macOS VM:
// cross_thread_rtt floors [16.9, 43.3, 42.8, 43.1, 43.7]µs — uniform ~43µs,
// size-INDEPENDENT), which defeats drop-one. These helpers add a window-health
// RETRY set + a uniform-stall-vs-monotonic-copy DISCRIMINATOR so a partial
// uniform stall is either healed by a bounded retry or, if unhealed, fails as
// an ATTRIBUTABLE non-probative red naming the stall signature — never as a
// mysterious flatness red, and NEVER masking a real monotonic-in-size copy.
// The gate ceilings + healthy-path assertion are UNCHANGED (the pin is not
// weakened). See `flat_latency_test` / `cross_thread_rtt_test`.
// ---------------------------------------------------------------------------

/// Minimum number of ELEVATED per-size floors that must share a tight band for
/// a gate failure to be attributed to a size-INDEPENDENT uniform VM stall.
/// Three is the smallest count where the drop-one gate can still
/// fail from a stall alone: dropping one elevated floor leaves >= 2, so the
/// drop-one `robust_max` stays elevated and the ratio exceeds the ceiling. A
/// lone (or a pair of) elevated floor(s) is left to the drop-one rule.
pub const UNIFORM_STALL_MIN_SIZES: usize = 3;

/// The tight band factor within which the elevated floors must fall to be
/// classified as a size-INDEPENDENT uniform stall, vs a
/// monotonic-in-size copy. `1.3×` sits well below the SMALLEST adjacent
/// payload-size ratio in the sweep (`4×`: 262144→1048576), so a real O(n) copy
/// — whose adjacent floors differ by the >= 4× payload-size step — can NEVER
/// masquerade as uniform, while the observed macOS-VM stall (floors within
/// ~1.02× of each other) sits far inside the band. (A fixed-overhead, non-O(n)
/// regression inflates ALL sizes including the smallest, so nothing exceeds
/// `ceiling × min` and it is invisible to a ratio gate by design — not this
/// discriminator's concern.)
pub const UNIFORM_STALL_BAND: f64 = 1.3;

/// Window-health RETRY set: the keys (sizes) whose floor exceeds
/// `k × min_floor` — i.e. the windows to re-measure because, taken as the
/// drop-one `robust_max`, they could push the gate ratio to/above the ceiling.
///
/// Callers pass the gate's OWN ceiling as `k` (1.5 for `flat_latency`, 2.0 for
/// `cross_thread_rtt`): this is the tightest correct threshold. Any smaller `k`
/// would retry healthy-passing sizes (wasteful, and re-rolling a clean sample
/// could mask real signal); any larger `k` would skip some would-fail sizes. A
/// healthy, flat sweep (every floor within `k×` of the min) returns an EMPTY
/// set, so zero retry cost is paid on a clean machine. Order matches input order.
///
/// A degenerate `min_floor <= 0` (a measurement bug) returns an empty set — the
/// gate's resulting infinite ratio then fails loudly on its own rather than
/// being papered over by a retry.
pub fn sizes_needing_retry(floors: &[(usize, f64)], k: f64) -> Vec<usize> {
    let min_floor = floors.iter().map(|&(_, f)| f).fold(f64::MAX, f64::min);
    if min_floor <= 0.0 {
        return Vec::new();
    }
    let threshold = k * min_floor;
    floors
        .iter()
        .filter(|&&(_, f)| f > threshold)
        .map(|&(key, _)| key)
        .collect()
}

/// Uniform-stall DISCRIMINATOR: does a set of final
/// per-size floors that has ALREADY failed the drop-one gate carry the
/// size-INDEPENDENT signature of a VM stall (`true`) rather than a real
/// monotonic-in-size copy (`false`)?
///
/// Signature = there are `>= UNIFORM_STALL_MIN_SIZES` (3) floors that both
/// (a) exceed `ceiling × min_floor` (elevated) AND (b) fall within a tight
/// `UNIFORM_STALL_BAND` (1.3×) of each other. A real O(n) copy makes the floor
/// scale with payload size, so across the >= 4× adjacent size steps its
/// elevated floors span far more than 1.3× — it can never satisfy (b), and this
/// returns `false` so the gate fails NORMALLY (the pin is not weakened). The
/// observed macOS stall (four floors all ~43µs, min 16.9µs) satisfies both and
/// returns `true`.
///
/// NOTE the whole-sweep min is the denominator (consistent with the drop-one
/// gate). A degenerate `min_floor <= 0` returns `false` (let the gate fail
/// loudly). When `>= 3` floors are elevated, the drop-one `robust_max` is
/// necessarily elevated too, so a `true` verdict implies the gate is violated —
/// the caller still checks the ratio first and only consults this on a failure.
pub fn is_uniform_stall_signature(floors: &[(usize, f64)], ceiling: f64) -> bool {
    let min_floor = floors.iter().map(|&(_, f)| f).fold(f64::MAX, f64::min);
    if min_floor <= 0.0 {
        return false;
    }
    let threshold = ceiling * min_floor;
    let elevated: Vec<f64> = floors
        .iter()
        .map(|&(_, f)| f)
        .filter(|&f| f > threshold)
        .collect();
    if elevated.len() < UNIFORM_STALL_MIN_SIZES {
        return false;
    }
    let emin = elevated.iter().copied().fold(f64::MAX, f64::min);
    let emax = elevated.iter().copied().fold(f64::MIN, f64::max);
    // elevated floors are all > threshold > 0, so emin > 0 here.
    emax / emin <= UNIFORM_STALL_BAND
}

/// The FULL max/min flatness of a sweep's per-size floors — the decision input
/// to [`classify_flatness`]. Reports which size carried the min / max floor and
/// their `max/min` ratio (`f64::INFINITY` if `min <= 0`). Deliberately NOT the
/// drop-one-outlier ratio — see [`classify_flatness`] for why drop-one is unsafe
/// on a wide-geometry sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FullFlatness {
    /// (key, floor) of the minimum floor (the ratio denominator).
    pub min: (usize, f64),
    /// (key, floor) of the maximum floor (the ratio numerator — NO outlier
    /// dropped).
    pub max: (usize, f64),
    /// `max.1 / min.1`, or `f64::INFINITY` if `min.1 <= 0`.
    pub ratio: f64,
}

/// The verdict of the FULL-ratio flatness gate, composing the whole-
/// sweep [`FullFlatness`] with the [`is_uniform_stall_signature`] discriminator
/// into the SINGLE decision a latency-sweep gate acts on. Made a pure, oracle-
/// tested value so the composition — ratio FIRST, then the uniform-stall
/// discriminator only on a failure — is unit-verifiable and cannot be mis-wired
/// at a call site. Carries the [`FullFlatness`] so the caller can print which
/// size was min / max.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlatnessVerdict {
    /// The FULL max/min ratio is UNDER the ceiling — the path is flat. PASS.
    /// (Single VM-stalled windows are absorbed BEFORE the gate by the
    /// window-health retry, not by dropping a floor here.)
    Pass(FullFlatness),
    /// The ratio EXCEEDS the ceiling AND the elevated floors carry the
    /// size-INDEPENDENT uniform-stall signature ([`is_uniform_stall_signature`]):
    /// a shared-CI-VM stall spanning `>= UNIFORM_STALL_MIN_SIZES` measurement
    /// windows. An ATTRIBUTABLE, NON-PROBATIVE infra red (re-run on a quiescent
    /// host), NOT a zero-copy regression.
    UniformStall(FullFlatness),
    /// The ratio EXCEEDS the ceiling and the shape is NOT the uniform-stall
    /// signature. The gate FAILS NORMALLY — a genuine O(n)-copy regression, OR
    /// (the conservative residual) a persistent single/two-window stall that
    /// survived every retry and cannot be distinguished from a real single-size
    /// regression. Fail loud; a human re-runs on a quiescent host to disambiguate.
    RealCopy(FullFlatness),
}

/// Classify a payload sweep's per-size floors against `ceiling`.
///
/// Gates on the FULL `max/min` ratio (NO drop-one outlier), then consults the
/// [`is_uniform_stall_signature`] discriminator ONLY on a would-fail sweep to
/// route it to [`FlatnessVerdict::UniformStall`] (attributable infra red) vs
/// [`FlatnessVerdict::RealCopy`] (fail the gate). `floors` must be non-empty.
///
/// # Why drop-one-outlier is deliberately absent here
///
/// [`drop_one_outlier_robust_ratio`] discards the single worst floor to
/// tolerate one VM-stalled window, and is correct for the SIBLING gates
/// (`flat_latency_test` / `cross_thread_rtt_test`) — their sweeps put the
/// 2nd-largest size at ~1 MiB under a ~1 µs base RTT, so a real O(n) copy still
/// inflates that 2nd-largest floor thousands-of-× and the copy signal SURVIVES
/// the drop. It is UNSAFE for a gate whose sweep's 2nd-largest size is far
/// smaller than its largest AND whose base RTT is large. On the CLI-e2e sweep
/// `[64, 4096, 65536, 1048576]` the 2nd-largest (64 KiB) is 16× smaller than the
/// largest (1 MiB): a linear copy that adds e.g. +160 µs at 1 MiB adds only
/// ~160/16 = ~10 µs at 64 KiB — BELOW that chain's ~11-40 µs base RTT. Drop-one
/// discards the 1 MiB floor (where the copy actually shows), `max` collapses to
/// the barely-inflated 64 KiB floor, and the ratio is ~1× ⇒ a real copy PASSES.
/// The full `max/min` keeps the 1 MiB floor and catches
/// it. So this gate gates on the full ratio; the SINGLE-window-stall defense is
/// instead the window-health RETRY ([`run_window_health_retries`]): a transient
/// stall heals in a fresh subprocess, while a real copy re-measures just as slow
/// (it never heals). A stall that survives all retries fails as `RealCopy` — the
/// conservative direction (indistinguishable from a real single-size regression).
pub fn classify_flatness(floors: &[(usize, f64)], ceiling: f64) -> FlatnessVerdict {
    assert!(!floors.is_empty(), "classify_flatness needs >= 1 size");
    let min = floors
        .iter()
        .copied()
        .min_by(|a, b| a.1.partial_cmp(&b.1).expect("floor is finite"))
        .expect("non-empty");
    let max = floors
        .iter()
        .copied()
        .max_by(|a, b| a.1.partial_cmp(&b.1).expect("floor is finite"))
        .expect("non-empty");
    let ratio = if min.1 > 0.0 {
        max.1 / min.1
    } else {
        f64::INFINITY
    };
    let ff = FullFlatness { min, max, ratio };
    if ratio < ceiling {
        FlatnessVerdict::Pass(ff)
    } else if is_uniform_stall_signature(floors, ceiling) {
        FlatnessVerdict::UniformStall(ff)
    } else {
        FlatnessVerdict::RealCopy(ff)
    }
}

/// One window-health retry attempt record (stall hardening (b)) — the caller prints it.
#[derive(Debug, Clone, PartialEq)]
pub struct RetryRecord {
    /// The payload size (key) whose window was re-measured.
    pub size: usize,
    /// 1-based recompute ROUND this attempt happened in (the would-fail set is
    /// re-derived from the CURRENT floors at the start of each round).
    pub round: usize,
    /// 1-based attempt number for THIS size, cumulative across rounds
    /// (<= `max_retries` — the per-size budget).
    pub attempt: usize,
    /// The size's floor BEFORE this attempt.
    pub old_floor: f64,
    /// The floor this fresh window MEASURED (may be higher or lower than old).
    pub measured_floor: f64,
    /// `old_floor.min(measured_floor)` — the floor KEPT after this attempt.
    pub kept_floor: f64,
}

/// Stall hardening (b): window-health RETRY orchestration (PURE, no I/O; the only
/// non-pure dependency, the transport measurement, is injected as `remeasure`).
///
/// Runs in ROUNDS: each round re-derives the would-fail set (floor >
/// `ceiling * min_floor`, via [`sizes_needing_retry`]) from the CURRENT floors,
/// re-measures each member ONCE (keeping the MIN floor), and repeats until the
/// set is empty or every remaining member has exhausted its PER-SIZE budget of
/// `max_retries` total attempts across all rounds.
///
/// **Why per-round recompute:** a retry keeps
/// `min(old, fresh)` and can lower an elevated size's floor BELOW the global
/// min (the fresh window found a truer floor). The gate-ratio DENOMINATOR then
/// shrinks, and sizes that were healthy relative to the OLD min become
/// would-fail relative to the NEW one. A retry set frozen at entry (the
/// alternative) never re-measures those, so a retry could only make the run
/// REDDER: e.g. initial floors [20, 20.5, 20.2, 31, 20.3] PASS drop-one at
/// 20.3/20 = 1.025×, but the 31 retrying down to 13 leaves 20.3/13 = 1.56×,
/// over the 1.5 ceiling — a would-pass noisy run flipped to the non-probative
/// red.
/// Recomputing per round gives every would-fail size equal re-measure
/// opportunity against the floors it will actually be gated on: if their fresh
/// windows also find the truer 13-class floors the sweep heals and PASSES
/// legitimately; if they genuinely stay high the gate fails on equal-opportunity
/// floors (and the uniform-stall discriminator still routes the verdict).
///
/// Termination is guaranteed by the budgets: every round re-measures >= 1
/// budget-remaining size, and each size re-measures at most `max_retries`
/// times, so total remeasures <= `max_retries * stats.len()`. Floors never
/// increase (kept = min), so the min never rises and a set member can only
/// leave the would-fail set via its own retry (in a later round's recompute).
/// A healthy sweep's FIRST recompute returns an empty set — zero remeasures,
/// zero cost.
///
/// `stats` is mutated in place — a size's full `(size, floor, p50, p99, max)`
/// tuple is replaced when a fresh window beats its floor (so the reported
/// percentiles track the window that produced the kept floor). Returns the
/// per-attempt log (with round numbers) for the caller to print.
///
/// `remeasure(size) -> (floor, p50, p99, max)` re-measures ONE size in a fresh
/// temporal window; a transient VM stall almost always clears within it. With a
/// canned closure this whole orchestration is unit-testable with no transport.
pub fn run_window_health_retries<F>(
    stats: &mut [(usize, f64, f64, f64, f64)],
    ceiling: f64,
    max_retries: usize,
    mut remeasure: F,
) -> Vec<RetryRecord>
where
    F: FnMut(usize) -> (f64, f64, f64, f64),
{
    // Per-size attempt budget: a size never re-measures more than `max_retries`
    // times TOTAL, no matter how the reference min moves between rounds.
    let mut attempts: Vec<(usize, usize)> = stats.iter().map(|&(s, ..)| (s, 0)).collect();
    let mut log = Vec::new();
    let mut round = 0usize;
    loop {
        round += 1;
        // Re-derive the would-fail set from the CURRENT floors, keeping only
        // sizes with remaining budget.
        let floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        let retry_set: Vec<usize> = sizes_needing_retry(&floors, ceiling)
            .into_iter()
            .filter(|&size| {
                attempts
                    .iter()
                    .find(|e| e.0 == size)
                    .map(|e| e.1 < max_retries)
                    .unwrap_or(false)
            })
            .collect();
        if retry_set.is_empty() {
            // Every elevated size healed OR exhausted its budget — done.
            return log;
        }
        for size in retry_set {
            // Live re-check (cheap invariant guard: floors never increase, so a
            // set member cannot become healthy mid-round except via its OWN
            // retry, which lands in a later round's recompute).
            let min_floor = stats.iter().map(|&(_, f, ..)| f).fold(f64::MAX, f64::min);
            let old = stats
                .iter()
                .find(|&&(s, ..)| s == size)
                .map(|&(_, f, ..)| f)
                .expect("retry size present in stats");
            if old <= ceiling * min_floor {
                continue;
            }
            let (mfloor, mp50, mp99, mmax) = remeasure(size);
            let entry = attempts
                .iter_mut()
                .find(|e| e.0 == size)
                .expect("retry size present in budget");
            entry.1 += 1;
            let attempt = entry.1;
            let kept = old.min(mfloor);
            log.push(RetryRecord {
                size,
                round,
                attempt,
                old_floor: old,
                measured_floor: mfloor,
                kept_floor: kept,
            });
            if mfloor < old {
                for e in stats.iter_mut() {
                    if e.0 == size {
                        *e = (size, kept, mp50, mp99, mmax);
                    }
                }
            }
        }
    }
}

/// A self-contained, **isolated** iceoryx2 transport for tests
/// that build publishers/subscribers directly (the `NodeContext::for_tests`
/// pattern), replacing the deleted `InProcessPublisher`.
///
/// Each `TestTransport` owns its own iceoryx2 `TransportManager` rooted at a
/// unique per-instance SHM prefix (via [`iceoryx_test_config`]), so two
/// `TestTransport`s never collide even on identical topic names — tests stay
/// parallel-safe with no manual topic-uniquing. The manager is held for the
/// `TestTransport`'s lifetime, so the returned publishers/subscribers stay
/// valid as long as the `TestTransport` is in scope.
///
/// Mirrors the old `InProcessPublisher::new(topic, clock, history, max)` +
/// `add_subscriber_channel(buf)` ergonomics so a migrated test reads almost
/// identically:
///
/// ```no_run
/// # #[cfg(feature = "test-helpers")] {
/// use cerulion_core::testing::TestTransport;
/// use cerulion_core::wire::MaxSliceLen;
/// let tt = TestTransport::with_buffer_size(8);
/// let publisher = tt.publisher("topic", MaxSliceLen::const_new(256), 0);
/// let subscriber = tt.subscriber("topic");
/// # }
/// ```
#[cfg(any(test, feature = "test-helpers"))]
pub struct TestTransport {
    mgr: std::sync::Arc<crate::transport::TransportManager>,
}

#[cfg(any(test, feature = "test-helpers"))]
impl TestTransport {
    /// Isolated transport with the default subscriber buffer size (16).
    pub fn new() -> Self {
        Self::with_buffer_size(16)
    }

    /// Isolated transport with an explicit subscriber buffer size (the
    /// iceoryx2 analog of the old per-channel `add_subscriber_channel(buf)`
    /// capacity — applied to every subscriber created from this manager).
    pub fn with_buffer_size(subscriber_buffer_size: usize) -> Self {
        let ix_config = iceoryx_test_config();
        let config = crate::transport::TransportConfig {
            node_name: "cerulion_test_transport".into(),
            clock: std::sync::Arc::new(crate::clock::VirtualClock::new()),
            subscriber_buffer_size,
            network: None,
        };
        let mgr = crate::transport::TransportManager::init_for_test(config, ix_config)
            .expect("TestTransport: init_for_test failed");
        Self { mgr }
    }

    /// Create an iceoryx2 publisher on `topic` (replaces
    /// `InProcessPublisher::new`). `history` mirrors the old constructor's
    /// `history_size` argument.
    pub fn publisher(
        &self,
        topic: &str,
        max_slice_len: crate::wire::MaxSliceLen,
        history: usize,
    ) -> crate::transport::publisher::CerulionPublisher {
        self.mgr
            .create_publisher(topic, max_slice_len, history)
            .expect("TestTransport: create_publisher failed")
    }

    /// Create an iceoryx2 subscriber on `topic` (replaces
    /// `InProcessPublisher::add_subscriber_channel`). Subscribers created on
    /// the same `topic` from the same `TestTransport` receive what the
    /// matching publisher sends.
    pub fn subscriber(&self, topic: &str) -> crate::transport::subscriber::CerulionSubscriber {
        self.mgr
            .create_subscriber(topic)
            .expect("TestTransport: create_subscriber failed")
    }

    /// This manager's default per-topic config (the
    /// non-graph opener values) — the seed for
    /// [`crate::transport::TopicServiceConfig::for_topology`] in tests
    /// (`TopicServiceConfig` is `#[non_exhaustive]`, so tests cannot
    /// construct it literally).
    /// The manager behind this transport, for release-visible STATE oracles
    /// (a port count) beside a profile-gated log oracle.
    pub fn manager(&self) -> &crate::transport::TransportManager {
        &self.mgr
    }

    pub fn default_topic_config(&self) -> crate::transport::TopicServiceConfig {
        self.mgr.default_topic_config()
    }

    /// A publisher with an explicit per-topic service config — the
    /// publisher twin of [`Self::subscriber_with_buffers`], so a test can arm
    /// port-level knobs (`publisher_max_loaned_samples`) on the config it
    /// hands the create site. Returns the `Result` so tests can assert the
    /// loud choke-point rejections (e.g. a zero loan budget).
    pub fn publisher_with_topic_config(
        &self,
        topic: &str,
        max_slice_len: crate::wire::MaxSliceLen,
        history: usize,
        topic_config: crate::transport::TopicServiceConfig,
    ) -> crate::error::TransportResult<crate::transport::publisher::CerulionPublisher> {
        self.mgr
            .create_publisher_with_topic_config(topic, max_slice_len, history, topic_config)
    }

    /// A subscriber with an explicit per-topic service
    /// config plus its own queue depth — the graph runtime's path. Returns
    /// the `Result` so tests can assert the defensive ceiling rejection.
    pub fn subscriber_with_buffers(
        &self,
        topic: &str,
        topic_config: crate::transport::TopicServiceConfig,
        buffer_size: usize,
    ) -> crate::error::TransportResult<crate::transport::subscriber::CerulionSubscriber> {
        self.mgr
            .create_subscriber_with_buffers(topic, topic_config, buffer_size)
    }

    /// Attach a listener-less data-only capture tap
    /// ([`crate::transport::subscriber::DataOnlySubscriber`]) on `topic` — the
    /// record/replay tap constructor. Open-only (the topic's data service must
    /// already exist), so a test that wants a specific borrow/buffer budget
    /// provisions the service first (e.g. via [`Self::subscriber_with_buffers`])
    /// and then attaches this tap.
    pub fn data_only_subscriber(
        &self,
        topic: &str,
    ) -> crate::error::TransportResult<crate::transport::subscriber::DataOnlySubscriber> {
        self.mgr.create_data_only_subscriber(topic)
    }
}

#[cfg(any(test, feature = "test-helpers"))]
impl Default for TestTransport {
    fn default() -> Self {
        Self::new()
    }
}

// Kept LAST in the file: clippy `items_after_test_module` requires a
// `#[cfg(test)]` module to be the final item (no non-test items may follow it).
/// Publish a run record on the iceoryx2 namespace described by
/// `ix_config_json`, returning the live [`RunHandle`](crate::transport::run_registry::RunHandle).
///
/// A JSON-shaped adapter, following the same convention as
/// [`TransportManager::init_with_ix_config_json`](crate::TransportManager::init_with_ix_config_json):
/// `cerulion_bagd` deliberately carries NO direct `iceoryx2` dependency (the
/// exact-pin lockstep test guards that family), so its tests cannot name an
/// `iceoryx2::config::Config` to hand `RunHandle::publish_on_config` — but they
/// DO need a live run on the recorder's own isolated SHM root in order to prove
/// the recorder never taps the run registry.
///
/// Test scaffolding by design: production publishes on the process-global
/// namespace via [`RunHandle::publish`](crate::transport::run_registry::RunHandle::publish),
/// so this adapter is deliberately here rather than as a production entry point
/// with no production caller.
///
/// # Errors
///
/// [`TransportError`](crate::TransportError) if the JSON is not a valid
/// iceoryx2 config, or if the node / control service / publisher cannot be
/// created.
pub fn publish_run_on_ix_config_json(
    ix_config_json: &str,
    record: crate::transport::run_registry::RunRecord,
) -> crate::TransportResult<crate::transport::run_registry::RunHandle> {
    let config: iceoryx2::config::Config =
        serde_json::from_str(ix_config_json).map_err(|e| crate::TransportError::Internal {
            reason: format!("run registry: invalid iceoryx2 config JSON: {e}"),
        })?;
    crate::transport::run_registry::RunHandle::publish_on_config(&config, record)
}

// ───────────────────────────────────────────────────────────────────────────
// Prebuilt fixture-cdylib resolution
// ───────────────────────────────────────────────────────────────────────────

/// The `<target>/<profile>` directory the RUNNING test binary was built into.
///
/// Derived from [`std::env::current_exe`] — NOT from a hardcoded
/// `<repo_root>/target`, and NOT by reading `CARGO_TARGET_DIR`.
///
/// # Why not `<repo_root>/target`
///
/// A fixture locator that spells its own
/// `env!("CARGO_MANIFEST_DIR")/../target/debug` breaks under an isolated build tree
/// (`CARGO_TARGET_DIR=/some/cache/<tag>`, a common worktree setup):
/// `cargo build -p test_node_*_cdylib` writes the fixture into the ISOLATED
/// tree while the test looks in `<repo>/target/debug` — so a correctly-built
/// fixture reports as missing, with a build instruction the developer has
/// already followed.
///
/// # Why not `CARGO_TARGET_DIR`
///
/// Reading the env var is only a partial fix: cargo also honors
/// `build.target-dir` from `.cargo/config.toml`, and does not necessarily
/// export `CARGO_TARGET_DIR` into the test process. The running binary's own
/// location is the ground truth for where cargo put artifacts for THIS
/// invocation, whatever set it. It also gets `--target <triple>` right for
/// free (`<target>/<triple>/<profile>`), which no hardcoded path did.
///
/// # Layout
///
/// Cargo places integration-test binaries at `<target>/<profile>/deps/<name>-<hash>`,
/// so the profile dir is the grandparent. A binary exec'd straight out of
/// `<target>/<profile>/` is also tolerated (its parent IS the profile dir).
///
/// The returned path is used exactly as `current_exe()` reported it, walked
/// with [`Path::parent`] only: on macOS `current_exe()` is NOT canonicalized,
/// so any symlink/prefix assumption about the absolute path would be unsound.
///
/// # Panics
///
/// If `current_exe()` fails or the binary has no parent directory.
pub fn test_target_profile_dir() -> std::path::PathBuf {
    let exe = std::env::current_exe().expect(
        "current_exe() failed — cannot locate the cargo target directory for this test run",
    );
    let dir = exe.parent().unwrap_or_else(|| {
        panic!(
            "test binary {} has no parent directory — cannot locate the cargo target directory",
            exe.display()
        )
    });
    if dir.file_name() == Some(std::ffi::OsStr::new("deps")) {
        dir.parent()
            .unwrap_or_else(|| {
                panic!(
                    "test binary {} sits in a `deps` dir with no parent — cannot locate \
                     the cargo target directory",
                    exe.display()
                )
            })
            .to_path_buf()
    } else {
        dir.to_path_buf()
    }
}

/// The platform file name a cdylib crate builds to (`lib<name>.dylib` on
/// macOS, `lib<name>.so` on Linux/other Unix, `<name>.dll` on Windows).
pub fn cdylib_file_name(crate_name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{crate_name}.dylib")
    } else if cfg!(target_os = "windows") {
        format!("{crate_name}.dll")
    } else {
        format!("lib{crate_name}.so")
    }
}

/// Locate a prebuilt fixture cdylib, searching the RUNNING test binary's own
/// profile directory first and its sibling profile second.
///
/// The sibling fallback preserves the tolerance the `cerulion_cli` locators
/// already had (`for profile in ["debug", "release"]`) and is a strict
/// improvement on the `cerulion_core` ones, which hardcoded `debug` and so
/// silently loaded a DEBUG fixture into a `cargo test --release` run. Callers
/// that must not cross profiles use [`find_fixture_cdylib_same_profile`].
///
/// Build the fixture with the profile you run the test under — it lands in the
/// same target directory the test binary did, which is exactly why deriving
/// from [`test_target_profile_dir`] works under any `CARGO_TARGET_DIR`.
///
/// # Panics
///
/// If the fixture is not present under either profile, naming both searched
/// paths and the `cargo build -p <crate_name>` that produces it.
pub fn find_fixture_cdylib(crate_name: &str) -> std::path::PathBuf {
    let profile_dir = test_target_profile_dir();
    let file = cdylib_file_name(crate_name);

    let primary = profile_dir.join(&file);
    if primary.exists() {
        return primary;
    }

    // `<target>/debug` ⇄ `<target>/release`. A custom profile has no sibling.
    let sibling = match profile_dir.file_name().and_then(|s| s.to_str()) {
        Some("debug") => profile_dir.parent().map(|t| t.join("release")),
        Some("release") => profile_dir.parent().map(|t| t.join("debug")),
        _ => None,
    };
    if let Some(dir) = &sibling {
        let p = dir.join(&file);
        if p.exists() {
            return p;
        }
    }

    let searched = match &sibling {
        Some(dir) => format!("{} or {}", primary.display(), dir.join(&file).display()),
        None => primary.display().to_string(),
    };
    panic!(
        "fixture cdylib `{file}` not found — run `cargo build -p {crate_name}` first \
         (searched {searched}; these paths follow THIS test binary's own target \
         directory, so build the fixture under the same CARGO_TARGET_DIR / profile \
         you are running the test under)"
    );
}

/// Locate a prebuilt fixture cdylib in the RUNNING test binary's profile
/// directory ONLY — no sibling-profile fallback.
///
/// For oracles that compare the HOST's static log level against what the
/// CHILD emits (`cdylib_tracing_stopgap_test`, its only caller today):
/// [`debug_level_compiled_in`] is predictive of the child only when both
/// resolve the same `STATIC_MAX_LEVEL`, i.e. the same profile, so a
/// sibling-profile fallback must fail loudly instead of being used.
///
/// What this does NOT cover: staleness. A fixture built under the right
/// profile from source edited after the last build is accepted —
/// `cargo test` cannot rebuild a fixture crate, so rebuild it before trusting
/// a test that loads it.
///
/// # Panics
///
/// If the fixture is not present under the running profile.
pub fn find_fixture_cdylib_same_profile(crate_name: &str) -> std::path::PathBuf {
    let profile_dir = test_target_profile_dir();
    let file = cdylib_file_name(crate_name);
    let path = profile_dir.join(&file);
    assert!(
        path.exists(),
        "fixture cdylib `{file}` not found at {} — run `cargo build -p {crate_name}` \
         under the SAME profile as this test (this gate does not accept a \
         sibling-profile fixture). The path follows THIS test binary's own target \
         directory, so build under the same CARGO_TARGET_DIR too.",
        path.display()
    );
    path
}

/// The shared GAP-FRAME fixture: ONE hand-laid
/// `sensor_msgs/Image` wire frame whose top-level offset-table entries use the
/// full legality the `PayloadAudit::Frame` audit deliberately grants
/// (`crates/cerulion_core/src/codegen/frame_walker.rs`): fields placed OUT OF
/// declaration order, the big variable field page-aligned deep in the payload
/// tail, dead gap bytes between fields and trailing slack — all inside
/// `total_size`. The adopt-take ingress publisher produces exactly this shape, so the wire-legality
/// oracle suite pins it in four independent places (walker decode, generated
/// accessors, bagd byte-fidelity, gateway forward, replay diff) from this ONE
/// stated layout.
///
/// Layout (PAYLOAD-relative; the wire frame prepends the 32-byte header):
///
/// ```text
/// [0,   16)  fixed section: height u32 | width u32 | is_bigendian u8 (pad 3) | step u32
/// [16,  40)  offset table, declaration order: header | encoding | data
/// [40,  64)  DEAD GAP (0xEE)                       <- 24 bytes above the floor
/// [64,  83)  `header` sub-frame (19 B)             <- declared FIRST, placed first
/// [83, 4096) DEAD GAP (0xEE)                       <- big gap up to the page
/// [4096,8192) `data` (4096 B, page-aligned)        <- declared THIRD, placed second
/// [8192,8205) DEAD GAP (0xEE)
/// [8205,8209) `encoding` ("rgb8")                  <- declared SECOND, placed LAST
/// [8209,8217) TRAILING SLACK (0xEE)
/// ```
///
/// `encoding` (declaration index 1) placed AFTER `data` (index 2) is the
/// out-of-declaration-order half of the pin; `data` at 4096 is the
/// page-aligned half. The `schema_hash` is a parameter because this module
/// cannot depend on `native_ros2_messages` (a dev-dependency) — tests pass
/// `Image::SCHEMA_HASH`.
#[cfg(any(test, feature = "test-helpers"))]
pub mod gap_frame {
    use crate::shm_runtime::write_offset_entry;
    use crate::wire::WireHeader;

    /// `sensor_msgs/Image`'s `#[repr(C)]` fixed-section size: height(4) +
    /// width(4) + is_bigendian(1, pad 3) + step(4) = 16. Drift-guarded in the
    /// oracle tests against the generated `Image::WIRE_FIXED_SIZE`.
    pub const IMAGE_WIRE_FIXED_SIZE: usize = 16;
    /// Image's variable fields, declaration order: `header`, `encoding`,
    /// `data`.
    pub const IMAGE_VARIABLE_FIELD_COUNT: usize = 3;
    /// The first payload byte a top-level entry may legally point at:
    /// fixed section + offset table.
    pub const DATA_FLOOR: usize = IMAGE_WIRE_FIXED_SIZE + 8 * IMAGE_VARIABLE_FIELD_COUNT;

    /// Hand fixed-field values (asymmetric, so a swapped offset shows up).
    pub const HEIGHT: u32 = 480;
    pub const WIDTH: u32 = 640;
    pub const IS_BIGENDIAN: u8 = 1;
    pub const STEP: u32 = 1920;

    /// The `header` sub-frame's hand values + placement.
    pub const HEADER_SEC: i32 = 7;
    pub const HEADER_NANOSEC: u32 = 9;
    pub const HEADER_FRAME_ID: &str = "map";
    pub const HEADER_OFF: usize = 64;
    /// `std_msgs/Header` sub-frame: fixed `Time`(8) | 1 entry (8) | frame_id.
    pub const HEADER_LEN: usize = 16 + HEADER_FRAME_ID.len();

    /// The big field, PAGE-ALIGNED in the payload tail (the adopt-take shape).
    pub const DATA_OFF: usize = 4096;
    pub const DATA_LEN: usize = 4096;

    /// `encoding`, placed LAST although declared SECOND.
    pub const ENCODING: &str = "rgb8";
    pub const ENCODING_OFF: usize = DATA_OFF + DATA_LEN + 13;

    /// Trailing slack after the last placed field, inside `total_size`.
    pub const TRAILING_SLACK: usize = 8;
    /// Whole payload length (fixed + table + gaps + fields + slack).
    pub const PAYLOAD_LEN: usize = ENCODING_OFF + ENCODING.len() + TRAILING_SLACK;
    /// The wire `total_size` every fixture frame carries.
    pub const TOTAL_SIZE: usize = WireHeader::SIZE + PAYLOAD_LEN;

    /// Every dead byte (gaps + slack) carries this fill, so byte-fidelity
    /// assertions prove the gaps travel VERBATIM, not as re-zeroed slack.
    pub const GAP_FILL: u8 = 0xEE;

    // The layout is self-consistent by construction, checked at compile time:
    // every placement is at/above the floor and no two placements overlap.
    const _: () = assert!(HEADER_OFF >= DATA_FLOOR);
    const _: () = assert!(HEADER_OFF + HEADER_LEN <= DATA_OFF);
    const _: () = assert!(DATA_OFF + DATA_LEN <= ENCODING_OFF);
    const _: () = assert!(
        DATA_OFF.is_multiple_of(4096),
        "the big field placement is page-aligned"
    );

    /// The `data` payload oracle: a 251-period byte pattern (not the gap
    /// fill), hand-recomputable by every consumer.
    pub fn data_bytes() -> Vec<u8> {
        (0..DATA_LEN).map(|i| (i % 251) as u8).collect()
    }

    /// The hand-built `std_msgs/Header` sub-frame (the element-layout
    /// convention: fixed `builtin_interfaces/Time` inline, then the sub-frame's
    /// own offset table, then `frame_id`).
    pub fn header_subframe() -> Vec<u8> {
        let mut sub = vec![0u8; 16];
        sub[0..4].copy_from_slice(&HEADER_SEC.to_le_bytes());
        sub[4..8].copy_from_slice(&HEADER_NANOSEC.to_le_bytes());
        write_offset_entry(&mut sub, 8, 0, 16, HEADER_FRAME_ID.len() as u32);
        sub.extend_from_slice(HEADER_FRAME_ID.as_bytes());
        assert_eq!(sub.len(), HEADER_LEN);
        sub
    }

    /// Build the full gap frame (32-byte wire header + the payload above).
    /// `sequence`/`timestamp_ns` are the caller's hand values so multi-frame
    /// suites (bagd, gateway, replay) mint distinct, deterministic frames.
    pub fn image_gap_frame(schema_hash: u64, sequence: u32, timestamp_ns: u64) -> Vec<u8> {
        let mut frame = vec![GAP_FILL; TOTAL_SIZE];

        // Wire header, stamped the way the production writer stamps a
        // variable schema (offset_table_offset is FRAME-relative).
        let mut header = WireHeader::new(schema_hash, sequence, timestamp_ns);
        header.total_size = TOTAL_SIZE as u32;
        header.offset_table_offset = (WireHeader::SIZE + IMAGE_WIRE_FIXED_SIZE) as u32;
        header.offset_table_count = IMAGE_VARIABLE_FIELD_COUNT as u32;
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);

        let payload = &mut frame[WireHeader::SIZE..];
        // Fixed section (repr(C) offsets; the pad byte range [9,12) keeps the
        // gap fill — fixed-section padding is not observed by any reader).
        payload[0..4].copy_from_slice(&HEIGHT.to_le_bytes());
        payload[4..8].copy_from_slice(&WIDTH.to_le_bytes());
        payload[8] = IS_BIGENDIAN;
        payload[12..16].copy_from_slice(&STEP.to_le_bytes());

        // Offset table, DECLARATION order — while the placements below are
        // deliberately NOT in that order.
        write_offset_entry(
            payload,
            IMAGE_WIRE_FIXED_SIZE,
            0,
            HEADER_OFF as u32,
            HEADER_LEN as u32,
        );
        write_offset_entry(
            payload,
            IMAGE_WIRE_FIXED_SIZE,
            1,
            ENCODING_OFF as u32,
            ENCODING.len() as u32,
        );
        write_offset_entry(
            payload,
            IMAGE_WIRE_FIXED_SIZE,
            2,
            DATA_OFF as u32,
            DATA_LEN as u32,
        );

        payload[HEADER_OFF..HEADER_OFF + HEADER_LEN].copy_from_slice(&header_subframe());
        payload[DATA_OFF..DATA_OFF + DATA_LEN].copy_from_slice(&data_bytes());
        payload[ENCODING_OFF..ENCODING_OFF + ENCODING.len()].copy_from_slice(ENCODING.as_bytes());

        frame
    }
}

/// `true` iff this build compiled `tracing::debug!` in.
///
/// The exact static gate `debug!` applies — `level_enabled!` is
/// `Level::DEBUG <= STATIC_MAX_LEVEL` — so this cannot disagree with the
/// macro under any combination of `max_level_*` / `release_max_level_*`
/// features. `cerulion_core` enables `tracing/release_max_level_info`, and
/// Cargo unifies that feature across every binary linking the crate, so in a
/// release build `STATIC_MAX_LEVEL` is `INFO` and this reads `false`.
///
/// Use it where a test's DEBUG-line expectation is PRESENCE rather than a
/// count (skip a `("DEBUG", marker)` entry from a line-kind sweep, say). For
/// a count, use [`debug_lines_expected`].
pub fn debug_level_compiled_in() -> bool {
    tracing::Level::DEBUG <= tracing::level_filters::STATIC_MAX_LEVEL
}

/// `true` iff this build compiled `tracing::trace!` in — the same static gate
/// as [`debug_level_compiled_in`], one level lower. `release_max_level_info`
/// compiles `trace!` out exactly as it does `debug!`, so a test asserting a
/// TRACE-only line is PRESENT (`logs_contain("output not written this tick")`)
/// is the same class and takes this gate.
pub fn trace_level_compiled_in() -> bool {
    tracing::Level::TRACE <= tracing::level_filters::STATIC_MAX_LEVEL
}

/// How many DEBUG-level lines a test may demand: `n` where `debug!` is
/// compiled in, `0` where it is not.
///
/// # The class this closes, and the gate that enforces it
///
/// A DEBUG-line count reads 0 in a release build no matter what the code
/// under test did (see [`debug_level_compiled_in`]), so a test that demands a
/// nonzero one fails there deterministically — and it fails in EXACTLY one
/// job: `Latency Threshold (Linux)` is the only CI job running
/// `cargo test -p cerulion_core --lib --release`, and it runs on push to main
/// (plus a manual `workflow_dispatch`) only. Every PR shard is a debug
/// profile, so such a test is green on the PR and main goes
/// red at the merge.
///
/// The rule is therefore structural: every DEBUG-count assertion in
/// `cerulion_core` and `rmw_cerulion` (src AND tests) must route its
/// expectation through this helper (or a presence check through
/// [`debug_level_compiled_in`]), enforced by the comment-stripped source walk
/// in `crates/cerulion_core/tests/debug_count_discipline_test.rs`, which names the
/// offending `file:line` and its enclosing `fn`. Adding a release build to
/// PR CI would cost too much; the walk is the cheap substitute.
///
/// # What it does and does not pin
///
/// The debug pin is NOT weakened: in a debug build it demands exactly `n`.
/// But `0` is nearly vacuous in release — measured on `unknown_hash`: with
/// only this guard, mutating the suppressed arm from `debug!` to `warn!`
/// PASSED in release, because that demoted line carries the suppressed
/// MESSAGE, which no WARN-keyed count looks for. So a caller must ALSO pin,
/// level-free, what carries the contract in release: that the suppressed
/// message never appears at a LOUD level (an unconditional never-loud sweep
/// over WARN/INFO/ERROR), and the latch's own unconditional counter
/// (`FailureRegimeLatch::total_failures` / `is_failing`, or the site's
/// per-entity counter, on the rule that a counter is the
/// release-safe complement to a log assertion).
pub fn debug_lines_expected(n: usize) -> usize {
    if debug_level_compiled_in() {
        n
    } else {
        0
    }
}

/// The five `tracing` level tokens, loudest first — a LIST, so the source walk
/// in `crates/cerulion_core/tests/debug_count_discipline_test.rs` reads it as the
/// level table it is, not as a DEBUG selector.
const LEVEL_TOKENS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];

/// The LEVEL token of one captured log line, or `None` if its header carries
/// none.
///
/// `tracing-test`'s subscriber and the cdylib-local stderr subscriber both
/// render `<timestamp> <LEVEL> <span>: <target>: <message> <fields>` with
/// ANSI off, and the SPAN is the test function's own name. Testing the level
/// with a bare `line.contains("ERROR")` therefore counts a line whose span
/// name, topic, type name or any field VALUE happens to contain the token, so
/// a future rename or an uppercase field value could silently invert an
/// "exactly N" oracle in either direction. This reads whole whitespace tokens
/// out of the line HEADER (everything before the first `": "`: the timestamp,
/// the level and the span), so it can never match inside an identifier and a
/// message body never votes.
///
/// A line whose shape this cannot parse yields `None` and matches no level.
/// That direction is safe for PRESENCE and COUNT oracles — an unparseable
/// header zeroes them, so they fail rather than pass — but NOT for an absence
/// guard (`count_at(..) == 0` would also read zero and pass), so pair every
/// absence guard with a positive count over the same capture.
pub fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVEL_TOKENS.into_iter().find(|level| *level == token))
}

/// Count captured lines carrying BOTH the level token (see [`line_level`])
/// and the marker.
///
/// The level is part of the predicate on purpose: matching text alone lets
/// a SUPPRESSED arm emitted at `warn!` — where
/// suppression does nothing while the message and the counters stay intact —
/// pass an "exactly N at DEBUG" oracle, and a loud head demoted to `info!`
/// pass an "exactly one warn" one.
pub fn count_at(lines: &[&str], level: &str, marker: &str) -> usize {
    lines
        .iter()
        .filter(|l| line_level(l) == Some(level) && l.contains(marker))
        .count()
}

/// `Ok` iff some captured line carries `marker` at `level` — for use inside
/// `logs_assert`. "Loud" is a LEVEL claim: a bare `logs_contain(marker)` would
/// pass a `warn!` demoted to `info!`.
pub fn logged_at(lines: &[&str], level: &str, marker: &str) -> Result<(), String> {
    if count_at(lines, level, marker) > 0 {
        Ok(())
    } else {
        Err(format!(
            "expected a {level} line carrying {marker:?}; got {lines:?}"
        ))
    }
}

/// Lines carrying ALL of `markers` at `level`, in capture order — REFUSING if
/// a line carrying the same markers sits at any OTHER level.
///
/// # The regression this closes
///
/// A level-free `lines.filter(|l| l.contains(marker)).count() == 1` oracle
/// rejects a SECOND copy of that line wherever it is emitted. Replacing it
/// with a level-qualified count — the guard against a loud head silently demoted to
/// `info!` — buys the level and LOSES that: one WARN beside a stray INFO copy
/// of the same message still reads exactly 1. Both properties are wanted at
/// every "exactly one loud line" site, and pairing them by hand is a step that
/// gets forgotten at some sites and not
/// others. Doing both in ONE call means the pairing cannot be dropped without
/// deleting the call.
///
/// `markers` is a CONJUNCTION, and the exclusivity is over that same
/// conjunction: a site counting "the degraded warn FOR THIS TOPIC" must not be
/// failed by the same message legitimately emitted for a different topic.
///
/// NOT for an absence guard. A marker with a legitimate twin at another level
/// — a suppressed DEBUG repeat beside a loud head — makes this return `Err`,
/// not 0, so `count_at(.., level, marker) == 0` stays THE form for "never at
/// this level". A capture carrying the markers NOWHERE is `Ok(0)`.
///
/// A MATCHING line whose header carries no level token (see [`line_level`]) is
/// reported as a copy at another level rather than silently ignored — the
/// direction that fails loudly.
pub fn lines_at_exclusively<'a>(
    lines: &[&'a str],
    level: &str,
    markers: &[&str],
) -> Result<Vec<&'a str>, String> {
    if markers.is_empty() {
        // `all` over an empty slice is `true`, which would silently mean "every
        // line at this level, and Err if ANY line is at another" — a predicate
        // no caller wants and none could have meant.
        return Err("an exclusive level count needs at least one marker".to_string());
    }
    let matching: Vec<&'a str> = lines
        .iter()
        .copied()
        .filter(|l| markers.iter().all(|m| l.contains(m)))
        .collect();
    let (at_level, elsewhere): (Vec<&'a str>, Vec<&'a str>) = matching
        .iter()
        .copied()
        .partition(|l| line_level(l) == Some(level));
    if !elsewhere.is_empty() {
        // Name BOTH causes and each offender's LEVEL. A marker that is a TAIL
        // the loud head shares with its own suppressed twin is neither a duplicate
        // emission nor a demotion, and a message saying only "copy" would point
        // away from the remedy, which is to narrow the marker.
        let shown: Vec<String> = elsewhere
            .iter()
            .map(|l| format!("[{}] {l}", line_level(l).unwrap_or("<no level token>")))
            .collect();
        return Err(format!(
            "{} line(s) carry {markers:?}: {} at {level}, {} elsewhere — either the markers are \
             also carried by a line at ANOTHER level (narrow them to this level's own message) \
             or the line was emitted twice: {shown:?}",
            matching.len(),
            at_level.len(),
            elsewhere.len(),
        ));
    }
    Ok(at_level)
}

/// How many lines carry ALL of `markers` at `level` — refusing, exactly as
/// [`lines_at_exclusively`] does, if a copy sits at another level. THE form for
/// an "exactly N loud lines" oracle: it is the level-token count and the
/// level-free total in one call.
pub fn count_at_exclusively(
    lines: &[&str],
    level: &str,
    markers: &[&str],
) -> Result<usize, String> {
    lines_at_exclusively(lines, level, markers).map(|matched| matched.len())
}

/// UNCONDITIONAL, level-independent: a suppressed repeat must never be LOUD.
///
/// The half of the suppression contract that survives
/// `release_max_level_info`, where every DEBUG count reads 0 and a suppressed
/// arm promoted to `info!`/`warn!`/`error!` would otherwise pass. Paired with
/// the gated DEBUG count at every site ([`debug_lines_expected`]): the source
/// walk in `crates/cerulion_core/tests/debug_count_discipline_test.rs` requires a
/// call to a helper of this NAME, carrying the site's marker, beside every
/// gated site — it checks the name and the argument, not this body, which is
/// why the body lives in ONE place.
pub fn never_loud(lines: &[&str], marker: &str) -> Result<(), String> {
    for level in ["WARN", "INFO", "ERROR"] {
        let n = count_at(lines, level, marker);
        if n != 0 {
            return Err(format!(
                "a suppressed repeat was emitted at {level} ({n} line(s)) for marker {marker:?}"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod robust_ratio_tests {
    use super::*;

    #[test]
    fn floor_ns_returns_minimum() {
        assert_eq!(floor_ns(&[3.0, 1.0, 2.0]), 1.0);
    }

    #[test]
    fn floor_ns_empty_is_f64_max() {
        assert_eq!(floor_ns(&[]), f64::MAX);
    }

    #[test]
    fn robust_ratio_drops_single_high_outlier() {
        // After sort by floor: 10.0, 10.5, 11.0, 100.0.
        let floors = [(64, 10.0), (1024, 11.0), (4096, 10.5), (65536, 100.0)];
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert_eq!(rf.min, (64, 10.0));
        assert_eq!(rf.dropped, (65536, 100.0));
        assert_eq!(rf.robust_max, (1024, 11.0));
        assert!((rf.ratio - 1.1).abs() < 1e-9, "ratio = {}", rf.ratio);
    }

    #[test]
    fn robust_ratio_still_catches_monotonic_copy() {
        // A real O(n) copy inflates EVERY size; dropping the worst (4000) still
        // leaves robust_max=2000 / min=10 == 200x.
        let floors = [(64, 10.0), (1024, 1000.0), (4096, 2000.0), (65536, 4000.0)];
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert_eq!(rf.min, (64, 10.0));
        assert_eq!(rf.dropped, (65536, 4000.0));
        assert_eq!(rf.robust_max, (4096, 2000.0));
        assert!((rf.ratio - 200.0).abs() < 1e-9, "ratio = {}", rf.ratio);
    }

    #[test]
    fn robust_ratio_three_entries_minimum_valid_size() {
        // n=3 is the smallest valid input: robust_max = sorted[len-2] =
        // sorted[1] sits ADJACENT to min = sorted[0]. Pins the smallest-valid
        // indexing directly (the n=4 tests leave len-2 == 2). Sorted by floor:
        // 5.0, 6.0, 7.0 → min=(1,5), robust_max=(3,6), dropped=(2,7).
        let floors = [(1, 5.0), (2, 7.0), (3, 6.0)];
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert_eq!(rf.min, (1, 5.0));
        assert_eq!(rf.robust_max, (3, 6.0));
        assert_eq!(rf.dropped, (2, 7.0));
        assert!((rf.ratio - 1.2).abs() < 1e-9, "ratio = {}", rf.ratio);
    }

    #[test]
    #[should_panic(expected = "needs >= 3 sizes")]
    fn robust_ratio_panics_below_three_entries() {
        let _ = drop_one_outlier_robust_ratio(&[(1, 5.0), (2, 6.0)]);
    }

    #[test]
    fn robust_ratio_zero_min_is_infinite() {
        let rf = drop_one_outlier_robust_ratio(&[(1, 0.0), (2, 5.0), (3, 6.0)]);
        assert!(rf.ratio.is_infinite(), "ratio = {}", rf.ratio);
    }

    // -------- Window-health retry + uniform-stall discriminator --------
    //
    // Oracle vectors use the REAL sweep sizes; floor values are in µs (ratios
    // are unit-invariant, so the ns/µs distinction is irrelevant to the math).
    const SWEEP: [usize; 5] = [1_024, 16_384, 262_144, 1_048_576, 16_777_216];

    /// A measured macOS VM stall shape: [16.9, 43.3, 42.8, 43.1,
    /// 43.7]µs (uniform ~43µs, min 16.9). At the cross_thread ceiling K=2.0 the
    /// retry set is the FOUR elevated (bigger) sizes; the smallest (16.9, the
    /// min) is not retried.
    #[test]
    fn observed_macos_stall_retry_set_is_four_sizes() {
        let floors = [
            (SWEEP[0], 16.9),
            (SWEEP[1], 43.3),
            (SWEEP[2], 42.8),
            (SWEEP[3], 43.1),
            (SWEEP[4], 43.7),
        ];
        // K = 2.0 (cross_thread ceiling): min=16.9, threshold=33.8.
        assert_eq!(
            sizes_needing_retry(&floors, 2.0),
            vec![SWEEP[1], SWEEP[2], SWEEP[3], SWEEP[4]]
        );
        // K = 1.5 (flat_latency ceiling): threshold=25.35 — same four elevated.
        assert_eq!(
            sizes_needing_retry(&floors, 1.5),
            vec![SWEEP[1], SWEEP[2], SWEEP[3], SWEEP[4]]
        );
    }

    /// The observed macOS stall, if the retries do NOT heal it, carries the
    /// uniform (size-independent) signature at BOTH gate ceilings → the
    /// non-probative arm fires. The drop-one gate is also genuinely violated.
    #[test]
    fn observed_macos_stall_is_uniform_signature() {
        let floors = [
            (SWEEP[0], 16.9),
            (SWEEP[1], 43.3),
            (SWEEP[2], 42.8),
            (SWEEP[3], 43.1),
            (SWEEP[4], 43.7),
        ];
        assert!(is_uniform_stall_signature(&floors, 2.0));
        assert!(is_uniform_stall_signature(&floors, 1.5));
        // And the drop-one gate really fails (robust_max=43.3, min=16.9 ≈ 2.56x):
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert!(rf.ratio > 2.0, "ratio = {}", rf.ratio);
    }

    /// A real monotonic-in-size copy [17, 25, 60, 200, 800]µs: the retry set
    /// fires per the rule, but the shape is NOT uniform (the elevated floors
    /// span 13.3×, far beyond the 1.3× band) → gate must FAIL normally.
    #[test]
    fn monotonic_copy_retry_set_but_not_uniform() {
        let floors = [
            (SWEEP[0], 17.0),
            (SWEEP[1], 25.0),
            (SWEEP[2], 60.0),
            (SWEEP[3], 200.0),
            (SWEEP[4], 800.0),
        ];
        // K=2.0: min=17, threshold=34 → 60/200/800 elevated (25 < 34 is not).
        assert_eq!(
            sizes_needing_retry(&floors, 2.0),
            vec![SWEEP[2], SWEEP[3], SWEEP[4]]
        );
        // NOT uniform: elevated {60,200,800} span 13.3× ≫ 1.3× band.
        assert!(!is_uniform_stall_signature(&floors, 2.0));
        assert!(!is_uniform_stall_signature(&floors, 1.5));
        // The gate genuinely fails (robust_max=200/min=17 ≈ 11.8x) — a REAL copy.
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert!(rf.ratio > 2.0, "ratio = {}", rf.ratio);
    }

    /// A healthy flat sweep [16.9, 17.2, 17.0, 18.1, 17.5]µs → zero retries,
    /// not uniform, and the drop-one gate passes at BOTH ceilings.
    #[test]
    fn healthy_sweep_no_retry_and_passes() {
        let floors = [
            (SWEEP[0], 16.9),
            (SWEEP[1], 17.2),
            (SWEEP[2], 17.0),
            (SWEEP[3], 18.1),
            (SWEEP[4], 17.5),
        ];
        assert!(sizes_needing_retry(&floors, 1.5).is_empty());
        assert!(sizes_needing_retry(&floors, 2.0).is_empty());
        assert!(!is_uniform_stall_signature(&floors, 1.5));
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert!(rf.ratio < 1.5, "ratio = {}", rf.ratio);
    }

    /// The classic single-outlier case [17, 17.5, 42, 17.2, 18]µs
    /// (one stalled MIDDLE size): drop-one still discards the outlier and the
    /// gate PASSES — verdict unchanged. The one elevated size is retried (a
    /// cheap, harmless cleanup) but is NOT the uniform signature (< 3 elevated).
    #[test]
    fn single_outlier_verdict_unchanged() {
        let floors = [
            (SWEEP[0], 17.0),
            (SWEEP[1], 17.5),
            (SWEEP[2], 42.0),
            (SWEEP[3], 17.2),
            (SWEEP[4], 18.0),
        ];
        // Exactly the one stalled middle size is a retry candidate.
        assert_eq!(sizes_needing_retry(&floors, 2.0), vec![SWEEP[2]]);
        // One elevated floor is not the uniform signature.
        assert!(!is_uniform_stall_signature(&floors, 2.0));
        // Drop-one drops the 42µs outlier → robust_max=18/min=17 → gate PASSES.
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert_eq!(rf.dropped, (SWEEP[2], 42.0));
        assert!(rf.ratio < 2.0, "ratio = {}", rf.ratio);
    }

    /// Edge: all-equal floors → no retry, not uniform, gate ratio 1.0.
    #[test]
    fn all_equal_floors_no_retry_not_uniform() {
        let floors = [
            (SWEEP[0], 20.0),
            (SWEEP[1], 20.0),
            (SWEEP[2], 20.0),
            (SWEEP[3], 20.0),
            (SWEEP[4], 20.0),
        ];
        assert!(sizes_needing_retry(&floors, 1.5).is_empty());
        assert!(!is_uniform_stall_signature(&floors, 1.5));
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert!((rf.ratio - 1.0).abs() < 1e-9, "ratio = {}", rf.ratio);
    }

    /// Edge: `sizes_needing_retry` and `is_uniform_stall_signature` handle a
    /// 2-size list without panicking (only `drop_one_outlier_robust_ratio`
    /// requires >= 3). The big size is a retry candidate; 1 elevated < 3 so the
    /// uniform signature is false.
    #[test]
    fn two_size_list_is_handled() {
        let floors = [(SWEEP[0], 10.0), (SWEEP[1], 50.0)];
        assert_eq!(sizes_needing_retry(&floors, 2.0), vec![SWEEP[1]]);
        assert!(!is_uniform_stall_signature(&floors, 2.0));
    }

    /// Edge: degenerate/zero + empty floors are guarded — no retry, not uniform,
    /// no panic (the caller's gate then fails loudly on the infinite ratio).
    #[test]
    fn zero_and_empty_floors_guarded() {
        let all_zero = [(SWEEP[0], 0.0), (SWEEP[1], 0.0), (SWEEP[2], 0.0)];
        assert!(sizes_needing_retry(&all_zero, 2.0).is_empty());
        assert!(!is_uniform_stall_signature(&all_zero, 2.0));
        let empty: [(usize, f64); 0] = [];
        assert!(sizes_needing_retry(&empty, 2.0).is_empty());
        assert!(!is_uniform_stall_signature(&empty, 2.0));
    }

    /// Boundary: exactly 3 elevated floors JUST inside the band → uniform; the
    /// same 3 with one floor pushed JUST outside the band → not uniform. Pins
    /// the `UNIFORM_STALL_BAND` edge directly.
    #[test]
    fn uniform_band_boundary() {
        // min=10, K=2.0 → threshold=20. Elevated {40, 44, 48}: band 48/40=1.2 < 1.3.
        let inside = [
            (SWEEP[0], 10.0),
            (SWEEP[1], 40.0),
            (SWEEP[2], 44.0),
            (SWEEP[3], 48.0),
            (SWEEP[4], 10.0),
        ];
        assert!(is_uniform_stall_signature(&inside, 2.0));
        // Push the top elevated floor out: {40, 44, 60}: band 60/40=1.5 > 1.3.
        let outside = [
            (SWEEP[0], 10.0),
            (SWEEP[1], 40.0),
            (SWEEP[2], 44.0),
            (SWEEP[3], 60.0),
            (SWEEP[4], 10.0),
        ];
        assert!(!is_uniform_stall_signature(&outside, 2.0));
    }

    /// Window-health retry orchestration HEALS: the observed stall shape with a
    /// remeasure closure that returns a clean 17µs floor. Each of the 4 elevated
    /// sizes is re-measured ONCE (attempt 1 heals it back under 2.0×min), the
    /// stats are updated to the clean floor, and the sweep is flat afterwards.
    #[test]
    fn retry_orchestration_heals_in_one_attempt() {
        let mut stats = vec![
            (SWEEP[0], 16.9, 16.9, 16.9, 16.9),
            (SWEEP[1], 43.3, 43.3, 43.3, 43.3),
            (SWEEP[2], 42.8, 42.8, 42.8, 42.8),
            (SWEEP[3], 43.1, 43.1, 43.1, 43.1),
            (SWEEP[4], 43.7, 43.7, 43.7, 43.7),
        ];
        // Canned: every re-measurement lands a clean 17.0µs (no transport).
        let log = run_window_health_retries(&mut stats, 2.0, 2, |_size| (17.0, 17.0, 17.0, 17.0));
        assert_eq!(log.len(), 4, "one healing attempt per elevated size");
        assert!(log.iter().all(|r| r.attempt == 1));
        assert!(log.iter().all(|r| (r.kept_floor - 17.0).abs() < 1e-9));
        // Stats updated; the sweep is now flat (all floors 16.9 or 17.0).
        let final_floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(!is_uniform_stall_signature(&final_floors, 2.0));
        let rf = drop_one_outlier_robust_ratio(&final_floors);
        assert!(rf.ratio < 2.0, "healed ratio = {}", rf.ratio);
    }

    /// Window-health retry orchestration does NOT heal: the remeasure closure
    /// keeps returning an elevated 40µs floor. Each of the 4 elevated sizes
    /// exhausts all `max_retries` (2) attempts, and the final shape still
    /// carries the uniform signature (so the caller takes the non-probative arm).
    #[test]
    fn retry_orchestration_exhausts_when_unhealed() {
        let mut stats = vec![
            (SWEEP[0], 16.9, 16.9, 16.9, 16.9),
            (SWEEP[1], 43.3, 43.3, 43.3, 43.3),
            (SWEEP[2], 42.8, 42.8, 42.8, 42.8),
            (SWEEP[3], 43.1, 43.1, 43.1, 43.1),
            (SWEEP[4], 43.7, 43.7, 43.7, 43.7),
        ];
        // Canned: every re-measurement stays elevated (40µs, still > 2.0×16.9).
        let log = run_window_health_retries(&mut stats, 2.0, 2, |_size| (40.0, 40.0, 40.0, 40.0));
        assert_eq!(log.len(), 8, "2 attempts x 4 elevated sizes, none healed");
        // Final shape is still a uniform stall → the non-probative arm fires.
        let final_floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(is_uniform_stall_signature(&final_floors, 2.0));
    }

    /// A healthy sweep triggers ZERO retries (the remeasure closure is never
    /// called — proven by panicking inside it). Under the per-round recompute
    /// this is the "first recompute returns empty" pin: a healthy sweep pays
    /// nothing.
    #[test]
    fn retry_orchestration_no_retries_on_healthy_sweep() {
        let mut stats = vec![
            (SWEEP[0], 16.9, 16.9, 16.9, 16.9),
            (SWEEP[1], 17.2, 17.2, 17.2, 17.2),
            (SWEEP[2], 17.0, 17.0, 17.0, 17.0),
            (SWEEP[3], 18.1, 18.1, 18.1, 18.1),
            (SWEEP[4], 17.5, 17.5, 17.5, 17.5),
        ];
        let log = run_window_health_retries(&mut stats, 1.5, 2, |_size| {
            panic!("healthy sweep must not re-measure any size");
        });
        assert!(log.is_empty());
    }

    /// The min-shift vector, HEAL arm: initial floors
    /// [20, 20.5, 20.2, 31, 20.3] PASS drop-one at 20.3/20 = 1.025×; the 31
    /// retries down to 13 → the min (ratio denominator) drops → the four ~20
    /// floors become would-fail (20.x/13 up to 1.58× >= 1.5). A
    /// FROZEN retry set would never re-measure them and the would-pass run would flip
    /// to the non-probative red. With the per-round recompute they are
    /// re-measured in the second round; their fresh windows also find 13-class floors
    /// (a uniformly-jittery machine) → the sweep heals to flat and PASSES legitimately.
    #[test]
    fn min_shift_recompute_heals_to_pass() {
        let mut stats = vec![
            (SWEEP[0], 20.0, 20.0, 20.0, 20.0),
            (SWEEP[1], 20.5, 20.5, 20.5, 20.5),
            (SWEEP[2], 20.2, 20.2, 20.2, 20.2),
            (SWEEP[3], 31.0, 31.0, 31.0, 31.0),
            (SWEEP[4], 20.3, 20.3, 20.3, 20.3),
        ];
        // Sanity: the INITIAL sweep already passes drop-one at the 1.5 ceiling
        // (the flaw only bites BECAUSE the retry lowers the min).
        let initial: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(drop_one_outlier_robust_ratio(&initial).ratio < 1.5);
        // Canned fresh windows: the elevated size finds 13; the others, when
        // re-asked after the min shift, find their own 13-class floors.
        let log = run_window_health_retries(&mut stats, 1.5, 2, |size| {
            let f = if size == SWEEP[0] {
                13.2
            } else if size == SWEEP[1] {
                13.1
            } else if size == SWEEP[2] {
                13.3
            } else {
                // SWEEP[3] (the initially-elevated size) and SWEEP[4] both
                // find 13.0 in their fresh windows.
                13.0
            };
            (f, f, f, f)
        });
        // Round 1: only the 31 is elevated (threshold 1.5×20 = 30). Round 2:
        // min=13 → threshold 19.5 → the four ~20 floors retried. Round 3: all
        // floors 13.0–13.3 → empty set → stop. 5 attempts total, each size 1.
        assert_eq!(log.len(), 5);
        assert_eq!(log[0].size, SWEEP[3]);
        assert_eq!(log[0].round, 1);
        assert!(log.iter().all(|r| r.attempt == 1));
        assert!(log[1..].iter().all(|r| r.round == 2));
        // The final sweep PASSES the 1.5× gate (the heal path) and is not
        // uniform-stall shaped.
        let final_floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        let rf = drop_one_outlier_robust_ratio(&final_floors);
        assert!(rf.ratio < 1.5, "healed ratio = {}", rf.ratio);
        assert!(!is_uniform_stall_signature(&final_floors, 1.5));
    }

    /// The min-shift vector, GENUINE-FAIL arm: same initial floors, the 31
    /// retries to 13, but the four ~20 floors STAY ~20 on re-measure (genuinely
    /// divergent, not transient jitter). Each gets its full per-size budget
    /// (2 attempts) across rounds 2-3, budgets exhaust, and the gate fails —
    /// but now on floors that were all given EQUAL re-measure opportunity, and
    /// the uniform-stall signature still routes it to the attributable panic.
    #[test]
    fn min_shift_recompute_honest_fail_stays_attributable() {
        let mut stats = vec![
            (SWEEP[0], 20.0, 20.0, 20.0, 20.0),
            (SWEEP[1], 20.5, 20.5, 20.5, 20.5),
            (SWEEP[2], 20.2, 20.2, 20.2, 20.2),
            (SWEEP[3], 31.0, 31.0, 31.0, 31.0),
            (SWEEP[4], 20.3, 20.3, 20.3, 20.3),
        ];
        // Canned: the elevated size heals to 13; the others re-measure to their
        // own old values (no improvement).
        let log = run_window_health_retries(&mut stats, 1.5, 2, |size| {
            let f = if size == SWEEP[0] {
                20.0
            } else if size == SWEEP[1] {
                20.5
            } else if size == SWEEP[2] {
                20.2
            } else if size == SWEEP[3] {
                13.0
            } else {
                20.3 // SWEEP[4]
            };
            (f, f, f, f)
        });
        // Round 1: the 31 → 13 (1 attempt). Rounds 2-3: the four ~20 floors,
        // attempts 1 and 2 each, no improvement. Round 4: still elevated but
        // budgets exhausted → stop. 1 + 4 + 4 = 9 attempts; per-size <= 2.
        assert_eq!(log.len(), 9);
        assert!(log.iter().all(|r| r.attempt <= 2));
        assert!(log[1..5].iter().all(|r| r.round == 2 && r.attempt == 1));
        assert!(log[5..].iter().all(|r| r.round == 3 && r.attempt == 2));
        // The gate FAILS on the final floors (20.3/13 ≈ 1.56×)...
        let final_floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        let rf = drop_one_outlier_robust_ratio(&final_floors);
        assert!(rf.ratio >= 1.5, "ratio = {}", rf.ratio);
        // ...and the uniform-stall signature routes it to the ATTRIBUTABLE
        // non-probative arm (4 elevated floors within a 20.5/20 = 1.025× band).
        assert!(is_uniform_stall_signature(&final_floors, 1.5));
    }

    // -------- `classify_flatness` FULL-RATIO gate DECISION -----------
    //
    // The CLI-e2e graph-latency gate (`cli_e2e_graph_latency_test.rs`) sweeps
    // FOUR payload sizes, each a separate SUBPROCESS run, and gates the per-size
    // FLOOR flatness. The gate uses the FULL max/min ratio (NO drop-one — see the
    // `classify_flatness` rustdoc: this sweep's 2nd-largest size is 16× smaller
    // than its largest, so a real O(n) copy's 2nd-largest signal is below the
    // base RTT and drop-one would hide the copy). Single-window stalls are
    // absorbed BEFORE classify by the window-health retry; a stall that survives
    // all retries fails as `RealCopy` (the conservative residual). These vectors
    // pin the composed decision at BOTH ceilings (virtual 1.5×, real 5.0×)
    // against hand oracles.
    const CLI_SWEEP: [usize; 4] = [64, 4_096, 65_536, 1_048_576];

    /// The headline regression pin: a REALISTIC top-dominant O(n) copy
    /// — 1 MiB ≈ +160 µs of linear copy over a ~20 µs base, so 64 KiB (16× smaller)
    /// carries only ~10 µs, BELOW the base ⇒ floors [20, 20.4, 26, 180]µs. The
    /// FULL max/min ratio (180/20 = 9×) → `RealCopy` at BOTH ceilings. This EXACT
    /// vector PASSES under a drop-one gate (drop the
    /// 180 → 2nd-max collapses to 26 → 26/20 = 1.3× < 1.5×), silently masking a
    /// real copy — which is why this gate does not use drop-one.
    #[test]
    fn classify_realistic_top_size_copy_fails() {
        let floors = [
            (CLI_SWEEP[0], 20.0),
            (CLI_SWEEP[1], 20.4),
            (CLI_SWEEP[2], 26.0),
            (CLI_SWEEP[3], 180.0),
        ];
        // Full max/min catches it at BOTH ceilings.
        assert!(matches!(
            classify_flatness(&floors, 1.5),
            FlatnessVerdict::RealCopy(_)
        ));
        assert!(matches!(
            classify_flatness(&floors, 5.0),
            FlatnessVerdict::RealCopy(_)
        ));
        // Documents the hole: the drop-one ratio on this vector is a passing
        // ~1.3× (26/20), i.e. drop-one would have MISSED this copy.
        let rf = drop_one_outlier_robust_ratio(&floors);
        assert!(
            rf.ratio < 1.5,
            "drop-one ratio {} should be a (wrongly) passing ~1.3x — the HIGH",
            rf.ratio
        );
    }

    /// The residual (documented in the CLI-e2e module doc): an UNHEALED
    /// single-window stall — [20, 20, 40, 20]µs, 65_536's window persistently
    /// elevated 20→40µs — is indistinguishable from a real single-size regression
    /// once the window-health retry has already re-measured it (pre-retry stalls
    /// are healed BEFORE classify sees them). At the TIGHT virtual ceiling it
    /// fails conservatively as `RealCopy` (a human re-runs to disambiguate); at
    /// the LOOSE real ceiling (5.0×, a catastrophe backstop) the same 2× elevation
    /// is absorbed → `Pass`. Contrast with `classify_realistic_top_size_copy_fails`
    /// (which the old drop-one gate would have WRONGLY passed).
    #[test]
    fn classify_unhealed_single_elevation_fails_as_real_copy() {
        let floors = [
            (CLI_SWEEP[0], 20.0),
            (CLI_SWEEP[1], 20.0),
            (CLI_SWEEP[2], 40.0),
            (CLI_SWEEP[3], 20.0),
        ];
        // Virtual (tight): full ratio 40/20 = 2.0× > 1.5× and only ONE elevated
        // floor (< UNIFORM_STALL_MIN_SIZES) → not the uniform signature → RealCopy.
        assert!(matches!(
            classify_flatness(&floors, 1.5),
            FlatnessVerdict::RealCopy(_)
        ));
        // Real (loose catastrophe backstop): 2.0× < 5.0× → Pass. The loose ceiling
        // deliberately absorbs a single 2× stall on the jittery live path.
        assert!(matches!(
            classify_flatness(&floors, 5.0),
            FlatnessVerdict::Pass(_)
        ));
    }

    /// A steep monotonic O(n) memcpy — floors scale with payload
    /// [20, 120, 1_800, 28_000]µs. The FULL max/min ratio (28_000/20 = 1400×) →
    /// `RealCopy` at BOTH ceilings (and it is NOT the uniform signature — the
    /// elevated floors span far beyond the 1.3× band). The gate catches a genuine
    /// regression.
    #[test]
    fn classify_monotonic_copy_still_fails_as_real_copy() {
        let floors = [
            (CLI_SWEEP[0], 20.0),
            (CLI_SWEEP[1], 120.0),
            (CLI_SWEEP[2], 1_800.0),
            (CLI_SWEEP[3], 28_000.0),
        ];
        assert!(matches!(
            classify_flatness(&floors, 1.5),
            FlatnessVerdict::RealCopy(_)
        ));
        assert!(matches!(
            classify_flatness(&floors, 5.0),
            FlatnessVerdict::RealCopy(_)
        ));
    }

    /// THREE of four windows uniformly stalled ~43µs (min 20µs) — a VM stall
    /// spanning multiple subprocess windows that SURVIVED retries. The full ratio
    /// (43.4/20 = 2.17×) exceeds the ceiling, but the signature is
    /// size-INDEPENDENT (>= 3 elevated floors within the 1.3× band) →
    /// `UniformStall` (attributable non-probative), NOT `RealCopy`.
    #[test]
    fn classify_uniform_multi_window_stall_is_attributable() {
        let floors = [
            (CLI_SWEEP[0], 20.0),
            (CLI_SWEEP[1], 43.0),
            (CLI_SWEEP[2], 42.5),
            (CLI_SWEEP[3], 43.4),
        ];
        assert!(matches!(
            classify_flatness(&floors, 1.5),
            FlatnessVerdict::UniformStall(_)
        ));
        // The full-ratio gate is genuinely violated (max 43.4 / min 20 ≈ 2.17×).
        assert!(is_uniform_stall_signature(&floors, 1.5));
    }

    /// A healthy flat sweep [20, 20.3, 20.1, 20.5]µs → `Pass` at both ceilings,
    /// and the carried `FullFlatness` reports the true min (64 B) and max (1 MiB).
    #[test]
    fn classify_healthy_flat_sweep_passes() {
        let floors = [
            (CLI_SWEEP[0], 20.0),
            (CLI_SWEEP[1], 20.3),
            (CLI_SWEEP[2], 20.1),
            (CLI_SWEEP[3], 20.5),
        ];
        match classify_flatness(&floors, 1.5) {
            FlatnessVerdict::Pass(ff) => {
                assert_eq!(ff.min, (CLI_SWEEP[0], 20.0));
                assert_eq!(ff.max, (CLI_SWEEP[3], 20.5));
                assert!(ff.ratio < 1.5, "ratio = {}", ff.ratio);
            }
            other => panic!("expected Pass, got {other:?}"),
        }
    }

    /// The retry layer is now the SOLE single-window-stall defense (drop-one is
    /// gone), so pin the heal path end-to-end THROUGH the gate decision: a would-
    /// fail single-window stall [20, 20, 40, 20]µs whose fresh re-measure returns a
    /// healthy 20µs floor ⇒ final floors flat ⇒ `classify_flatness` == `Pass`.
    /// The injected `remeasure` closure stands in for the fresh subprocess.
    #[test]
    fn classify_after_retry_heals_stall_to_pass() {
        let mut stats = vec![
            (CLI_SWEEP[0], 20.0, 20.0, 20.0, 20.0),
            (CLI_SWEEP[1], 20.0, 20.0, 20.0, 20.0),
            (CLI_SWEEP[2], 40.0, 40.0, 40.0, 40.0), // the stalled window
            (CLI_SWEEP[3], 20.0, 20.0, 20.0, 20.0),
        ];
        // Pre-retry, the sweep would FAIL the tight ceiling as RealCopy.
        let pre: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(matches!(
            classify_flatness(&pre, 1.5),
            FlatnessVerdict::RealCopy(_)
        ));
        // Fresh window clears the stall: 65_536 re-measures at a healthy 20µs.
        // `2` mirrors the CLI-e2e gate's MAX_RETRIES per-size budget.
        let log = run_window_health_retries(&mut stats, 1.5, 2, |_size| (20.0, 20.0, 20.0, 20.0));
        assert_eq!(
            log.len(),
            1,
            "exactly the one stalled size re-measured once"
        );
        assert_eq!(log[0].size, CLI_SWEEP[2]);
        // Post-retry the sweep is flat ⇒ classify PASSES at BOTH ceilings.
        let post: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(matches!(
            classify_flatness(&post, 1.5),
            FlatnessVerdict::Pass(_)
        ));
        assert!(matches!(
            classify_flatness(&post, 5.0),
            FlatnessVerdict::Pass(_)
        ));
    }

    // ─── L23: fixture-cdylib resolution ────────────────────────────────

    #[test]
    fn cdylib_file_name_matches_the_platform_convention() {
        let name = cdylib_file_name("test_node_cdylib");
        // Hand oracle per platform — not a re-derivation of the function.
        #[cfg(target_os = "macos")]
        assert_eq!(name, "libtest_node_cdylib.dylib");
        #[cfg(target_os = "windows")]
        assert_eq!(name, "test_node_cdylib.dll");
        #[cfg(not(any(target_os = "macos", target_os = "windows")))]
        assert_eq!(name, "libtest_node_cdylib.so");
    }

    /// The derived directory must be an ANCESTOR of the running test binary —
    /// which is the whole point: it follows wherever cargo actually put this
    /// invocation's artifacts (isolated `CARGO_TARGET_DIR`, `build.target-dir`,
    /// `--target <triple>`), instead of a hardcoded `<repo_root>/target`.
    #[test]
    fn test_target_profile_dir_is_an_ancestor_of_the_running_test_binary() {
        let exe = std::env::current_exe().expect("current_exe");
        let dir = test_target_profile_dir();
        assert!(
            exe.starts_with(&dir),
            "derived profile dir {} is not an ancestor of the running binary {}",
            dir.display(),
            exe.display()
        );
        // Exactly one level (binary directly in the profile dir) or two (the
        // usual `<profile>/deps/<bin>`) — never an arbitrary walk-up.
        let depth = exe
            .strip_prefix(&dir)
            .expect("ancestor")
            .components()
            .count();
        assert!(
            depth == 1 || depth == 2,
            "expected <profile>/<bin> or <profile>/deps/<bin>, got depth {depth}              for {} under {}",
            exe.display(),
            dir.display()
        );
        // A `deps` component must never survive into the returned dir.
        assert_ne!(dir.file_name(), Some(std::ffi::OsStr::new("deps")));
    }

    /// A missing fixture must name the DERIVED search root, not a repo-relative
    /// one — the anti-tautology half of the test above: it proves the lookup
    /// (not merely the accessor) is keyed to this binary's own target dir.
    ///
    /// This test deliberately triggers the locator's panic, so a
    /// `fixture cdylib \`libl23_no_such_fixture_crate.*\` not found` line on
    /// stderr is EXPECTED. The default panic hook is left installed on purpose:
    /// swapping it is process-global and would race sibling tests, suppressing a
    /// genuine panic message elsewhere in the binary.
    #[test]
    fn a_missing_fixture_panics_naming_the_derived_target_dir() {
        let dir = test_target_profile_dir();
        let err = std::panic::catch_unwind(|| {
            find_fixture_cdylib("l23_no_such_fixture_crate");
        })
        .expect_err("a missing fixture must panic");
        let msg = err
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| err.downcast_ref::<&str>().map(|s| (*s).to_string()))
            .expect("panic payload is a string");
        assert!(
            msg.contains(&dir.display().to_string()),
            "panic must name the derived target dir {}; got: {msg}",
            dir.display()
        );
        assert!(
            msg.contains("cargo build -p l23_no_such_fixture_crate"),
            "panic must name the build command; got: {msg}"
        );
    }

    /// The same-profile locator REFUSES rather than falling back: its panic
    /// names the running profile's path and the no-sibling rule, and never the
    /// tolerant locator's two-path `searched … or …` search. A variant that
    /// delegates to [`find_fixture_cdylib`] compiles and passes every other
    /// test — this is the arm that fails it. Same deliberate-panic note as the
    /// test above.
    #[test]
    fn a_missing_same_profile_fixture_refuses_naming_the_running_profile() {
        let dir = test_target_profile_dir();
        let err = std::panic::catch_unwind(|| {
            find_fixture_cdylib_same_profile("l23_no_such_fixture_crate");
        })
        .expect_err("a missing same-profile fixture must panic");
        let msg = err
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| err.downcast_ref::<&str>().map(|s| (*s).to_string()))
            .expect("panic payload is a string");
        assert!(
            msg.contains(&dir.display().to_string()),
            "panic must name the running profile dir {}; got: {msg}",
            dir.display()
        );
        assert!(
            msg.contains("does not accept a sibling-profile fixture"),
            "panic must state the no-sibling rule; got: {msg}"
        );
        assert!(
            msg.contains("cargo build -p l23_no_such_fixture_crate"),
            "panic must name the build command; got: {msg}"
        );
        assert!(
            !msg.contains("(searched "),
            "the same-profile locator must not have searched a sibling profile; got: {msg}"
        );
    }

    /// [`count_at_exclusively`] against hand-built capture vectors — the
    /// pairing it exists to make unforgettable, pinned on BOTH sides.
    ///
    /// Three of the eight arms are the three ways a hand-written
    /// pair can be got wrong, and each fails a DIFFERENT assertion here:
    /// dropping the level (arm (a) — a demoted head counts), dropping the
    /// level-free total (arm (b) — a duplicate at another level passes), and
    /// matching the level anywhere on the line instead of in its header (arm
    /// (d) — a field VALUE or a span name votes; that one is a defect in
    /// [`line_level`], not in this body). The rest pin the contract's edges:
    /// (c) the conjunction scope, (e) that a capture carrying the markers
    /// NOWHERE is `Ok(0)` rather than an error, (f) that an UNPARSEABLE header
    /// fails loudly in both directions — counted as ours, or dropped before
    /// the exclusivity check — which is the fail-closed claim the doc makes and
    /// the only one the other arms cannot see, (g) a count above 1, where
    /// returning the level-free total instead of the level count first shows,
    /// and (h) that an empty marker set is refused rather than read as "every
    /// line at this level".
    #[test]
    fn count_at_exclusively_pairs_the_level_count_with_the_level_free_total() {
        const MARKER: &str = "l735 oracle marker";
        let warn = format!("2026-09-03T00:00:00.0Z  WARN probe: cerulion: {MARKER} topic=/a");
        let warn_other_topic =
            format!("2026-09-03T00:00:00.1Z  WARN probe: cerulion: {MARKER} topic=/b");
        let info_copy = format!("2026-09-03T00:00:00.2Z  INFO probe: cerulion: {MARKER} topic=/a");
        let unrelated = "2026-09-03T00:00:00.3Z  WARN probe: cerulion: something else".to_string();

        // (a) The plain case: one WARN, nothing else carrying the marker.
        let lines: Vec<&str> = vec![&warn, &unrelated];
        assert_eq!(
            count_at_exclusively(&lines, "WARN", &[MARKER]),
            Ok(1),
            "one WARN carrying the marker and no copy elsewhere"
        );
        assert_eq!(
            lines_at_exclusively(&lines, "WARN", &[MARKER]),
            Ok(vec![warn.as_str()]),
            "the lines twin returns the matching lines themselves, in capture order \
             (three call sites index this Vec and read fields off the line)"
        );
        let wrong_level = count_at_exclusively(&lines, "INFO", &[MARKER])
            .expect_err("asking for a level the marker never uses must refuse, not read 0");
        assert!(
            wrong_level.contains("1 line(s) carry")
                && wrong_level.contains("0 at INFO, 1 elsewhere")
                && wrong_level.contains("[WARN]")
                && wrong_level.contains(MARKER),
            "the refusal must report the offender AND its level, not a bare 0; got: {wrong_level}"
        );

        // (b) THE regression: a duplicate of the same marker at INFO beside the
        // WARN. The level-token count alone still reads 1 — the whole point.
        let dup: Vec<&str> = vec![&warn, &info_copy];
        assert_eq!(
            count_at(&dup, "WARN", MARKER),
            1,
            "the level-token count alone cannot see the INFO copy"
        );
        let err = count_at_exclusively(&dup, "WARN", &[MARKER])
            .expect_err("a copy at another level must refuse");
        assert!(
            err.contains("1 at WARN, 1 elsewhere") && err.contains("[INFO]"),
            "the refusal must name both counts and show the offending line WITH its level; \
             got: {err}"
        );

        // (c) The markers are a CONJUNCTION, and the exclusivity is over that
        // same conjunction: the INFO copy names topic=/a, so counting the
        // topic=/b WARN is unaffected by it.
        let mixed: Vec<&str> = vec![&warn, &warn_other_topic, &info_copy];
        assert_eq!(
            count_at_exclusively(&mixed, "WARN", &[MARKER, "topic=/b"]),
            Ok(1),
            "a conjunction is exclusive over the conjunction, not over its first term"
        );
        assert!(
            count_at_exclusively(&mixed, "WARN", &[MARKER, "topic=/a"]).is_err(),
            "the topic=/a conjunction DOES have a copy at INFO"
        );

        // (d) The level is read from the line HEADER, never from its body: a
        // message whose FIELD VALUE is the token must not vote (a
        // `level=WARN` field is the obvious shape).
        let body_token =
            format!("2026-09-03T00:00:00.4Z  INFO probe: cerulion: {MARKER} level=WARN");
        let bodies: Vec<&str> = vec![&body_token];
        let field_vote = count_at_exclusively(&bodies, "WARN", &[MARKER])
            .expect_err("a `level=WARN` FIELD must not make an INFO line count as WARN");
        assert!(
            field_vote.contains("0 at WARN, 1 elsewhere") && field_vote.contains("[INFO]"),
            "the line is INFO, whatever its fields say; got: {field_vote}"
        );

        // (e) No line carries the marker at all: 0, not an error — an absence
        // guard reads the same as it always did.
        let empty: Vec<&str> = vec![&unrelated];
        assert_eq!(count_at_exclusively(&empty, "WARN", &[MARKER]), Ok(0));

        // (f) A line whose header carries NO level token fails LOUDLY — the
        // fail-closed direction the doc promises, and the one the other arms
        // cannot see. Two shapes, because the two ways to get it wrong differ:
        // counting it as ours (fail-open) and dropping it before the
        // exclusivity check (silently ignored) both read `Ok(1)` here.
        let headerless = format!("{MARKER} topic=/a");
        let beside: Vec<&str> = vec![&warn, &headerless];
        assert!(
            count_at_exclusively(&beside, "WARN", &[MARKER]).is_err(),
            "an unparseable header must fail loudly, not vote and not vanish"
        );
        let alone: Vec<&str> = vec![&headerless];
        assert!(
            count_at_exclusively(&alone, "WARN", &[MARKER]).is_err(),
            "a marker whose ONLY line has no parseable level is not a WARN"
        );

        // (g) A NON-TRIVIAL count: at n == 1 a body returning the level-free
        // total instead of the level count reads the same, so the two can only
        // be told apart above 1.
        let two: Vec<&str> = vec![&warn, &warn_other_topic, &unrelated];
        assert_eq!(count_at_exclusively(&two, "WARN", &[MARKER]), Ok(2));

        // (h) An empty marker set is refused, never read as "every line".
        assert!(count_at_exclusively(&two, "WARN", &[]).is_err());
    }

    /// The gate helpers checked against what tracing ACTUALLY compiled in — by
    /// emitting real events and counting them, never by re-deriving the
    /// predicate. In a debug build every arm reads 1, so a variant that always returns
    /// `0` fails here; in a release build (the `--lib --release` step of the latency job,
    /// the one place this class ever reddens) the DEBUG and TRACE arms read 0, so a
    /// variant that always returns `n` — byte for byte the behaviour that reddens a
    /// release run — fails here too. The unconditional INFO arm is the
    /// anti-tautology half: a dead capture would satisfy every other line.
    ///
    /// Every arm counts its probe AT THE PROBE'S DECLARED LEVEL. Counting the
    /// message text alone would let the DEBUG probe be re-emitted at `trace!`
    /// (still one line; `never_loud` permits TRACE) and the TRACE probe at
    /// `debug!` — each swap leaving the helper it pins unexercised in exactly
    /// the profile where both levels are compiled in.
    #[test]
    #[tracing_test::traced_test]
    fn the_gate_helpers_agree_with_what_tracing_compiled_in() {
        // The emits are plain literals (the walk cross-references a marker
        // against the LITERAL message of each `debug!`/`trace!`); the consts
        // must carry the same text, and the INFO arm below fails if they drift.
        const INFO_PROBE: &str = "gate probe at info";
        const DEBUG_PROBE: &str = "gate probe at debug";
        const TRACE_PROBE: &str = "gate probe at trace";
        tracing::info!("gate probe at info");
        tracing::debug!("gate probe at debug");
        tracing::trace!("gate probe at trace");
        logs_assert(|lines: &[&str]| {
            // Each probe is counted AT ITS DECLARED LEVEL, never by message
            // text: a text-only count reads 1 for a DEBUG probe re-emitted at
            // `trace!` (and `never_loud` permits TRACE), so the level this test
            // exists to pin would go unpinned. `count_at_exclusively` also
            // fails a duplicate of the same probe at any other level.
            let info_probes = count_at_exclusively(lines, "INFO", &[INFO_PROBE])?;
            if info_probes != 1 {
                return Err(format!(
                    "the capture is dead or doubled: {info_probes} INFO probe line(s) at INFO, \
                     want 1; {lines:?}"
                ));
            }
            // A quiet probe must never surface loudly — the level-free twin.
            never_loud(lines, DEBUG_PROBE)?;
            never_loud(lines, TRACE_PROBE)?;
            let debugs = count_at_exclusively(lines, "DEBUG", &[DEBUG_PROBE])?;
            if debugs != debug_lines_expected(1) {
                return Err(format!(
                    "debug_lines_expected(1) = {} but tracing emitted {debugs} DEBUG probe \
                     line(s) at DEBUG: the helper disagrees with the static gate it claims to \
                     mirror",
                    debug_lines_expected(1)
                ));
            }
            let traces = count_at_exclusively(lines, "TRACE", &[TRACE_PROBE])?;
            let want_traces = usize::from(trace_level_compiled_in());
            if traces != want_traces {
                return Err(format!(
                    "trace_level_compiled_in() = {} but tracing emitted {traces} TRACE probe \
                     line(s) at TRACE",
                    trace_level_compiled_in()
                ));
            }
            if debug_lines_expected(0) != 0 {
                return Err("debug_lines_expected(0) must be 0 in every profile".to_string());
            }
            Ok(())
        });
    }
}
