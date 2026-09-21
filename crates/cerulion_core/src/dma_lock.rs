// SPDX-License-Identifier: AGPL-3.0-only
//! CPU DMA / C-state latency lock — caps how deep the CPU may idle for
//! the duration of a latency-sensitive run.
//!
//! The Linux kernel's pm_qos subsystem accepts a "max acceptable
//! exit-from-idle latency" hint (µs) via `/dev/cpu_dma_latency`. The
//! value written into a fd opened on that device is the deepest C-state
//! exit latency the process will tolerate; the kernel then forbids any
//! C-state whose exit latency is **strictly greater** than it. The
//! constraint is reference-counted on open fds, so closing the fd drops
//! the request.
//!
//! - Value `0` → pin C0/POLL: the core never idles (max power, lowest
//!   wake latency). This is what [`cpu_dma_lock`] writes.
//! - A small value (e.g. `10`) → allow only shallow states such as C1
//!   (~1µs exit) and block deep ones like C2/C3 (tens-to-hundreds of µs
//!   exit + cache flush). The core can still idle for power while
//!   keeping caches warm and wake latency low. See
//!   [`cpu_dma_lock_with_max_us`].
//!
//! In practice this suppresses the multi-µs tail spikes that show up
//! at p99/p99.9 on listener-based benches — the spikes are the cost
//! of waking the CPU from a deep C-state.
//!
//! [`deepest_nonflushing_cap_us`] productizes the "small value" case:
//! it reads the per-CPU cpuidle table and derives the exact cap that
//! allows every shallow (non-cache-flushing) C-state while blocking the
//! first cache-flushing one — so the cap adapts to the hardware instead
//! of a hand-picked constant.
//!
//! On non-Linux targets this module compiles to a no-op
//! (`/dev/cpu_dma_latency` and the cpuidle sysfs tree do not exist
//! outside Linux).
//!
//! # Who calls this
//!
//! The `cerulion graph run` verb does, for the length of a live run. An
//! operator steers it from outside: `--no-cpu-dma-lock` turns the automatic
//! cap off, `CERULION_CPU_DMA_LOCK=1` forces the hard C0 pin and `=0`
//! disables the cap, and `CERULION_CPU_DMA_LOCK_US=<n>` sets an explicit cap
//! in microseconds (see "Environment variables" in `docs/user-api.md` and
//! `docs/deployment_tuning.md`). Node code never calls it: a node is a
//! library the run verb loads, and the lock belongs to the process that runs
//! the graph. The examples below show the call the run verb and the
//! framework's benches make.

use std::io;
use std::path::Path;

/// RAII handle for an active CPU DMA / C-state latency request.
///
/// While alive, the kernel forbids the CPU from entering any C-state
/// whose exit latency exceeds the value this lock was acquired with
/// (`0` µs = no idle at all, i.e. pinned to C0/POLL). Drop closes the
/// underlying fd, releasing the pm_qos request reference.
#[must_use = "the DMA latency lock is released when this value is dropped — bind it to a named local (e.g. `let _lock = ...`) for the desired scope"]
pub struct CpuDmaLock {
    #[cfg(target_os = "linux")]
    _file: std::fs::File,
}

/// Acquire the CPU DMA / C-state lock pinned to C0 (`0` µs).
///
/// Convenience wrapper for [`cpu_dma_lock_with_max_us`]`(0)` — the
/// hard "never idle" pin (max power, lowest wake latency). Preserved
/// for back-compat with existing callers and benches.
///
/// ```rust,no_run
/// // Bind the lock to a named local for the whole run: it is released
/// // when the value drops.
/// let _lock = cerulion_core::dma_lock::cpu_dma_lock()
///     .expect("cpu_dma_lock — verify /dev/cpu_dma_latency is writable");
/// ```
pub fn cpu_dma_lock() -> io::Result<CpuDmaLock> {
    cpu_dma_lock_with_max_us(0)
}

/// Acquire the CPU DMA / C-state lock at a specific max-acceptable
/// C-state exit latency (µs).
///
/// `max_exit_latency_us` is the deepest C-state exit latency the
/// process will tolerate. `0` pins C0/POLL (no idle, max power, lowest
/// wake latency — equivalent to [`cpu_dma_lock`]). A small value (e.g.
/// `10`) allows only shallow states such as C1 (~1µs exit) and blocks
/// deep ones like C2/C3 — the core can still idle for power while
/// keeping caches warm for low wake latency. The kernel blocks any
/// C-state whose exit latency is **strictly greater** than the value,
/// so to allow a state whose exit latency is `L` you pass `L` (or more).
///
/// On Linux, opens `/dev/cpu_dma_latency` for writing and writes
/// `max_exit_latency_us` as a native-endian `i32`. Requires write
/// access to that device (typically root or `CAP_SYS_NICE`). On
/// non-Linux targets this is a no-op that always succeeds — the device
/// does not exist, and the latency benefit is Linux-specific.
///
/// Returns an error if the device cannot be opened (`PermissionDenied`
/// without root) or if the kernel was built without pm_qos support
/// (`NotFound`, extremely rare on modern distros).
///
/// # Precondition
///
/// `max_exit_latency_us >= 0`. The kernel reads `/dev/cpu_dma_latency`
/// as a SIGNED `s32` µs cap whose only meaningful range is `>= 0`; a
/// negative value carries no budget meaning (it does not request an idle
/// "deeper than C0") and is rejected upstream by the CLI resolver. This
/// boundary `debug_assert!`s the `>= 0` invariant as well, so a negative
/// value passed in a debug build trips loudly at the kernel-write
/// boundary instead of writing a meaningless cap.
///
/// ```rust,no_run
/// // Allow only shallow C-states (C1): keep caches warm + low wake
/// // latency while still idling for power.
/// let _lock = cerulion_core::dma_lock::cpu_dma_lock_with_max_us(10)
///     .expect("cpu_dma_lock — verify /dev/cpu_dma_latency is writable");
/// ```
pub fn cpu_dma_lock_with_max_us(max_exit_latency_us: i32) -> io::Result<CpuDmaLock> {
    debug_assert!(
        max_exit_latency_us >= 0,
        "max_exit_latency_us must be >= 0 (got {max_exit_latency_us}); a negative C-state exit-latency cap is meaningless to pm_qos"
    );
    #[cfg(target_os = "linux")]
    {
        use std::fs::OpenOptions;
        use std::io::Write;
        let mut file = OpenOptions::new()
            .write(true)
            .open("/dev/cpu_dma_latency")?;
        file.write_all(&max_exit_latency_us.to_ne_bytes())?;
        Ok(CpuDmaLock { _file: file })
    }
    #[cfg(not(target_os = "linux"))]
    {
        // Suppress unused-variable on non-Linux (no device to write to).
        let _ = max_exit_latency_us;
        Ok(CpuDmaLock {})
    }
}

/// Read one CPU's cpuidle C-state table from `dir` (e.g.
/// `/sys/devices/system/cpu/cpu0/cpuidle`).
///
/// Returns `Some(states)` as `(name, exit_latency_us)` pairs in
/// directory-iteration order (the `derive_cap_from_states` formula is
/// order-independent), or `None` when the table cannot be trusted:
///
/// - `dir` absent or unreadable → `None`.
/// - any `state*` dir whose `latency` file is missing OR non-numeric →
///   `None` for the WHOLE table. This is the FAIL-SAFE: a
///   present-but-unreadable state must abort to the safe fallback
///   rather than be silently dropped, because dropping a (possibly
///   cache-flushing) state would LOOSEN the derived cap.
///
/// A `state*` dir whose `name` file is unreadable defaults to an empty
/// name (informational only — the formula keys off `latency`).
/// Non-`state` entries are skipped. An empty `dir` (no `state*`
/// entries) yields `Some(vec![])`, which `derive_cap_from_states` maps
/// to `None` (fewer than two positive states).
///
/// Pure filesystem I/O over a passed-in path, so it is hermetically
/// testable on every OS via a tempdir fixture.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn read_cpuidle_states(dir: &Path) -> Option<Vec<(String, i32)>> {
    use std::fs;
    // Absent/unreadable directory → cannot trust the table.
    let entries = fs::read_dir(dir).ok()?;
    let mut states: Vec<(String, i32)> = Vec::new();
    // `flatten()` drops unreadable dir *entries* (a rare filesystem
    // race on the entry itself, not a state we can see-but-not-parse).
    for entry in entries.flatten() {
        if !entry.file_name().to_string_lossy().starts_with("state") {
            continue;
        }
        let path = entry.path();
        // FAIL-SAFE: a present state whose `latency` is missing or
        // non-numeric aborts the WHOLE table (return None) — never
        // silently drop it, which could loosen the cap.
        let latency = fs::read_to_string(path.join("latency"))
            .ok()?
            .trim()
            .parse::<i32>()
            .ok()?;
        // The name is informational (the formula keys off latency, not
        // the name); default to empty if unreadable.
        let name = fs::read_to_string(path.join("name"))
            .map(|s| s.trim().to_string())
            .unwrap_or_default();
        states.push((name, latency));
    }
    Some(states)
}

/// Derive the deepest C-state exit-latency cap (µs) that still allows
/// every *shallow* (non-cache-flushing) C-state while blocking the
/// first cache-flushing one — read live from the CPUs' cpuidle tables.
///
/// On Linux, delegates to `deepest_cap_from_cpu_root` over the real
/// `/sys/devices/system/cpu` root: it enumerates EVERY `cpu<N>/cpuidle`
/// (hybrid P+E-core parts exist and the pm_qos value is
/// GLOBAL), reads each core's table via `read_cpuidle_states` (`latency`
/// = exit latency in µs, plus the informational `name`), applies the
/// flush-boundary formula (see `derive_cap_from_states`) per core, and
/// returns the **MIN** of the per-core caps — the most conservative
/// value, so the global cap blocks the shallowest cache-flushing state
/// present on ANY core. The per-core formula caps at `boundary - 1`,
/// where `boundary` is the exit latency at the start of the largest
/// *multiplicative* gap among the positive-latency states. POLL (latency
/// `0`) is busy-poll, never a flush boundary, and is always allowed.
///
/// Returns `None` when the cap cannot be determined:
/// - `/sys/devices/system/cpu` itself is unreadable (non-sysfs env);
/// - a core's cpuidle table is present but unreadable/malformed
///   (`read_cpuidle_states` → `None`) — a FAIL-SAFE abort, never a
///   silent loosening of the cap;
/// - no core yielded a determinable cap (no readable cpuidle table, or
///   fewer than two positive C-states on every core).
///
/// A core whose cpuidle dir is simply absent (offline / no driver) is
/// SKIPPED, not fatal. On non-Linux targets this is always `None`:
/// those platforms never auto-cap, so the cap silently no-ops there. In
/// every `None` case the caller falls back to its default cap (`graph run`
/// uses 10µs).
pub fn deepest_nonflushing_cap_us() -> Option<i32> {
    #[cfg(target_os = "linux")]
    {
        const CPU_ROOT: &str = "/sys/devices/system/cpu";
        deepest_cap_from_cpu_root(Path::new(CPU_ROOT))
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// Aggregate the per-core non-flushing caps under one sysfs CPU `root`
/// (e.g. `/sys/devices/system/cpu`) into the single GLOBAL pm_qos cap (µs),
/// or `None` when none can be determined.
///
/// Split out from [`deepest_nonflushing_cap_us`] (its sole non-test caller,
/// which passes the real `/sys` root) so the multi-core aggregation glue —
/// the `cpu<N>` digit-only name filter, the per-core `read_cpuidle_states` +
/// `derive_cap_from_states`, the absent-cpuidle-dir SKIP, the malformed-core
/// ABORT, and the cross-core MIN — is hermetically testable via a tempdir
/// fixture on every OS (mirrors `read_cpuidle_states`'s path-injected,
/// cfg-agnostic shape).
///
/// Behaviour (identical to the inlined logic it replaced):
/// - `root` unreadable → `None`.
/// - a `cpu<N>` dir whose `cpuidle` subdir is ABSENT → SKIPPED (offline /
///   no driver — not fatal).
/// - a `cpu<N>` dir whose `cpuidle` table is present but unreadable/malformed
///   (`read_cpuidle_states` → `None`) → FAIL-SAFE ABORT to `None` for the
///   whole probe (never loosen the cap from a bad read).
/// - a core with fewer than two positive C-states (`derive_cap_from_states`
///   → `None`) contributes no cap but is NOT fatal (see the skip-site
///   comment).
/// - non-`cpu<N>` siblings (`cpufreq`, `cpuidle`, `cpu_capacity`, a bare
///   `cpu`, …) are ignored.
/// - returns the MIN of the per-core caps (the GLOBAL pm_qos value must block
///   the shallowest cache-flushing state on ANY core), or `None` when no core
///   yielded a cap.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn deepest_cap_from_cpu_root(root: &Path) -> Option<i32> {
    use std::fs;
    // The pm_qos value is GLOBAL, so the cap must hold across every core —
    // unreadable cpu root → cannot determine a cap.
    let entries = match fs::read_dir(root) {
        Ok(e) => e,
        Err(_) => {
            tracing::debug!("cpuidle: CPU root unreadable — caller will use the fallback cap");
            return None;
        }
    };
    let mut caps: Vec<i32> = Vec::new();
    let mut cores_with_cap: usize = 0;
    let mut total_states: usize = 0;
    for entry in entries.flatten() {
        let file_name = entry.file_name();
        let name = file_name.to_string_lossy();
        // Match `cpu<N>` (digits only) — skip cpufreq / cpuidle /
        // cpu_capacity / a bare `cpu` / etc.
        let is_cpu_n = name
            .strip_prefix("cpu")
            .is_some_and(|rest| !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()));
        if !is_cpu_n {
            continue;
        }
        let cpuidle = entry.path().join("cpuidle");
        // A core without a cpuidle dir (offline / no driver) is skipped —
        // absent is not fatal.
        if !cpuidle.is_dir() {
            continue;
        }
        // Present-but-unreadable/malformed table → FAIL-SAFE abort to the
        // safe fallback (never loosen the cap from a bad read).
        let states = match read_cpuidle_states(&cpuidle) {
            Some(s) => s,
            None => {
                tracing::debug!(
                    "cpuidle: a core's C-state table is present but unreadable/malformed — caller will use the fallback cap"
                );
                return None;
            }
        };
        total_states += states.len();
        // Per-core derive→None SKIP — ASYMMETRIC with the malformed-table
        // ABORT above. A core with fewer than two
        // positive-latency C-states has no derivable flush cliff. The
        // realistic single-positive case (POLL + a shallow C1 only) MUST stay
        // allowed — capping below it would wrongly pin C0 — so a no-cliff core
        // simply contributes no cap and we defer to the OTHER cores (or, if
        // none yields one, the caller's 10µs fallback) rather than aborting
        // the whole probe. ONLY an unreadable/malformed table is a hard abort.
        if let Some(cap) = derive_cap_from_states(&states) {
            caps.push(cap);
            cores_with_cap += 1;
        }
    }
    // The GLOBAL pm_qos value must block the shallowest cache-flushing
    // state present on ANY core → take the MIN.
    match caps.iter().min().copied() {
        Some(cap) => {
            tracing::debug!(
                cap_us = cap,
                cores = cores_with_cap,
                states = total_states,
                "cpuidle: derived deepest non-flushing C-state cap"
            );
            Some(cap)
        }
        None => {
            tracing::debug!(
                "cpuidle: no determinable cap (no readable cpuidle table, or fewer than two positive C-states on every core) — caller will use the fallback cap"
            );
            None
        }
    }
}

/// Pure flush-boundary formula over a cpuidle state table — no I/O, so
/// it is hermetically testable on every OS.
///
/// `states` is `(name, exit_latency_us)` for each cpuidle state, in any
/// order. The cap must ALLOW shallow non-flushing C-states (cheap, no
/// cache flush) but BLOCK the first cache-FLUSHING state. The kernel's
/// `/dev/cpu_dma_latency` forbids any C-state whose exit latency is
/// **strictly greater** than the cap, so returning `boundary - 1`
/// blocks the boundary state itself while permitting everything below.
///
/// Algorithm:
/// 1. POLL has latency `0` — it is busy-poll, never a flush boundary;
///    exclude it from the gap analysis (it is always allowed anyway).
/// 2. Keep states with `latency > 0`, sorted ascending by latency. If
///    fewer than two remain there is no determinable cliff → `None`
///    (the caller falls back to its default cap).
/// 3. Find the consecutive pair `(lo, hi)` maximizing the ratio
///    `hi / lo`. The flush boundary is `hi`; return `Some(hi - 1)`.
///
/// **Tie-break:** when two gaps share the maximum ratio, the
/// shallowest (lowest-latency, first in ascending order) gap wins —
/// this is the conservative choice (caps lower, blocks more deep
/// states). The walk uses a strict `>` to replace the best, so the
/// first maximal pair is retained.
///
/// Worked oracle (a measured cpuidle table):
/// `[(POLL,0),(C1,1),(C2,127),(C3,1048)]` → positives `{1,127,1048}`;
/// ratios `1→127 = 127×`, `127→1048 ≈ 8.3×` → biggest gap at `C2(127)`
/// → cap `126` (allows C1 exit-1, blocks C2 exit-127).
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn derive_cap_from_states(states: &[(String, i32)]) -> Option<i32> {
    let mut positives: Vec<i32> = states
        .iter()
        .filter_map(|(_name, latency)| (*latency > 0).then_some(*latency))
        .collect();
    if positives.len() < 2 {
        return None;
    }
    positives.sort_unstable();

    // Seed from the first (shallowest) gap; only replace on a strictly
    // larger ratio so ties resolve toward the shallowest boundary.
    let mut best_hi = positives[1];
    let mut best_ratio = positives[1] as f64 / positives[0] as f64;
    for pair in positives.windows(2).skip(1) {
        let ratio = pair[1] as f64 / pair[0] as f64;
        if ratio > best_ratio {
            best_ratio = ratio;
            best_hi = pair[1];
        }
    }
    Some(best_hi - 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn state(name: &str, latency: i32) -> (String, i32) {
        (name.to_string(), latency)
    }

    /// Write a fake `state*/{latency,name}` fixture under `dir`. `None`
    /// for either file leaves that file absent.
    fn write_state(dir: &Path, state_dir: &str, latency: Option<&str>, name: Option<&str>) {
        let sdir = dir.join(state_dir);
        std::fs::create_dir_all(&sdir).expect("create state dir");
        if let Some(l) = latency {
            std::fs::write(sdir.join("latency"), l).expect("write latency");
        }
        if let Some(n) = name {
            std::fs::write(sdir.join("name"), n).expect("write name");
        }
    }

    #[test]
    fn derive_cap_x86_cliff_returns_126() {
        // Bench-box layout: POLL(0), C1(1), C2(127), C3(1048).
        // Biggest multiplicative gap is 1→127 (127×) → boundary 127 →
        // cap 126 (allows C1 exit-1, blocks C2 exit-127).
        let states = [
            state("POLL", 0),
            state("C1", 1),
            state("C2", 127),
            state("C3", 1048),
        ];
        assert_eq!(derive_cap_from_states(&states), Some(126));
    }

    #[test]
    fn derive_cap_arm_jetson_layout_picks_largest_ratio_gap() {
        // ARM-ish names, three positive states: WFI(1), cpu-sleep(200),
        // cluster-sleep(3000). Ratios 1→200 = 200×, 200→3000 = 15× →
        // biggest gap at 200 → boundary 200 → cap 199.
        let states = [
            state("WFI", 1),
            state("cpu-sleep-0", 200),
            state("cluster-sleep-0", 3000),
        ];
        assert_eq!(derive_cap_from_states(&states), Some(199));
    }

    #[test]
    fn derive_cap_empty_is_none() {
        let states: [(String, i32); 0] = [];
        assert_eq!(derive_cap_from_states(&states), None);
    }

    #[test]
    fn derive_cap_single_positive_state_is_none() {
        // Only one positive-latency state (POLL is excluded): no cliff
        // to find → None → caller falls back to its default cap.
        let states = [state("POLL", 0), state("C1", 1)];
        assert_eq!(derive_cap_from_states(&states), None);
    }

    #[test]
    fn derive_cap_gradual_no_clear_cliff_tie_breaks_to_shallowest() {
        // a(10), b(20), c(40): ratios 10→20 = 2×, 20→40 = 2× (tie).
        // Tie-break is the shallowest gap (first in ascending order) →
        // boundary 20 → cap 19.
        let states = [state("a", 10), state("b", 20), state("c", 40)];
        assert_eq!(derive_cap_from_states(&states), Some(19));
    }

    #[test]
    fn cpu_dma_lock_does_not_panic() {
        // CI on Linux usually runs without root → PermissionDenied is
        // acceptable. macOS / Windows hit the no-op path. The only
        // portable assertion is "calling does not panic."
        let _ = cpu_dma_lock();
    }

    #[test]
    fn cpu_dma_lock_with_max_us_does_not_panic() {
        // Same contract as `cpu_dma_lock_does_not_panic`: PermissionDenied
        // on rootless Linux, no-op off Linux. A shallow-C-state cap (126µs,
        // the bench-box C1 ceiling) must not panic regardless.
        let _ = cpu_dma_lock_with_max_us(126);
    }

    #[test]
    fn deepest_nonflushing_cap_us_does_not_panic() {
        // Off Linux → None; on Linux → Some(cap) or None depending on
        // the cpuidle tree. Either way it must not panic.
        let _ = deepest_nonflushing_cap_us();
    }

    #[cfg(not(target_os = "linux"))]
    #[test]
    fn cpu_dma_lock_off_linux_is_a_successful_no_op() {
        let _ = cpu_dma_lock().expect("non-Linux helper is a no-op and must succeed");
    }

    #[cfg(not(target_os = "linux"))]
    #[test]
    fn deepest_nonflushing_cap_us_off_linux_is_none() {
        assert_eq!(deepest_nonflushing_cap_us(), None);
    }

    // --- read_cpuidle_states (Fix C) — run on every OS via tempdir ---

    #[test]
    fn read_cpuidle_states_reads_well_formed_table() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_state(tmp.path(), "state0", Some("0"), Some("POLL"));
        write_state(tmp.path(), "state1", Some("1"), Some("C1"));
        write_state(tmp.path(), "state2", Some("127"), Some("C2"));
        let mut states = read_cpuidle_states(tmp.path()).expect("well-formed table → Some");
        // Directory iteration order is unspecified; sort before asserting.
        states.sort_by_key(|(_, latency)| *latency);
        assert_eq!(
            states,
            vec![state("POLL", 0), state("C1", 1), state("C2", 127)],
        );
    }

    #[test]
    fn read_cpuidle_states_skips_non_state_entries() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_state(tmp.path(), "state0", Some("1"), Some("C1"));
        write_state(tmp.path(), "state1", Some("127"), Some("C2"));
        // A non-`state` sibling dir + file (e.g. cpuidle's driver/
        // governor entries) must be ignored, not parsed.
        std::fs::create_dir_all(tmp.path().join("driver")).expect("driver dir");
        std::fs::write(tmp.path().join("current_governor"), "menu").expect("governor file");
        let states = read_cpuidle_states(tmp.path()).expect("Some");
        assert_eq!(states.len(), 2);
    }

    #[test]
    fn read_cpuidle_states_non_numeric_latency_aborts_to_none() {
        // The KEY #6 pin: a present state with a garbage `latency` must
        // abort the WHOLE table (fail-safe), never silently drop the
        // state — dropping a possibly-flushing state loosens the cap.
        let tmp = tempfile::tempdir().expect("tempdir");
        write_state(tmp.path(), "state0", Some("1"), Some("C1"));
        write_state(tmp.path(), "state1", Some("not-a-number"), Some("C2"));
        assert_eq!(read_cpuidle_states(tmp.path()), None);
    }

    #[test]
    fn read_cpuidle_states_missing_latency_aborts_to_none() {
        // A present state with NO `latency` file is equally untrusted →
        // None for the whole table (fail-safe abort).
        let tmp = tempfile::tempdir().expect("tempdir");
        write_state(tmp.path(), "state0", Some("1"), Some("C1"));
        write_state(tmp.path(), "state1", None, Some("C2"));
        assert_eq!(read_cpuidle_states(tmp.path()), None);
    }

    #[test]
    fn read_cpuidle_states_missing_name_defaults_empty() {
        // `name` is informational; an unreadable name defaults to empty
        // (the formula keys off latency, so this must NOT abort).
        let tmp = tempfile::tempdir().expect("tempdir");
        write_state(tmp.path(), "state0", Some("42"), None);
        let states = read_cpuidle_states(tmp.path()).expect("Some");
        assert_eq!(states, vec![(String::new(), 42)]);
    }

    #[test]
    fn read_cpuidle_states_empty_dir_is_some_empty() {
        // No `state*` entries → Some(empty) (documented). The caller's
        // `derive_cap_from_states` then maps the empty table to None.
        let tmp = tempfile::tempdir().expect("tempdir");
        assert_eq!(read_cpuidle_states(tmp.path()), Some(vec![]));
    }

    #[test]
    fn read_cpuidle_states_absent_dir_is_none() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let missing = tmp.path().join("does-not-exist");
        assert_eq!(read_cpuidle_states(&missing), None);
    }

    // --- deepest_cap_from_cpu_root (Fix 1 — the multi-core aggregation
    // glue, hermetically tested over a tempdir CPU root on every OS) ---

    /// Write a full bench-box cpuidle table (POLL/C1/C2/C3 → cap 126) under
    /// `<root>/<cpu>/cpuidle`.
    fn write_x86_core(root: &Path, cpu: &str) {
        let cpuidle = root.join(cpu).join("cpuidle");
        write_state(&cpuidle, "state0", Some("0"), Some("POLL"));
        write_state(&cpuidle, "state1", Some("1"), Some("C1"));
        write_state(&cpuidle, "state2", Some("127"), Some("C2"));
        write_state(&cpuidle, "state3", Some("1048"), Some("C3"));
    }

    /// (a) Two well-formed cores with DIFFERENT cliffs (cpu0 → 126, cpu1 →
    /// 199): the GLOBAL pm_qos cap is the MIN. Pins that the cross-core MIN is
    /// load-bearing (a max/first-wins regression would return 199).
    #[test]
    fn deepest_cap_from_cpu_root_takes_min_across_cores() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_x86_core(tmp.path(), "cpu0"); // → 126
        let cpu1 = tmp.path().join("cpu1").join("cpuidle"); // ARM-ish → 199
        write_state(&cpu1, "state0", Some("1"), Some("WFI"));
        write_state(&cpu1, "state1", Some("200"), Some("cpu-sleep-0"));
        write_state(&cpu1, "state2", Some("3000"), Some("cluster-sleep-0"));
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), Some(126));
    }

    /// (b) A present-but-MALFORMED core (non-numeric `latency`) ABORTS the
    /// whole probe to `None` — even though cpu0 alone would yield 126. The key
    /// fail-safe pin: a bad read must never loosen the cap by being skipped.
    #[test]
    fn deepest_cap_from_cpu_root_malformed_core_aborts_to_none() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_x86_core(tmp.path(), "cpu0"); // would be 126 on its own
        let cpu1 = tmp.path().join("cpu1").join("cpuidle");
        write_state(&cpu1, "state0", Some("1"), Some("C1"));
        write_state(&cpu1, "state1", Some("not-a-number"), Some("C2"));
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), None);
    }

    /// (c) A core whose `cpuidle` subdir is ABSENT (offline / no driver) is
    /// SKIPPED, not fatal → cpu0's cap survives. Distinguishes "absent dir"
    /// (skip) from "present but malformed" (abort, test (b)).
    #[test]
    fn deepest_cap_from_cpu_root_absent_cpuidle_dir_skips_core() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_x86_core(tmp.path(), "cpu0"); // → 126
        std::fs::create_dir_all(tmp.path().join("cpu1")).expect("cpu1 dir"); // no cpuidle/
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), Some(126));
    }

    /// (d) Non-`cpu<N>` siblings (`cpufreq`, `cpuidle`, `cpu_capacity`, a bare
    /// `cpu`) are ignored — pins the digit-only name filter. Each sibling
    /// carries a cpuidle table with a SHALLOWER cliff (cap 49), so a filter
    /// that wrongly let one through would lower the MIN to 49 instead of 126.
    #[test]
    fn deepest_cap_from_cpu_root_ignores_non_cpu_n_siblings() {
        let tmp = tempfile::tempdir().expect("tempdir");
        write_x86_core(tmp.path(), "cpu0"); // → 126
        for sib in ["cpufreq", "cpuidle", "cpu_capacity", "cpu"] {
            let cpuidle = tmp.path().join(sib).join("cpuidle");
            write_state(&cpuidle, "state0", Some("1"), Some("C1"));
            write_state(&cpuidle, "state1", Some("50"), Some("Csib")); // cliff → cap 49
        }
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), Some(126));
    }

    /// (e) Every core has fewer than two positive C-states (POLL + a single
    /// shallow C1) → no derivable cliff on ANY core → `None` (caller falls
    /// back to its 10µs default). Exercises the per-core derive→None SKIP plus
    /// the terminal empty-`caps` arm.
    #[test]
    fn deepest_cap_from_cpu_root_all_cores_too_few_states_is_none() {
        let tmp = tempfile::tempdir().expect("tempdir");
        for (cpu, c1) in [("cpu0", "1"), ("cpu1", "2")] {
            let cpuidle = tmp.path().join(cpu).join("cpuidle");
            write_state(&cpuidle, "state0", Some("0"), Some("POLL"));
            write_state(&cpuidle, "state1", Some(c1), Some("C1"));
        }
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), None);
    }

    /// (f) An empty root (no `cpu<N>` dirs at all) → `None`.
    #[test]
    fn deepest_cap_from_cpu_root_empty_root_is_none() {
        let tmp = tempfile::tempdir().expect("tempdir");
        assert_eq!(deepest_cap_from_cpu_root(tmp.path()), None);
    }

    /// (f, cont.) An ABSENT root (read_dir fails) → `None` — the read_dir Err
    /// arm, distinct from the empty-but-present root above.
    #[test]
    fn deepest_cap_from_cpu_root_absent_root_is_none() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let missing = tmp.path().join("does-not-exist");
        assert_eq!(deepest_cap_from_cpu_root(&missing), None);
    }
}
