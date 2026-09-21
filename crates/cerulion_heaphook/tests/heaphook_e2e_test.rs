// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(all(target_os = "linux", target_env = "gnu"))]
//! Subprocess e2e: `libcerulion_heaphook.so` `LD_PRELOAD`ed onto
//! the standalone `heaphook_probe` fixture.
//!
//! # Why a preloaded fixture, not a self-re-exec
//!
//! An integration test that links the `cerulion_heaphook` rlib would statically
//! pull in the `#[no_mangle]` `malloc` interposers AND the `cerulion_heaphook_abi`
//! symbol, so a re-exec of THIS binary could never be a clean "no preload"
//! control and `dlsym` would find the symbol without a preload. The
//! `heaphook_probe` fixture DELIBERATELY does not depend on the crate, so the
//! interposers reach it ONLY through the preload — a genuine LD_PRELOAD test.
//!
//! # What runs where
//!
//! These run on Linux/GNU only (`cfg`-gated; empty elsewhere), under the repo's
//! normal `cargo test -p cerulion_heaphook`, so CI's Linux jobs execute them.
//! They need prebuilt artifacts: `libcerulion_heaphook.so` (`cargo build -p
//! cerulion_heaphook` — `cargo test --test heaphook_e2e_test` links the RLIB
//! into the test binary but does NOT rebuild the CDYLIB artifact, so a stale
//! `.so` would pass these tests unnoticed without the
//! freshness guard) and the `heaphook_probe` binary (`cargo build -p heaphook_probe`). A
//! missing artifact fails loudly with the exact build command, and EVERY
//! preloaded/spawned artifact — the hook's own `.so` included — is
//! mtime-guarded against its source tree (never a false pass). The
//! pure window/registry/recursion/classify/abi LOGIC is unit-tested on every
//! platform in the crate's `#[cfg]`-free modules.

use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant, SystemTime};

use cerulion_heaphook::abi::HEAPHOOK_ABI_VERSION;

const CHILD_TIMEOUT: Duration = Duration::from_secs(30);
/// Ceiling on draining a probe's stdout/stderr AFTER it exited:
/// the probes FORK, and a wedged grandchild inheriting the pipe write ends
/// keeps `read_to_string` from ever seeing EOF.
const DRAIN_TIMEOUT: Duration = Duration::from_secs(10);
const DONE_MARKER: &str = "PROBE_DONE";

/// The running test binary's profile directory (`<target>[/<triple>]/<profile>`).
fn profile_dir() -> PathBuf {
    let exe = std::env::current_exe().expect("current_exe");
    // <profile>/deps/<test-bin> → <profile>
    exe.parent()
        .and_then(|p| p.parent())
        .expect("profile dir")
        .to_path_buf()
}

/// Locate an artifact by file name in THIS test binary's own profile dir.
///
/// Deliberately NO debug⇄release sibling fallback: picking an artifact from the
/// OTHER profile would silently run a stale binary against the current test (the
/// repo's stale-binary phantom-pass class). A missing artifact panics LOUDLY
/// with the build recipe naming the profile it must be built under.
fn locate(file: &str, build_hint: &str) -> PathBuf {
    let dir = profile_dir();
    let primary = dir.join(file);
    if primary.exists() {
        return primary;
    }
    panic!(
        "artifact `{file}` not found at {} — build it under THIS profile first: {build_hint}",
        primary.display()
    );
}

/// The newest mtime among the `.rs` files under `dir`, RECURSIVELY — a subdir
/// added to a guarded `src/` tree later must not silently fall out of the sweep
/// (a one-level walk would skip it without a trace, the same silent-weakening
/// class one level down). `Err` — never a silent skip — if the tree is
/// unreadable, holds no `.rs` files at all, or any entry's metadata cannot be
/// read: an untimed source cannot be proven older than the binary,
/// so the staleness check must FAIL loudly rather than pass
/// vacuously (the stale-binary phantom-pass class this whole helper exists to
/// close).
fn newest_rs_mtime(dir: &Path) -> Result<SystemTime, String> {
    newest_rs_mtime_walk(dir)?
        .ok_or_else(|| format!("no `.rs` sources found under {}", dir.display()))
}

/// The recursive walk half: `Ok(None)` = readable but no `.rs` files (fine for
/// a subdir; the TOP level turns it into the loud no-sources error above).
fn newest_rs_mtime_walk(dir: &Path) -> Result<Option<SystemTime>, String> {
    let read =
        std::fs::read_dir(dir).map_err(|e| format!("read_dir({}) failed: {e}", dir.display()))?;
    let mut newest: Option<SystemTime> = None;
    for entry in read {
        let entry = entry.map_err(|e| format!("dir entry in {} unreadable: {e}", dir.display()))?;
        let path = entry.path();
        let meta = entry
            .metadata()
            .map_err(|e| format!("metadata of {} unreadable: {e}", path.display()))?;
        let m = if meta.is_dir() {
            match newest_rs_mtime_walk(&path)? {
                Some(m) => m,
                None => continue,
            }
        } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
            meta.modified()
                .map_err(|e| format!("mtime of {} unreadable: {e}", path.display()))?
        } else {
            continue;
        };
        newest = Some(newest.map_or(m, |cur| cur.max(m)));
    }
    Ok(newest)
}

/// An artifact `cargo test --test heaphook_e2e_test` does not itself rebuild —
/// the two fixture crates AND the hook's own cdylib (the test links the RLIB;
/// the `.so` is a separate product) — can be STALE: an old binary left from
/// before a source edit, the repo's stale-binary phantom-pass class. Panic
/// loudly if any `.rs` under the crate's `src/` is newer than the built
/// artifact. (On CI the `cargo build --workspace` step rebuilds everything
/// first, so this never fires there; it catches a local dev who forgot to
/// rebuild.)
///
/// The artifact also depends on the crate MANIFEST — `Cargo.toml` fixes the
/// crate-type, dependencies and feature set — and on a `build.rs` if one exists.
/// Editing the manifest without touching a `.rs` would otherwise leave a stale
/// `.so`/binary accepted, so both are folded into the
/// newest-input mtime under the same fail-closed discipline.
///
/// Every input that would make the comparison UNPROVABLE — an unreadable binary
/// mtime, an unreadable/empty/unreadable-metadata source tree, an unreadable
/// manifest — panics rather than skips: skipping would let a leftover artifact
/// run as "fresh", the exact phantom-pass this guards against.
fn assert_fresh(bin: &Path, crate_dir: &str, build_hint: &str) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .join(crate_dir);
    let bin_m = bin
        .metadata()
        .and_then(|m| m.modified())
        .unwrap_or_else(|e| panic!("cannot read mtime of built artifact {}: {e}", bin.display()));

    // Newest of: every `.rs` under `src/`, `Cargo.toml`, and `build.rs` if present.
    let mut newest = newest_rs_mtime(&root.join("src")).unwrap_or_else(|e| {
        panic!(
            "cannot establish artifact `{}` freshness: {e}",
            bin.display()
        )
    });
    // Cargo.toml is REQUIRED — a crate without a manifest is a harness bug, not a
    // silent skip.
    let manifest = root.join("Cargo.toml");
    let manifest_m = manifest
        .metadata()
        .and_then(|m| m.modified())
        .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", manifest.display()));
    newest = newest.max(manifest_m);
    // build.rs is OPTIONAL — folded in only when it exists. `Path::exists()`
    // maps metadata ERRORS to false (the same fail-open class the directory
    // check already closes — an unreadable build.rs would silently skip), so stat it directly
    // and fail loudly on anything but a genuine NotFound.
    let build_rs = root.join("build.rs");
    match build_rs.metadata() {
        Ok(m) => {
            let build_m = m
                .modified()
                .unwrap_or_else(|e| panic!("cannot read mtime of {}: {e}", build_rs.display()));
            newest = newest.max(build_m);
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => panic!("cannot stat {}: {e}", build_rs.display()),
    }

    assert!(
        newest <= bin_m,
        "artifact `{}` is STALE — a source or manifest input under {} was edited after it \
         was built. Rebuild: {build_hint}",
        bin.display(),
        root.display()
    );
}

fn hook_so() -> PathBuf {
    // `cargo test --test heaphook_e2e_test` links the RLIB into the test binary
    // but does NOT rebuild the CDYLIB artifact, so a stale `.so` would pass
    // a test that should fail. Same
    // fail-closed freshness discipline as the fixtures, one artifact over.
    let hint = "cargo build -p cerulion_heaphook";
    let p = locate("libcerulion_heaphook.so", hint);
    assert_fresh(&p, "cerulion_heaphook", hint);
    p
}

fn probe_bin() -> PathBuf {
    let hint = "cargo build -p heaphook_probe";
    let p = locate("heaphook_probe", hint);
    assert_fresh(&p, "test_fixtures/heaphook_probe", hint);
    p
}

fn shim_so() -> PathBuf {
    let hint = "cargo build -p heaphook_shim";
    let p = locate("libheaphook_shim.so", hint);
    assert_fresh(&p, "test_fixtures/heaphook_shim", hint);
    p
}

/// A finished probe run.
struct Run {
    stdout: String,
    stderr: String,
    ok: bool,
}

impl Run {
    /// The value of a `key=value` fact line the probe printed.
    fn fact(&self, key: &str) -> Option<&str> {
        self.stdout.lines().find_map(|l| {
            let (k, v) = l.split_once('=')?;
            (k == key).then_some(v)
        })
    }

    /// Assert a fact equals `want`.
    fn assert_fact(&self, key: &str, want: &str) {
        let got = self.fact(key);
        assert_eq!(
            got,
            Some(want),
            "fact {key}={want} expected; got {got:?}\n--- stdout ---\n{}\n--- stderr ---\n{}",
            self.stdout,
            self.stderr
        );
    }
}

/// Run the probe. `preload` sets `LD_PRELOAD` to the hook; `extra_env` adds
/// pairs. Bounded wait; stdout/stderr drained on helper threads (a full pipe
/// can never wedge).
fn run_probe(probe: &str, preload: bool, extra_env: &[(&str, &str)]) -> Run {
    run_probe_ld(
        probe,
        preload.then(|| hook_so().display().to_string()),
        extra_env,
    )
}

/// Run the probe with an explicit `LD_PRELOAD` string (or none).
fn run_probe_ld(probe: &str, ld_preload: Option<String>, extra_env: &[(&str, &str)]) -> Run {
    let mut cmd = Command::new(probe_bin());
    cmd.env("CER_HEAPHOOK_PROBE", probe)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    match ld_preload {
        Some(v) => {
            cmd.env("LD_PRELOAD", v);
        }
        None => {
            cmd.env_remove("LD_PRELOAD");
        }
    }
    for (k, v) in extra_env {
        cmd.env(k, v);
    }
    // Put the probe in its OWN process group: the probes FORK,
    // and on a child/drain timeout we must SIGKILL the whole group — grandchild
    // included — so a forked descendant holding the pipe cannot leak past a
    // failure. `process_group(0)` makes the child a group leader (pgid == pid).
    use std::os::unix::process::CommandExt;
    cmd.process_group(0);
    let mut child = cmd.spawn().expect("spawn probe");
    let pgid = child.id() as libc::pid_t; // == the child's pid == its group id
    let mut out = child.stdout.take().expect("stdout");
    let mut err = child.stderr.take().expect("stderr");
    let (out_tx, out_rx) = std::sync::mpsc::channel();
    let out_t = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = out.read_to_string(&mut s);
        let _ = out_tx.send(s);
    });
    let (err_tx, err_rx) = std::sync::mpsc::channel();
    let err_t = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = err.read_to_string(&mut s);
        let _ = err_tx.send(s);
    });

    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(s) => break s,
            None if Instant::now() > deadline => {
                // SIGKILL the whole group (grandchildren included), then reap +
                // join the drains (the closed pipe now EOFs them) before panic.
                kill_group(pgid);
                let _ = child.wait();
                let _ = out_t.join();
                let _ = err_t.join();
                panic!("probe `{probe}` did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    };
    // BOUNDED drain: the probes FORK, and a wedged grandchild
    // inherits the pipe write ends — `read_to_string` then never sees EOF even
    // after the probe itself was reaped, and an unbounded `join()` would hang
    // this binary until the CI cancel. `recv_timeout` bounds it; on a wedge we
    // SIGKILL the whole process group (the grandchild holding
    // the pipe), which EOFs and lets us JOIN the drain threads before panicking
    // so repeated failures cannot leak processes or threads.
    let stdout = match out_rx.recv_timeout(DRAIN_TIMEOUT) {
        Ok(s) => s,
        Err(_) => {
            kill_group(pgid);
            let _ = out_t.join();
            let _ = err_t.join();
            panic!(
                "probe `{probe}` stdout drain wedged — a forked grandchild held the pipe (group killed)"
            );
        }
    };
    let stderr = match err_rx.recv_timeout(DRAIN_TIMEOUT) {
        Ok(s) => s,
        Err(_) => {
            kill_group(pgid);
            let _ = err_t.join();
            panic!(
                "probe `{probe}` stderr drain wedged — a forked grandchild held the pipe (group killed)"
            );
        }
    };
    let _ = out_t.join();
    let _ = err_t.join();
    Run {
        stdout,
        stderr,
        ok: status.success(),
    }
}

/// SIGKILL an entire process group by its leader pid (`killpg`), reaping any
/// forked grandchildren a probe left behind. Best-effort: an already-reaped
/// group returns ESRCH, which is fine.
fn kill_group(pgid: libc::pid_t) {
    // SAFETY: killpg with SIGKILL on our own child's group; no memory touched.
    unsafe {
        libc::killpg(pgid, libc::SIGKILL);
    }
}

/// Every probe prints the DONE marker and exits 0 on the happy path.
fn assert_completed(run: &Run, probe: &str) {
    assert!(
        run.stdout.contains(DONE_MARKER),
        "probe `{probe}` did not complete\n--- stdout ---\n{}\n--- stderr ---\n{}",
        run.stdout,
        run.stderr
    );
    assert!(
        run.ok,
        "probe `{probe}` exited nonzero\nstderr:\n{}",
        run.stderr
    );
}

// ── the arms ────────────────────────────────────────────────────────────────

#[test]
fn handshake_present_reports_version_and_won_malloc() {
    let run = run_probe("handshake", true, &[]);
    assert_completed(&run, "handshake");
    run.assert_fact("present", "1");
    run.assert_fact("version", &HEAPHOOK_ABI_VERSION.to_string());
    run.assert_fact("status_won", "1");
}

/// The fixture is clean: with NO preload the handshake symbol is genuinely
/// absent (proving the probe does not statically link the interposers, which is
/// the whole reason it is a separate crate).
#[test]
fn handshake_absent_without_preload() {
    let run = run_probe("handshake", false, &[]);
    assert_completed(&run, "handshake");
    run.assert_fact("present", "0");
}

/// A SECOND malloc interposer preloaded AHEAD of the hook must make the hook
/// report `status_won == 0` (the foreign-allocator degrade). This pins that the
/// won-malloc detection is by MODULE IDENTITY (`dladdr`), not an address compare
/// that would be preempted to the foreign allocator and report a false "won".
#[test]
fn handshake_foreign_allocator_reports_not_won() {
    let ld = format!("{}:{}", shim_so().display(), hook_so().display());
    let run = run_probe_ld("handshake", Some(ld), &[]);
    assert_completed(&run, "handshake");
    run.assert_fact("present", "1"); // the hook's symbol is still there
    run.assert_fact("status_won", "0"); // but it did NOT win malloc
}

#[test]
fn a_malloc_under_an_armed_window_lands_in_the_slot() {
    let run = run_probe("window", true, &[]);
    assert_completed(&run, "window");
    run.assert_fact("in_slot", "1");
    run.assert_fact("range_test", "1");
    run.assert_fact("escape", "0");
    run.assert_fact("disarm_rc", "0");
}

#[test]
fn a_bump_past_the_tail_escapes_and_spills_to_the_real_heap() {
    let run = run_probe("escape", true, &[]);
    assert_completed(&run, "escape");
    run.assert_fact("in_slot", "0"); // spilled to real heap
    run.assert_fact("escape", "1"); // GrowthPastTail code
    run.assert_fact("disarm_rc", "1"); // disarm returns the latched escape code
}

#[test]
fn a_free_of_a_registered_range_routes_to_the_release_callback() {
    let run = run_probe("segment", true, &[]);
    assert_completed(&run, "segment");
    run.assert_fact("release_calls", "1"); // routed to release, NOT glibc free
    run.assert_fact("released_ptr_match", "1"); // the exact interior pointer
    run.assert_fact("released_cookie_match", "1"); // the registered cookie
    run.assert_fact("segment_reregistered_ok", "1"); // the free actually unregistered it
}

/// With the window closed, a malloc-heavy loop is byte-identical with and
/// without the hook (the value checksum, not addresses).
#[test]
fn a_window_closed_loop_is_byte_identical_with_and_without_the_hook() {
    let with = run_probe("loop", true, &[]);
    assert_completed(&with, "loop");
    // The preload actually LOADED: ld.so treats a failed
    // LD_PRELOAD as a warning and continues, so without this check the
    // comparison below is trivially vacuous (both runs unhooked).
    with.assert_fact("hook_present", "1");
    let without = run_probe("loop", false, &[]);
    assert_completed(&without, "loop");
    without.assert_fact("hook_present", "0");
    let a = with.fact("checksum").expect("checksum with hook");
    let b = without.fact("checksum").expect("checksum without hook");
    assert_eq!(
        a, b,
        "the hook perturbed a window-closed loop: with={a} without={b}"
    );
}

#[test]
fn atfork_folds_the_registry_and_clears_it_in_the_child() {
    let run = run_probe("atfork", true, &[]);
    assert_completed(&run, "atfork");
    run.assert_fact("atfork_parent_register_ok", "1"); // anti-vacuity
    run.assert_fact("atfork_child_cleared", "1");
}

/// A window armed ACROSS a fork: the slot is quarantined from arm time, so the
/// child freeing an in-slot pointer is a no-op, not a glibc free of SHM. The
/// `survived` fact discriminates the atfork child-TLS clear (a fresh child
/// allocation must land on the real heap, NOT the inherited window), and
/// `p_in_window` is the anti-vacuity guard (the bumped pointer really was
/// in-slot, else the arm proves nothing).
#[test]
fn an_armed_window_slot_survives_a_fork_and_the_child_tls_is_cleared() {
    let run = run_probe("armed_fork", true, &[]);
    assert_completed(&run, "armed_fork");
    run.assert_fact("armed_fork_p_in_window", "1"); // anti-vacuity
                                                    // The discriminating free rides an INTERIOR in-slot pointer —
                                                    // the first bump equals the slot base, itself a valid glibc pointer, so
                                                    // freeing only it could never abort even with quarantine routing broken.
    run.assert_fact("armed_fork_p2_interior", "1");
    run.assert_fact("armed_fork_survived", "1"); // no abort AND child TLS cleared
}

/// The aligned allocators bump into an armed window with correct alignment, and
/// `malloc_usable_size` of a window pointer is the EXACT ledger size.
#[test]
fn the_aligned_allocators_and_usable_size_work_on_a_window() {
    let run = run_probe("aligned", true, &[]);
    assert_completed(&run, "aligned");
    run.assert_fact("posix_memalign_in_slot_aligned", "1");
    run.assert_fact("aligned_alloc_in_slot_aligned", "1");
    run.assert_fact("memalign_in_slot_aligned", "1");
    run.assert_fact("window_usable_exact", "1");
}

/// `reallocarray` of a registered-segment pointer routes like `realloc`.
#[test]
fn reallocarray_of_a_segment_pointer_moves_and_releases() {
    let run = run_probe("reallocarray", true, &[]);
    assert_completed(&run, "reallocarray");
    run.assert_fact("reallocarray_non_null", "1"); // not a failing (NULL) realloc
    run.assert_fact("reallocarray_release_calls", "1");
    run.assert_fact("reallocarray_moved_off_segment", "1");
    // The full `nmemb * size` request reached the allocator (an implementation
    // ignoring `size` gets a minimum chunk far below 256) and the moved bytes
    // match the pattern-filled source.
    run.assert_fact("reallocarray_usable_ge_request", "1");
    run.assert_fact("reallocarray_data_ok", "1");
}

/// Many bumps grow the ledger `Vec` — each growth is a re-entrant real `malloc`
/// the recursion guard must route past our logic (without it, infinite
/// recursion). All bumps land in the slot; the process survives.
#[test]
fn the_recursion_guard_survives_a_ledger_growing_bump_storm() {
    let run = run_probe("guard_stress", true, &[]);
    assert_completed(&run, "guard_stress");
    let total = run.fact("stress_total").expect("stress_total");
    run.assert_fact("stress_in_slot", total);
    run.assert_fact("disarm_rc", "0");
}

/// An INCIDENTAL in-slot allocation freed AFTER disarm is a no-op (quarantine),
/// never glibc free of a shared-memory address — the process survives.
#[test]
fn a_freed_in_slot_pointer_after_disarm_is_a_noop_not_a_glibc_free() {
    let run = run_probe("quarantine", true, &[]);
    assert_completed(&run, "quarantine");
    run.assert_fact("in_slot", "1");
    run.assert_fact("retire_rc", "0"); // a quarantine extent was retired
    run.assert_fact("survived", "1"); // reached the end ⇒ no abort
}

/// A realloc of a pointer into a registered segment moves the bytes to the real
/// heap (copied under the lock), releases the sample once, and unregisters it.
#[test]
fn a_realloc_of_a_segment_pointer_moves_releases_and_unregisters() {
    let run = run_probe("segment_realloc", true, &[]);
    assert_completed(&run, "segment_realloc");
    run.assert_fact("realloc_data_ok", "1");
    run.assert_fact("release_calls", "1");
    run.assert_fact("released_ptr_match", "1");
    run.assert_fact("released_cookie_match", "1");
    run.assert_fact("moved_off_segment", "1");
}

/// DISCLOSURE regression: a realloc of a pointer into a
/// DISARMED slot fails SAFELY (null + ENOMEM, copies nothing) instead of copying
/// `tail_limit - addr` bytes and splicing a later allocation into the caller's
/// buffer. The probe arranges two in-window allocations A then B so a naive tail
/// copy WOULD span B (`span_covers_b`), disarms, then reallocs A — a null return
/// with copies-nothing is the disclosure proof, and A stays valid.
#[test]
fn a_realloc_of_a_disarmed_slot_pointer_fails_safe_and_discloses_nothing() {
    let run = run_probe("disarmed_realloc", true, &[]);
    assert_completed(&run, "disarmed_realloc");
    // The scenario is real: both allocations landed in-slot, B after A, and a
    // `tail - A` copy would have reached entirely through B.
    run.assert_fact("a_in_slot", "1");
    run.assert_fact("b_in_slot", "1");
    run.assert_fact("b_after_a", "1");
    run.assert_fact("span_covers_b", "1");
    // The fix: null result, no copy, ENOMEM set, old block untouched.
    run.assert_fact("realloc_null", "1");
    run.assert_fact("errno_enomem", "1");
    run.assert_fact("a_intact", "1");
}

/// A realloc of a window pointer escapes (Reallocated), moves off the slot, and
/// preserves the old bytes.
#[test]
fn a_realloc_of_a_window_pointer_escapes_and_preserves_data() {
    let run = run_probe("realloc_escape", true, &[]);
    assert_completed(&run, "realloc_escape");
    run.assert_fact("realloc_data_ok", "1");
    run.assert_fact("escape", "2"); // EscapeKind::Reallocated
    run.assert_fact("moved_off_slot", "1");
    run.assert_fact("disarm_rc", "2"); // disarm returns the latched escape code
}

/// `calloc` into an armed window zeroes the slot even when it arrived dirty.
#[test]
fn calloc_into_an_armed_window_zeroes_a_dirty_slot() {
    let run = run_probe("calloc_zero", true, &[]);
    assert_completed(&run, "calloc_zero");
    run.assert_fact("in_slot", "1");
    run.assert_fact("all_zero", "1");
}

/// A registered range freed with no callback set is leaked, and the leak is
/// counted (Principle #3 observability).
#[test]
fn a_release_with_no_callback_is_counted() {
    let run = run_probe("counter", true, &[]);
    assert_completed(&run, "counter");
    run.assert_fact("counter_increased", "1");
}

/// `CERULION_HEAPHOOK_DEBUG=1` makes the load breadcrumb reach the child's
/// stderr, naming the ABI version and the won-malloc bit.
#[test]
fn the_load_breadcrumb_names_the_abi_version_on_stderr() {
    let run = run_probe("handshake", true, &[("CERULION_HEAPHOOK_DEBUG", "1")]);
    assert_completed(&run, "handshake");
    let want = format!("cerulion heap hook loaded: abi v{HEAPHOOK_ABI_VERSION} won_malloc=1");
    assert!(
        run.stderr.contains(&want),
        "breadcrumb {want:?} not on stderr:\n{}",
        run.stderr
    );
}
