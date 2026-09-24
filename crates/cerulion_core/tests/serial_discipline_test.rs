// SPDX-License-Identifier: AGPL-3.0-only
//! CI hardening: **a test that reaches the process-global
//! iceoryx2 singleton must be `#[serial]` unless its whole package runs
//! `-- --test-threads=1`.**
//!
//! `TransportManager::get_or_init()` returns ONE manager per process, backed
//! by iceoryx2's shared-memory singleton on the default namespace. Two libtest
//! threads driving it concurrently interleave topic creation, port slots and
//! SHM state; the failure mode is not a clean panic but an intermittent one —
//! the phantom-pass class. `cerulion_core` and several other packages are run
//! with `-- --test-threads=1` in CI, which serialises them wholesale, but
//! `cerulion_cli`, `cerulion_cli_engine` and friends are NOT, and there is no
//! CI step that would notice a new singleton-touching test dropped into one of
//! them: it would pass locally, pass on most CI runs, and fail on some.
//!
//! The rule is deliberately narrow so it cannot produce false positives:
//!
//! * only files under a package's `tests/` directory are walked. A
//!   `#[cfg(test)] mod` inside `src/` is NOT covered, because production code
//!   in the same file legitimately calls `get_or_init()` while the unit tests
//!   beside it never touch it — measured: `bag_cmd.rs` (97 tests),
//!   `flashback_cmd.rs` (19) and `topic_cmd.rs` (3) all have the call in
//!   PRODUCTION code, and a file-wide rule would flag all three wrongly.
//!   Covering them needs a production-vs-test region split, which is a
//!   different analysis;
//! * only a DIRECT `TransportManager::get_or_init(` in the file's own
//!   comment-stripped code counts. A test that reaches the singleton
//!   INDIRECTLY through a library helper is invisible here —
//!   `cerulion_cli_engine/tests/topic_observer_iox2_test.rs` is exactly that
//!   shape (the observers call it inside the engine) and is already fully
//!   `#[serial]` by hand;
//! * a package every one of whose CI `cargo test -p` steps carries
//!   `-- --test-threads=1` is exempt wholesale — libtest is already
//!   serialising it.
//!
//! It passes on adoption today (two covered files, both already fully
//! `#[serial]`), so it is a pure regression guard with no cleanup attached.
//!
//! # The nextest fence (the second rule)
//!
//! `cerulion_core` no longer runs `-- --test-threads=1` on the lanes that run
//! its `tests/` files: it runs under `cargo nextest run`, which gives EACH TEST
//! ITS OWN PROCESS. That dissolves most of what `#[serial]` defended — the
//! cdylib `NODES` registry, `set_var`, the `#[traced_test]` global subscriber,
//! a counting allocator are all process globals, and a fresh process has fresh
//! ones — but it does NOT dissolve the hazard this file is named for. The
//! DEFAULT iceoryx2 namespace is a MACHINE resource. Two processes see the same
//! one, so `#[serial]` (a mutex nothing else shares) protects nothing there,
//! and `--test-threads=1` is not a flag nextest takes.
//!
//! So for a package CI runs under nextest the rule becomes: a singleton-touching
//! binary must be in the `default-namespace` fence in `.config/nextest.toml`.
//! That is checked here in both directions — every caller is fenced, and every
//! fence entry is declared in [`FENCE_INVENTORY`] with a reason — because a
//! thirty-term filter on one line is easy to get wrong by eye, and
//! a mis-scoped filter that silently un-fences a binary is the failure mode the
//! whole change had to avoid.
//!
//! A package can be subject to BOTH rules (`cerulion_core` runs under nextest
//! on the PR lanes and under `cargo test -- --test-threads=1` on the release
//! and nightly ones); the nextest rule is checked first and is not waived by
//! the `--test-threads=1` exemption, because the nextest lane is the one where
//! two singleton binaries can overlap.
//!
//! # What is still out of scope, and what that now costs
//!
//! `src/` `#[cfg(test)] mod tests` blocks remain outside BOTH rules, for the
//! reason in the list above: production code in the same file legitimately
//! calls the accessor, so a file-wide rule flags three modules wrongly.
//!
//! Under `--test-threads=1` those unit tests were protected wholesale by the
//! flag. Under nextest they are protected by process-per-test for every class
//! EXCEPT the shared namespace, and nothing gates that — so the exclusion costs
//! slightly more than under the flag. VERIFIED empty: `cerulion_core/src`
//! contains no singleton CREATOR call outside a doc comment and an error
//! string, and every transport-using unit test there mints an isolated root.
//! Closing it properly needs a production-vs-test region split, which is a
//! different analysis; the doctest walk below closes the neighbouring gap that
//! `src/` really does run.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

/// Shared with `config_deny_unknown_fields_test.rs`: ONE literal-aware
/// comment stripper, so the two walks cannot drift apart again.
mod common;
use common::{code_only, code_only_blank_literals};

/// The calls that reach the process-global manager. A test file naming ANY of
/// these is talking to the singleton directly.
///
/// `get_or_init` alone was watched, and it is not the only door: `init`,
/// `init_with_config`, `init_default` and `init_with_ix_config_json` all
/// install or return the SAME `INSTANCE` `OnceLock`, so a test using one of
/// them was invisible to this gate while racing every other singleton test in
/// its package.
///
/// `init_for_test` is deliberately ABSENT and that is the interesting
/// exclusion: it is documented "NOT stored in `INSTANCE` — fresh per-call",
/// which is exactly what makes the per-test-SHM-root pattern parallel-safe.
/// Watching it would flag the files that are already doing the right thing.
///
/// Kept complete by [`the_watched_accessor_set_matches_the_singletons_own_doors`],
/// which DERIVES the set from `transport/mod.rs` rather than trusting this
/// list, so a sixth spelling fails loudly instead of arriving unwatched.
const SINGLETON_CALLS: &[&str] = &[
    "TransportManager::get_or_init(",
    "TransportManager::init(",
    "TransportManager::init_with_config(",
    "TransportManager::init_default(",
    "TransportManager::init_with_ix_config_json(",
    "TransportManager::get(",
];

/// The accessor whose name appears in a diagnostic when several would do.
const SINGLETON_CALL: &str = SINGLETON_CALLS[0];

/// The accessors that can INSTALL a manager, i.e. hand a caller a live handle
/// on the DEFAULT namespace in a process where nothing has done so yet.
///
/// [`SINGLETON_CALLS`] minus `get`, and the omission is the whole point.
/// `TransportManager::get()` never creates: it returns the existing `INSTANCE`
/// or an `Err`. Under `cargo test` that is still a door worth watching, because
/// a SIBLING THREAD may have initialised the instance and `get()` then hands
/// this test a live default-namespace manager — so the `#[serial]` rule keeps
/// watching the full set.
///
/// Under NEXTEST it cannot be that door. Each test is its own process, so
/// `get()` succeeds only if THIS test already installed a manager — which takes
/// one of the creators below, and that call is what the fence rule sees. A file
/// whose only singleton contact is `get()` therefore reaches nothing, and
/// `supervisor_precreate_iox2_test` is exactly that shape: it asserts
/// `get().is_err()`, i.e. that the process-global instance is UNSET, which
/// process-per-test makes MORE reliable than it is today rather than less.
///
/// Watching the full set here instead would not be conservative, it would be
/// wrong in a costly direction: it would demand a fence entry — and so
/// permanent serialisation — for a test that provably touches no namespace.
const SINGLETON_CREATORS: &[&str] = &[
    "TransportManager::get_or_init(",
    "TransportManager::init(",
    "TransportManager::init_with_config(",
    "TransportManager::init_default(",
    "TransportManager::init_with_ix_config_json(",
];

#[test]
fn the_creator_set_is_the_watched_set_minus_the_read_only_accessor() {
    // Derived rather than restated: if a sixth door is added to
    // `SINGLETON_CALLS`, it lands in the creator set too unless it is
    // deliberately named here — the fail-closed direction.
    let watched: BTreeSet<&str> = SINGLETON_CALLS.iter().copied().collect();
    let creators: BTreeSet<&str> = SINGLETON_CREATORS.iter().copied().collect();
    let read_only: Vec<&&str> = watched.difference(&creators).collect();
    assert_eq!(
        read_only,
        vec![&"TransportManager::get("],
        "exactly ONE watched accessor may be treated as non-creating, and it must be `get`. \
         A new accessor defaults into the creator set; if it genuinely creates nothing, say so \
         here with the reason."
    );
    assert!(
        creators.is_subset(&watched),
        "every creator must also be watched by the `#[serial]` rule"
    );
}

/// The nextest configuration, relative to the repo root.
const FENCE_FILE: &str = ".config/nextest.toml";

/// The test group that serialises the DEFAULT-namespace binaries.
const FENCE_GROUP: &str = "default-namespace";

/// EVERY binary in the `default-namespace` fence, with WHY it is there.
///
/// This is the hand-audited half of the fence: [`fence_binaries`] reads what
/// the config actually says, and
/// [`the_fence_membership_equals_the_declared_inventory`] asserts the two are
/// the same set. A binary may not join the fence without a line here, and a
/// line here that the config does not carry fails just as loudly — the point
/// being that a mis-scoped filter which silently un-fences a singleton binary
/// is the one failure mode this whole change has to avoid, and a filter is not
/// something easy to verify by eye.
///
/// Membership was DERIVED (grep every singleton accessor across
/// `cerulion_core/tests`) and then hand-READ, because the grep is wrong in both
/// directions — see the exclusions recorded in `.config/nextest.toml`.
const FENCE_INVENTORY: &[(&str, &str)] = &[
    (
        "adaptive_sizing_iceoryx2_test",
        "default-namespace manager: probes pool sizing on the shared root",
    ),
    (
        "cli_e2e_graph_latency_test",
        "spawns the real `cerulion` binary, which runs its graph on the default \
         namespace; also a release-only latency gate",
    ),
    (
        "cross_thread_rtt_test",
        "default-namespace ports moved across threads; also a flatness gate",
    ),
    (
        "deliver_history_failure_iox2_test",
        "default-namespace manager: history fault injection",
    ),
    (
        "dylib_corrupt_info_test",
        "default-namespace manager for the IPC build arm",
    ),
    (
        "e2e_cli_test",
        "default-namespace observability e2e — the namespace IS the subject",
    ),
    (
        "fill_from_e2e_test",
        "default-namespace manager: zero-copy fill_from round trip",
    ),
    (
        "flat_latency_test",
        "default-namespace manager; also the primary flatness gate",
    ),
    (
        "graph_latency_test",
        "default-namespace manager; also the user-POV latency gate",
    ),
    (
        "graph_test",
        "default-namespace manager on its runtime-stepping arms",
    ),
    (
        "latency_threshold_test",
        "default-namespace manager; also the release latency regression gate",
    ),
    (
        "macro_cdylib_test",
        "default-namespace manager for the cdylib IPC arm",
    ),
    (
        "macro_graph_test",
        "default-namespace manager: macro nodes in a real GraphRuntime",
    ),
    (
        "network_test",
        "default-namespace manager on its transport-backed arms",
    ),
    (
        "non_trigger_hold_iox2_test",
        "mixed: mostly isolated roots, three arms on the default namespace",
    ),
    (
        "output_proxy_test",
        "mixed: shares the global manager with unique-topic isolation",
    ),
    (
        "overflow_iox2_test",
        "default-namespace manager: overflow/spill parity",
    ),
    (
        "publisher_pool_sizing_test",
        "default-namespace manager: publisher pool provisioning",
    ),
    (
        "service_test",
        "default-namespace manager: request/response layer e2e",
    ),
    (
        "shm_footprint_probe",
        "default-namespace manager; hardware-only (`#[ignore]`), fenced so a local \
         `-- --ignored` run is protected too",
    ),
    (
        "shm_guard_madvise_iox2_test",
        "default-namespace manager; hardware-only (`#[ignore]`), same reason",
    ),
    (
        "snapshot_view_iox2_test",
        "default-namespace manager with unique-topic isolation",
    ),
    (
        "step_zero_alloc_test",
        "default-namespace manager: executor zero-alloc probe",
    ),
    (
        "trace_ring_hook_zero_alloc_test",
        "default-namespace manager: trace-ring hook zero-alloc probe",
    ),
    (
        "transport_test",
        "the singleton IS the subject — it asserts on `get()` after `get_or_init()`",
    ),
    (
        "zero_alloc_test",
        "default-namespace manager: subscriber-path zero-alloc probe",
    ),
    (
        "zero_copy_ci_test",
        "default-namespace manager: structural zero-copy verification",
    ),
    (
        "zero_copy_hot_path_test",
        "default-namespace manager: publisher-path zero-alloc probe",
    ),
];

/// The CLOSED set of execution classes, each paired with the LANE that owns
/// running arms of that class.
///
/// A free-text reason is not a contract: any 21-character string satisfied the
/// old check, so an arm could be misclassified, or given no owner at all, and
/// still read as declared. The class is therefore validated against this table,
/// and the lane is not a separate field anybody could forget to fill in — it is
/// a PROPERTY OF THE CLASS, so naming the class names the owner by construction.
///
/// `nobody (owed)` and `nobody (on demand)` are deliberately spellable. An arm
/// no lane can run is a real state and hiding it would defeat the inventory;
/// what must not happen is that state arriving SILENTLY.
const IGNORE_CLASSES: &[(&str, &str)] = &[
    (
        "child",
        "every CI pass — a parent test in the same file re-execs the binary and reads this child's exit code",
    ),
    (
        "unix-by-hand",
        "any Unix host, run by hand with `-- --ignored`",
    ),
    (
        "linux-only",
        "a Linux host only (needs a Linux kernel interface or CPU pinning)",
    ),
    (
        "jetson-board",
        "nobody (owed) — needs a Jetson board CI does not have",
    ),
    (
        "two-machine",
        "nobody (owed) — needs a desk and a robot with the harness env set on each",
    ),
    (
        "toolchain",
        "nobody (on demand) — asserts rustc's UNSTABLE diagnostic rendering, so CI must never gate on it; re-blessed after toolchain bumps",
    ),
];

/// Every `#[ignore]`d test arm in `cerulion_core/tests`, with the CLASS that
/// keeps it out of the default run and the LANE that owns running it.
///
/// WHY THIS EXISTS. `#[ignore]` is invisible by construction: a green
/// `cargo test` prints `N ignored` and says nothing about whether anybody ever
/// runs those arms. No workflow anywhere in this repo passes `--ignored`,
/// which makes "run it by hand" a promise kept by memory. This
/// inventory does not schedule anything — it makes UNOWNED a state you have to
/// write down, so an arm nobody can name a lane for is visible in review rather
/// than discovered years later.
///
/// THE CLASSES ARE NOT INTERCHANGEABLE, and one of them is easy to
/// misread:
///
/// * `child` — a self-re-exec CHILD process entry point. These are `#[ignore]`d
///   so libtest does not run them STANDALONE, but they execute on every
///   ordinary CI pass, driven by a parent test in the same file that re-execs
///   the binary and reads the child's exit code. They are NOT dark, and six of
///   the nineteen arms here are this class. Counting them as unowned overstates
///   the gap.
/// * `unix-by-hand` — needs several real processes and a real `MAP_SHARED` page.
///   Runnable on any Unix host.
/// * `linux-only` — needs a Linux-only kernel interface (`/proc/self/smaps`) or
///   CPU pinning. Runnable on a Linux host only.
/// * `jetson-board` — needs a specific board's `/dev/shm` behaviour, which CI
///   does not have; recorded as owed rather than pretended.
/// * `two-machine` — needs two hosts and hand-set environment. Same.
/// * `toolchain` — asserts rustc's own unstable diagnostic rendering, so it is
///   deliberately on-demand and re-blessed after toolchain bumps. CI must not
///   gate on it.
///
/// Compared BOTH directions by the test below, like the two inventories above:
/// an arm added without an entry fails, and an entry naming an arm that no
/// longer exists fails.
const IGNORED_ARM_INVENTORY: &[(&str, &str, &str, &str)] = &[
    (
        "barrier_level_gate_subprocess_iox2_test.rs",
        "box_two_process_split_merges_to_oracle",
        "unix-by-hand",
        "two REAL processes over one MAP_SHARED barrier page; the in-process twin is the CI gate",
    ),
    (
        "barrier_level_gate_subprocess_iox2_test.rs",
        "box_stalled_peer_poisons_and_exits_nonzero",
        "unix-by-hand",
        "proves the barrier GATES level advance rather than merely counting — needs a real stalled peer process",
    ),
    (
        "barrier_test.rs",
        "box_harness::box_wide_cross_process_rendezvous",
        "unix-by-hand",
        "8 child processes + owner over real MAP_SHARED and a real CPU park",
    ),
    (
        "barrier_test.rs",
        "box_harness::box_park_vs_spin_latency_ab",
        "unix-by-hand",
        "print-only park-vs-spin latency A/B; a measurement, not an assertion",
    ),
    (
        "barrier_test.rs",
        "box_harness::box_parked_waiter_survives_peer_crash_and_owner_unlink",
        "unix-by-hand",
        "munmap/unlink-during-park safety — a parked waiter must not SIGSEGV when a peer is SIGKILLed",
    ),
    (
        "barrier_test.rs",
        "box_harness::box_parked_waiter_still_wakes_after_owner_unlink",
        "unix-by-hand",
        "the positive twin — unlink kills the NAME, not the mapping or the wake path",
    ),
    (
        "monitor_wait_park_iox2_test.rs",
        "box_same_core::box_same_core_parked_live_loop_rtt_holds_the_platform_bound",
        "linux-only",
        "pins the parked live loop and its driver to one CPU, so it needs real affinity control and a quiet machine",
    ),
    (
        "shm_guard_madvise_iox2_test.rs",
        "advise_sets_nh_flag_on_our_iox2_pool_vmas",
        "linux-only",
        "reads a real /proc/self/smaps for the nh VmFlag; the sibling CI arms in this file are deliberately NOT ignored",
    ),
    (
        "shm_footprint_probe.rs",
        "probe_static_pool_footprint",
        "jetson-board",
        "asserts lazy/demand-paged SHM residency against a specific board's /dev/shm",
    ),
    (
        "shm_footprint_probe.rs",
        "probe_multi_pool_fit",
        "jetson-board",
        "multi-pool over-RAM apparent fit on that same board",
    ),
    (
        "shm_footprint_probe.rs",
        "probe_latency_big_vs_small_tier",
        "jetson-board",
        "oversizing-is-latency-free on that same board",
    ),
    (
        "cross_machine_data_plane_harness.rs",
        "cross_machine_data_plane",
        "two-machine",
        "a standing cross-machine data-plane diagnostic, driven by env; never a gate",
    ),
    (
        "macro_compile_fail_test.rs",
        "compile_fail_rustc_diagnostic_tests",
        "toolchain",
        "asserts rustc's own diagnostic rendering for const-eval panics and type errors",
    ),
    (
        "credit_os_sync_independence_test.rs",
        "child_prints_credit_primitive_availability",
        "child",
        "self-re-exec entry point; the parent runs it with `--exact … --ignored` and reads the park decision it prints",
    ),
    (
        "cdylib_iox2_log_level_test.rs",
        "subprocess_child_cdylib_level_probe",
        "child",
        "self-re-exec entry point; the parent reads the level this cdylib's own iceoryx2-log static reports",
    ),
    (
        "cdylib_iox2_log_level_test.rs",
        "subprocess_child_raw_ffi_level_probe",
        "child",
        "self-re-exec entry point for the raw-FFI half of the same contract",
    ),
    (
        "cdylib_tracing_stopgap_test.rs",
        "subprocess_child_discard_probe",
        "child",
        "self-re-exec entry point whose STDERR is the parent's oracle — libtest captures its own",
    ),
    (
        "iox2_log_level_test.rs",
        "subprocess_child_iox2_probe",
        "child",
        "self-re-exec entry point; iceoryx2's own logger writes to the process stderr, so a child is the only way to read it",
    ),
    (
        "notify_shortfall_iox2_test.rs",
        "subprocess_child_holds_a_subscriber",
        "child",
        "self-re-exec entry point; the condition under test is a listener whose OWNING PROCESS died without deregistering, which needs a second process to kill",
    ),
    (
        "output_proxy_test.rs",
        "subprocess_child_holds_a_subscriber",
        "child",
        "self-re-exec entry point; the same killed-consumer condition, driven against a graph output topic",
    ),
    (
        "lat_probe_env_test.rs",
        "subprocess_probe_report",
        "child",
        "self-re-exec entry point reporting the probe's env-resolved settings to its parent",
    ),
    (
        "reg_channel_iox2_test.rs",
        "subprocess_child_pump_panic_storm_probe",
        "child",
        "self-re-exec entry point driving a panic storm the parent classifies",
    ),
];

/// Binaries that claim every runner slot because their verdict is a MEASURED
/// WALL — they need a quiet machine, not protection from one sibling.
///
/// A separate mechanism from [`FENCE_INVENTORY`] on purpose (see the config's
/// header): a mutex group would let the whole rest of the suite run alongside
/// and skew the measurement, which is precisely what these gates cannot have.
/// Several appear in BOTH lists; that is not a duplication to collapse.
///
/// Whether each is whole-file `#![cfg(not(debug_assertions))]` was CHECKED, not
/// assumed. A grep for the cfg ANYWHERE in the file would call
/// `cross_thread_rtt_test` and `flat_latency_test` release-only; both are
/// deliberately DEBUG-OK (their metric is a RATIO, which is build-mode
/// invariant) and gate every pull request. That makes those two the members
/// this fence matters most for — the release-only five run on push-to-main lanes
/// where little else is competing for the machine.
const TIMING_INVENTORY: &[(&str, &str)] = &[
    (
        "cli_e2e_graph_latency_test",
        "release-only full-journey latency e2e; also spawns the real binary",
    ),
    (
        "cross_thread_rtt_test",
        "cross-thread RTT flatness — DEBUG-OK, so it gates every PR, and a \
         loaded runner inflates the per-size floors it compares",
    ),
    (
        "flat_latency_test",
        "the primary one-way flatness gate — DEBUG-OK, so it gates every PR; \
         its own header records macOS VM stalls inflating a size's floor",
    ),
    (
        "graph_latency_test",
        "release-only user-POV latency gate (p50 plus flatness)",
    ),
    (
        "ingress_publish_raw_notify_iox2_test",
        "asserts a notify lands inside a 1 s bounded window — DEBUG-OK, so it \
         runs on every PR and a loaded runner can invert it",
    ),
    (
        "latency_threshold_test",
        "release-only latency regression gate — the ci.yml test-latency headline",
    ),
    (
        "period_decompose_bench_test",
        "release-only per-hop decomposition bench",
    ),
    (
        "streaming_latency_bench_test",
        "release-only spin-then-block bench",
    ),
];

/// Does this repo-relative path compile to its OWN test BINARY?
///
/// cargo builds a test target from a DEPTH-1 `tests/*.rs` only. Anything
/// deeper — `tests/common/mod.rs`, `tests/ui/**` trybuild fixtures — is either
/// a module compiled into other binaries or input data, and has no binary name
/// for a `binary(=…)` filter to match. Getting this wrong does not merely
/// produce a confusing diagnostic: it invents a binary name ("mod") that the
/// fence's own membership and existence checks would both ACCEPT, so the
/// suggested fix would read as protection while protecting nothing.
fn is_test_binary(rel: &str) -> bool {
    let Some((_, tail)) = rel.split_once("/tests/") else {
        return false;
    };
    !tail.contains('/') && tail.ends_with(".rs")
}

#[test]
fn only_a_depth_one_tests_file_is_a_test_binary() {
    assert!(is_test_binary(
        "crates/cerulion_core/tests/transport_test.rs"
    ));
    assert!(is_test_binary(
        "crates/cerulion_viz/bin/cerulion_vizd/tests/host_test.rs"
    ));
    // A nested module compiled INTO other binaries is not one.
    assert!(!is_test_binary("crates/cerulion_core/tests/common/mod.rs"));
    // trybuild fixtures are input data, not targets.
    assert!(!is_test_binary("crates/cerulion_core/tests/ui/pass/x.rs"));
    // `src/` is not a test target at all.
    assert!(!is_test_binary("crates/cerulion_core/src/lib.rs"));
    // The real helper this rule exists for must classify as NOT a binary,
    // otherwise the fence's escape hatch stays unreachable.
    assert!(!is_test_binary("crates/cerulion_core/tests/common/mod.rs"));
}

/// The binaries a `[[profile.default.overrides]]` block assigns to `group`,
/// read out of the nextest config.
///
/// Deliberately a small line-oriented reader rather than a TOML dependency:
/// the file is ours, its shape is fixed by this gate, and the ONE thing that
/// must not happen is a parser that silently returns an empty set and makes
/// every assertion below vacuous — so an override block naming the group but
/// yielding no binaries is an error, not an empty answer.
///
/// Only the EXACT-match form `binary(=name)` is read. The globbing form
/// `binary(name)` is a substring match that would quietly pull in a future
/// `transport_test_v2`, so a filter using it yields nothing here and fails the
/// membership check with the offending line printed.
fn fence_binaries(config: &str, group: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut in_override = false;
    let mut block = String::new();
    let mut blocks: Vec<String> = Vec::new();
    for line in config.lines() {
        let t = line.trim();
        if t.starts_with("[[") || (t.starts_with('[') && !t.starts_with("[[")) {
            if in_override {
                blocks.push(std::mem::take(&mut block));
            }
            in_override = t == "[[profile.default.overrides]]";
            continue;
        }
        if in_override {
            block.push_str(line);
            block.push('\n');
        }
    }
    if in_override {
        blocks.push(block);
    }
    for b in blocks {
        // A `#` comment is prose, not configuration.
        let code: String = b
            .lines()
            .map(|l| l.split('#').next().unwrap_or(""))
            .collect::<Vec<_>>()
            .join("\n");
        if !code.contains(&format!("'{group}'")) && !code.contains(&format!("\"{group}\"")) {
            continue;
        }
        let mut rest = code.as_str();
        while let Some(at) = rest.find("binary(=") {
            rest = &rest[at + "binary(=".len()..];
            let Some(end) = rest.find(')') else { break };
            out.insert(rest[..end].trim().to_string());
            rest = &rest[end..];
        }
    }
    out
}

/// The binaries carrying `threads-required = 'num-cpus'` in a `default`
/// override block.
///
/// Reuses [`fence_binaries`], whose `marker` is any quoted token that
/// identifies the block — here the VALUE `'num-cpus'` rather than a group name,
/// since a `threads-required` override joins no group. That is a little
/// oblique, so the failure mode is worth stating: every drift it could produce
/// (a numeric `threads-required = 4`, the two overrides merged into one block,
/// `'num-cpus'` appearing in the other block) surfaces LOUDLY as a set
/// mismatch in [`the_fence_membership_equals_the_declared_inventory`], because
/// the inventory is compared in both directions. None of them can pass
/// silently.
fn threads_required_binaries(config: &str) -> BTreeSet<String> {
    fence_binaries(config, "num-cpus")
}

#[test]
fn the_fence_reader_reads_only_exact_binary_filters_from_the_right_block() {
    let cfg = "\
[test-groups]
g = { max-threads = 1 }

[[profile.default.overrides]]
filter = 'binary(=a) + binary(=b)'
test-group = 'g'

[[profile.default.overrides]]
filter = 'binary(=c)'
threads-required = 'num-cpus'

[profile.default]
fail-fast = false
";
    assert_eq!(
        fence_binaries(cfg, "g"),
        ["a", "b"].iter().map(|s| s.to_string()).collect()
    );
    // The other block's binary must NOT leak into the group.
    assert!(!fence_binaries(cfg, "g").contains("c"));
    assert_eq!(
        threads_required_binaries(cfg),
        ["c"].iter().map(|s| s.to_string()).collect()
    );
    // A group nobody assigns yields nothing (and the caller treats that as an
    // error, so it cannot pass silently).
    assert!(fence_binaries(cfg, "absent").is_empty());
    // The GLOBBING form is deliberately not read — it is a substring match.
    let glob = "[[profile.default.overrides]]\nfilter = 'binary(a)'\ntest-group = 'g'\n";
    assert!(
        fence_binaries(glob, "g").is_empty(),
        "`binary(name)` is a substring match and must not be read as membership"
    );
    // A `#` comment naming the group does not create a block, and a comment
    // naming a binary does not create a member.
    let commented = "\
[[profile.default.overrides]]
# test-group = 'g' and binary(=ghost)
filter = 'binary(=real)'
test-group = 'g'
";
    assert_eq!(
        fence_binaries(commented, "g"),
        ["real"].iter().map(|s| s.to_string()).collect()
    );
}

/// Every `pub fn` in `transport/mod.rs` that reaches the `INSTANCE` static,
/// directly or by delegating to one that does.
///
/// Text-derived by fixpoint: seed with the functions whose body names
/// `INSTANCE`, then repeatedly add any `pub fn` calling `Self::<seed>(`. That
/// is how `init` (which only calls `Self::init_singleton`) and `init_default`
/// (which only calls `Self::init`) are reached — neither mentions `INSTANCE`.
fn derived_singleton_accessors(src: &str) -> BTreeSet<String> {
    let code = code_only(src);
    let ch: Vec<char> = code.chars().collect();
    // All indices below are CHAR indices. The file contains multi-byte
    // characters (em dashes in its doc comments), so mixing them with the
    // BYTE offsets `str::find` returns panics on a boundary.
    let at = |i: usize, pat: &str| -> bool {
        let p: Vec<char> = pat.chars().collect();
        i + p.len() <= ch.len() && ch[i..i + p.len()] == p[..]
    };

    // (name, is_pub, body) for every fn. PRIVATE fns are collected too: they
    // carry the delegation edges (`init` reaches `INSTANCE` only through the
    // private `init_singleton`), but only public ones can be called by a test,
    // so only those need watching.
    let mut fns: Vec<(String, bool, String)> = Vec::new();
    let mut i = 0usize;
    while i < ch.len() {
        if !at(i, "fn ") {
            i += 1;
            continue;
        }
        let prev: String = ch[i.saturating_sub(4)..i].iter().collect();
        let is_pub = prev.trim_end().ends_with("pub");
        let mut j = i + 3;
        let mut name = String::new();
        while j < ch.len() && (ch[j].is_alphanumeric() || ch[j] == '_') {
            name.push(ch[j]);
            j += 1;
        }
        i = j;
        if name.is_empty() {
            continue;
        }
        let mut k = j;
        while k < ch.len() && ch[k] != '{' && ch[k] != ';' {
            k += 1;
        }
        if k >= ch.len() || ch[k] == ';' {
            continue;
        }
        let mut depth = 0i32;
        let mut close = k;
        for (idx, c) in ch.iter().enumerate().skip(k) {
            if *c == '{' {
                depth += 1;
            } else if *c == '}' {
                depth -= 1;
                if depth == 0 {
                    close = idx;
                    break;
                }
            }
        }
        if close > k {
            fns.push((name, is_pub, ch[k..close].iter().collect()));
            i = close;
        }
    }

    let mut set: BTreeSet<String> = fns
        .iter()
        .filter(|(_, _, body)| body.contains("INSTANCE"))
        .map(|(n, _, _)| n.clone())
        .collect();
    loop {
        let before = set.len();
        for (name, _, body) in &fns {
            if set.contains(name) {
                continue;
            }
            if set.iter().any(|s| body.contains(&format!("Self::{s}("))) {
                set.insert(name.clone());
            }
        }
        if set.len() == before {
            break;
        }
    }
    fns.iter()
        .filter(|(n, is_pub, _)| *is_pub && set.contains(n))
        .map(|(n, _, _)| n.clone())
        .collect()
}

/// The watched list must cover every door the singleton actually has.
#[test]
fn the_watched_accessor_set_matches_the_singletons_own_doors() {
    let root = repo_root();
    let src = fs::read_to_string(root.join("crates/cerulion_core/src/transport/mod.rs"))
        .expect("read transport/mod.rs");
    let derived = derived_singleton_accessors(&src);

    // Vacuity guard: the fixpoint must find the door everybody knows about.
    assert!(
        derived.contains("get_or_init"),
        "the accessor derivation found {derived:?} and not even `get_or_init` — it has gone \
         stale (the static was renamed, or the impl moved) and every claim below is vacuous"
    );

    let watched: BTreeSet<String> = SINGLETON_CALLS
        .iter()
        .map(|c| {
            c.trim_start_matches("TransportManager::")
                .trim_end_matches('(')
                .to_string()
        })
        .collect();

    let unwatched: Vec<&String> = derived.iter().filter(|n| !watched.contains(*n)).collect();
    assert!(
        unwatched.is_empty(),
        "`transport/mod.rs` has singleton accessor(s) this gate does not watch: {unwatched:?}. \
         A test calling one races every other singleton test in its package while this gate \
         reports a clean sheet.\n\nFIX: add the call to `SINGLETON_CALLS`, or — if the new \
         accessor does NOT touch `INSTANCE` (the `init_for_test` shape) — say so here."
    );

    // The other direction: a watched call that no longer exists means the list
    // is stale and the gate is watching a token nothing can produce.
    let vanished: Vec<&String> = watched
        .iter()
        .filter(|n| !derived.contains(*n) && !src.contains(&format!("pub fn {n}(")))
        .collect();
    assert!(
        vanished.is_empty(),
        "`SINGLETON_CALLS` names accessor(s) that no longer exist: {vanished:?}"
    );
}

/// Packages whose test serialisation is delegated to a runner this text walk
/// cannot read — e.g. a sharding script that derives package names from a
/// matrix variable rather than naming them on the `run:` line.
///
/// Empty today. The walk already treats a `run:` line that invokes a
/// `scripts/*.sh` AND names the package as serial (the script owns the flag),
/// so this list is only for the case where the package name never appears in
/// the workflow text at all.
const SERIAL_BY_RUNNER: &[(&str, &str)] = &[];

/// `tests/` HELPER modules that call the singleton, with why that is safe.
///
/// A helper has no `#[test]` of its own, so the per-file pairing rule cannot
/// speak for it: its call executes inside every binary that includes it, and
/// the tests that would need `#[serial]` live in those other files. Each such
/// module must therefore be named here with the reason, rather than skipped
/// because it happens to contain no tests.
///
/// Empty today — `cerulion_core/tests/common/mod.rs` is a pure source-walk
/// helper that opens no transport.
const SINGLETON_HELPER_MODULES: &[(&str, &str)] = &[];

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_core lives one level under the repo root")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

/// Count of non-overlapping occurrences of an attribute like `#[serial]`,
/// tolerating whitespace and a `serial_test::` path prefix.
fn count_attr(src: &str, names: &[&str]) -> usize {
    let mut n = 0usize;
    let bytes: Vec<char> = src.chars().collect();
    let mut i = 0usize;
    while i + 1 < bytes.len() {
        if bytes[i] != '#' || bytes[i + 1] != '[' {
            i += 1;
            continue;
        }
        let mut j = i + 2;
        while j < bytes.len() && bytes[j].is_whitespace() {
            j += 1;
        }
        let rest: String = bytes[j..bytes.len().min(j + 48)].iter().collect();
        let rest = rest
            .strip_prefix("serial_test::")
            .unwrap_or(&rest)
            .to_string();
        if names.iter().any(|name| {
            rest.strip_prefix(name)
                .map(|tail| tail.trim_start())
                .is_some_and(|tail| tail.starts_with(']') || tail.starts_with('('))
        }) {
            n += 1;
        }
        i += 2;
    }
    n
}

#[test]
fn the_attribute_counter_counts_the_forms_that_actually_appear() {
    let src = "#[test]\n#[serial]\n#[ serial ]\n#[serial_test::serial]\n#[tokio::test]\n\
               #[serial(group)]\n#[serialize]\n";
    assert_eq!(count_attr(src, &["test", "tokio::test"]), 2);
    // `#[serialize]` must NOT count as `#[serial]` — the closing-delimiter
    // check is what stops a prefix match.
    assert_eq!(count_attr(src, &["serial"]), 4);
}

/// Package name declared by the nearest ancestor `Cargo.toml`.
fn package_of(root: &Path, rel: &Path) -> Option<String> {
    let mut dir = rel.parent()?.to_path_buf();
    loop {
        let manifest = root.join(&dir).join("Cargo.toml");
        if manifest.exists() {
            let text = fs::read_to_string(&manifest).ok()?;
            for line in text.lines() {
                let t = line.trim();
                if let Some(rest) = t.strip_prefix("name") {
                    let rest = rest.trim_start();
                    if let Some(rest) = rest.strip_prefix('=') {
                        return Some(rest.trim().trim_matches('"').to_string());
                    }
                }
                if t.starts_with('[') && t != "[package]" && !out_of_package_section(t) {
                    break;
                }
            }
        }
        if !dir.pop() {
            return None;
        }
    }
}

fn out_of_package_section(section: &str) -> bool {
    section == "[package]"
}

/// How ONE CI invocation runs a package's `tests/` binaries.
///
/// The three arms protect the process-global iceoryx2 singleton in three
/// different ways, and collapsing them to a bool hides
/// the one that matters most.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RunnerPolicy {
    /// `cargo test … -- --test-threads=1`. libtest is serialised wholesale, so
    /// no per-file rule is needed: nothing can run beside anything.
    LibtestSerial,
    /// `cargo nextest run …`. Each test gets its OWN PROCESS, which dissolves
    /// the cdylib-`NODES` / `set_var` / `#[traced_test]` / global-allocator
    /// classes outright — but a fresh process does NOT isolate the DEFAULT
    /// iceoryx2 namespace, which is a MACHINE resource. Protection there is
    /// FENCE MEMBERSHIP (`.config/nextest.toml`), and neither `#[serial]` (a
    /// mutex nothing else shares) nor `--test-threads=1` (a flag nextest does
    /// not take) can substitute for it.
    Nextest,
    /// `cargo test` with no flag: libtest threads share one process.
    Parallel,
}

/// What a package's invocations, taken together, say about it.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
struct PackageRunners {
    parallel: bool,
    nextest: bool,
    serial: bool,
}

impl PackageRunners {
    fn note(&mut self, p: RunnerPolicy) {
        match p {
            RunnerPolicy::Parallel => self.parallel = true,
            RunnerPolicy::Nextest => self.nextest = true,
            RunnerPolicy::LibtestSerial => self.serial = true,
        }
    }
}

/// For every package named on a test-running line in any workflow: the set of
/// runner policies it is invoked under.
///
/// Every invocation is folded in, not just the strictest. One parallel step is
/// enough to flake, and one NEXTEST step is enough to require the fence, so a
/// package running three ways is subject to the rules of all three.
fn ci_serialised_packages(root: &Path) -> BTreeMap<String, PackageRunners> {
    let dir = root.join(".github/workflows");
    let entries = fs::read_dir(&dir).expect("read .github/workflows");
    let mut texts: Vec<String> = Vec::new();
    for entry in entries {
        let path = entry.expect("dir entry").path();
        // BOTH extensions: GitHub reads `.yml` and `.yaml` alike, so a
        // workflow saved under the other spelling was invisible to this
        // reader — and an invisible PARALLEL step is the direction that
        // matters, since the map is a conjunction.
        let ext = path.extension().and_then(|e| e.to_str());
        if ext != Some("yml") && ext != Some("yaml") {
            continue;
        }
        texts.push(fs::read_to_string(&path).expect("read workflow"));
    }
    serialised_packages_from_texts(root, texts.iter().map(String::as_str))
}

/// The pure half of [`ci_serialised_packages`], so the reader's rules can be
/// driven against hand-written workflow text instead of only against whatever
/// `.github/workflows` happens to contain today.
fn serialised_packages_from_texts<'a>(
    root: &Path,
    texts: impl IntoIterator<Item = &'a str>,
) -> BTreeMap<String, PackageRunners> {
    let mut map: BTreeMap<String, PackageRunners> = BTreeMap::new();
    for text in texts {
        for raw in text.lines() {
            // A YAML `#` comment is PROSE, not CI configuration. Two comments
            // in `ci.yml` already minted phantom packages this way — a reader
            // that believes commented-out or explanatory text is describing
            // the lane is reading fiction.
            let line = strip_yaml_comment(raw);
            let line = line.as_str();
            let script = script_named(line);
            if !runs_tests(line) && script.is_none() {
                continue;
            }
            if !line_can_run_a_tests_dir_file(line) {
                continue;
            }
            // A script's own text decides. `is_script => serialises` was an
            // ASSUMPTION, and the anchor below claimed it would catch a lost
            // flag — it could not: dropping `-- --test-threads=1` from
            // `ci_test_shard.sh` left every gate green while ~30 singleton
            // files ran in parallel.
            let policy = match script {
                Some(rel) => match script_thread_policy(root, &rel) {
                    // Read the script: it says how it runs tests.
                    Some(p) => p,
                    // No evidence. NOT parallel: an unreadable runner
                    // contributes nothing and the package falls back to its
                    // other lines (or to `SERIAL_BY_RUNNER`).
                    None => continue,
                },
                None => line_policy(line),
            };
            for pkg in packages_named(line)
                .into_iter()
                .chain(shard_script_package(line))
            {
                map.entry(pkg).or_default().note(policy);
            }
        }
    }
    map
}

/// `line` with any YAML `#` comment removed. A `#` inside quotes is content.
fn strip_yaml_comment(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let (mut sq, mut dq) = (false, false);
    let chars: Vec<char> = line.chars().collect();
    for (i, &c) in chars.iter().enumerate() {
        match c {
            '\'' if !dq => sq = !sq,
            '"' if !sq => dq = !dq,
            // YAML starts a comment at a `#` that begins the line or follows
            // whitespace; `foo#bar` is a value.
            '#' if !sq && !dq && (i == 0 || chars[i - 1].is_whitespace()) => break,
            _ => {}
        }
        out.push(c);
    }
    out
}

/// `--test-threads=1` as a WHOLE token.
///
/// Substring matching read `--test-threads=16` as serial — the same hazard
/// `line_can_run_a_tests_dir_file` already guards against for `--doc`, so the
/// bar is the file's own.
fn has_thread_flag(line: &str) -> bool {
    line.split_whitespace().any(|t| t == "--test-threads=1")
}

/// Does this command line RUN tests at all?
///
/// Both runners count. Matching only `cargo test` was correct while that was
/// the only runner; after the nextest adoption it made the shard lane — the one
/// that actually runs `cerulion_core`'s `tests/` files on a pull request —
/// invisible, so the package's verdict would have rested entirely on the
/// release-latency and nightly lines that happen to name it with `-p`.
fn runs_tests(line: &str) -> bool {
    line.contains("cargo test") || is_nextest(line)
}

/// `cargo nextest run` as a sequence of WHOLE tokens.
///
/// Token-wise so a step NAME mentioning nextest, or a `--nextest-something`
/// flag, is not read as an invocation.
fn is_nextest(line: &str) -> bool {
    let toks: Vec<&str> = line.split_whitespace().collect();
    toks.windows(3)
        .any(|w| w[0] == "cargo" && w[1] == "nextest" && w[2] == "run")
}

/// How ONE command line runs its tests.
///
/// `--test-threads=1` is meaningless to nextest (it takes no libtest
/// arguments), so the nextest arm is checked FIRST: a line carrying both would
/// otherwise be read as libtest-serial and exempt its package from the fence
/// rule, which is the silent-inversion direction.
fn line_policy(line: &str) -> RunnerPolicy {
    if is_nextest(line) {
        RunnerPolicy::Nextest
    } else if has_thread_flag(line) {
        RunnerPolicy::LibtestSerial
    } else {
        RunnerPolicy::Parallel
    }
}

/// The repo-relative path of a `scripts/….sh` invoked on `line`, if any.
fn script_named(line: &str) -> Option<String> {
    line.split_whitespace().find_map(|t| {
        let t = t.trim_start_matches("./");
        // Whole-token match: a mid-string `find` would silently truncate the
        // `tools/` prefix of `tools/scripts/x.sh` and open the wrong path.
        let path = if t.starts_with("tools/scripts/") || t.starts_with("scripts/") {
            t
        } else {
            return None;
        };
        path.ends_with(".sh").then(|| path.to_string())
    })
}

/// How does the named script run the tests it runs?
///
/// `None` means NO EVIDENCE — the file is missing, or it runs no tests at all.
/// That is deliberately not read as parallel (see the call site); an unreadable
/// runner leaves the package to its other evidence.
///
/// EVERY test-running line in the script is folded in, and the WEAKEST wins:
/// one unserialised `cargo test` is enough to flake, so `Parallel` beats
/// `Nextest` beats `LibtestSerial`. A script that runs tests two ways is only
/// as strong as its weakest line.
fn script_thread_policy(root: &Path, rel: &str) -> Option<RunnerPolicy> {
    let text = fs::read_to_string(root.join(rel)).ok()?;
    let mut weakest: Option<RunnerPolicy> = None;
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("");
        if !runs_tests(line) {
            continue;
        }
        if !line_can_run_a_tests_dir_file(line) {
            continue;
        }
        let p = line_policy(line);
        weakest = Some(match weakest {
            None => p,
            Some(w) => weaker(w, p),
        });
    }
    weakest
}

/// The weaker of two policies, on the ordering `Parallel < Nextest < Serial`.
fn weaker(a: RunnerPolicy, b: RunnerPolicy) -> RunnerPolicy {
    let rank = |p: RunnerPolicy| match p {
        RunnerPolicy::Parallel => 0,
        RunnerPolicy::Nextest => 1,
        RunnerPolicy::LibtestSerial => 2,
    };
    if rank(a) <= rank(b) {
        a
    } else {
        b
    }
}

/// The package a `scripts/ci_test_shard.sh` invocation names POSITIONALLY.
///
/// The lane that actually runs `cerulion_core`'s `tests/` files on a pull
/// request is `./tools/scripts/ci_test_shard.sh cerulion_core ${{ matrix.shard }} 4`
/// — no `-p`, so [`packages_named`] finds nothing and the gate's verdict for
/// the package would rest entirely on the release-latency and nightly lines
/// that happen to name it with `-p`. Reading the positional argument grounds
/// it in the lane that runs the files instead.
///
/// Such a line runs under NEXTEST, and it is read from the script rather than
/// assumed: `script_thread_policy` opens `scripts/ci_test_shard.sh` and finds
/// its `exec cargo nextest run --profile … "$@"`. (A policy stated in this
/// comment would be true until the script changed and false after, which is
/// exactly why
/// the policy is read out of the file instead of stated here.) That function
/// supplies the POLICY; this one only supplies the package NAME.
///
/// Usage is `ci_test_shard.sh <package> <shard_index> <shard_count>`, with an
/// optional leading `--list` / `--check` / `--` separator, so the package is
/// the first following token that is not a flag and not an unexpanded
/// `${{ … }}` expression.
fn shard_script_package(line: &str) -> Option<String> {
    let toks: Vec<&str> = line.split_whitespace().collect();
    let at = toks.iter().position(|t| t.contains("ci_test_shard.sh"))?;
    toks.iter().skip(at + 1).find_map(|t| {
        // A flag, or a fragment of an unexpanded `${{ … }}` expression (which
        // `split_whitespace` breaks into several tokens). A package name
        // contains none of these characters, so skipping them cannot skip a
        // real one — and refusing to guess is what keeps a matrix-derived
        // package name in `SERIAL_BY_RUNNER`'s territory instead of inventing
        // a package called `matrix`.
        if t.starts_with('-') || t.contains(['$', '{', '}', '.']) {
            return None;
        }
        let name: String = t
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_' || *c == '-')
            .collect();
        // A shard index / count is positional too; a package name never
        // parses as a number.
        if name.is_empty() || name.chars().all(|c| c.is_ascii_digit()) {
            None
        } else {
            Some(name)
        }
    })
}

/// `true` when a `cargo test` invocation is capable of running a file under a
/// package's `tests/` directory — which is this gate's ENTIRE scope.
///
/// `cargo test -p <pkg> --doc` runs ONLY doctests. Doctests live in `src/` doc
/// comments, which the module header already places out of scope, so such a
/// step is not evidence about how the package's INTEGRATION tests are threaded
/// and must not enter the conjunction below.
///
/// This is not a hypothetical: there are four
/// `cargo test -p cerulion_core --doc` steps (`ci.yml` x2 and two
/// nightly mirrors kept outside this tree), none carrying
/// `--test-threads=1`. Folded in, they flip `cerulion_core` to PARALLEL and the
/// gate reports ~20 files as violations — every one of which CI really does run
/// single-threaded, through `scripts/ci_test_shard.sh` (which applies the flag
/// unconditionally at its `exec cargo test "$@" -- --test-threads=1`).
///
/// Deliberately NARROW: it excludes one INVOCATION FORM that provably cannot
/// reach a `tests/` file. It exempts no package and no file — a genuinely
/// parallel integration-test step still counts against its package.
fn line_can_run_a_tests_dir_file(line: &str) -> bool {
    !line.split_whitespace().any(|t| t == "--doc")
}

/// Package names following a `-p` / `--package` flag on a command line.
fn packages_named(line: &str) -> Vec<String> {
    let toks: Vec<&str> = line.split_whitespace().collect();
    let mut out = Vec::new();
    for (i, t) in toks.iter().enumerate() {
        if *t == "-p" || *t == "--package" {
            if let Some(next) = toks.get(i + 1) {
                let name = next.trim_matches(|c: char| !(c.is_alphanumeric() || c == '_'));
                if !name.is_empty() {
                    out.push(name.to_string());
                }
            }
        } else if let Some(name) = t.strip_prefix("--package=") {
            out.push(name.to_string());
        }
    }
    out
}

#[test]
fn the_workflow_reader_finds_packages_and_their_thread_policy() {
    assert_eq!(
        packages_named("run: cargo test -p cerulion_core --tests --lib -- --test-threads=1"),
        vec!["cerulion_core".to_string()]
    );
    assert_eq!(
        packages_named("cargo test --workspace --exclude camera_jpeg -- --test-threads=1"),
        Vec::<String>::new()
    );
    assert_eq!(
        packages_named("cargo test --package=cerulion_bag"),
        vec!["cerulion_bag".to_string()]
    );

    // A `--doc` step runs doctests ONLY, so it says nothing about how the
    // package's `tests/` files are threaded and must not enter the
    // conjunction. Every other shape does.
    assert!(
        !line_can_run_a_tests_dir_file("        run: cargo test -p cerulion_core --doc"),
        "a `--doc` invocation cannot run a `tests/` file"
    );
    assert!(
        line_can_run_a_tests_dir_file(
            "        run: cargo test -p cerulion_core --tests --lib -- --test-threads=1"
        ),
        "an ordinary invocation must still be read"
    );
    assert!(
        line_can_run_a_tests_dir_file("        run: cargo test -p cerulion_bag"),
        "a bare package invocation runs its `tests/` files"
    );
    // Whole-token match: a step whose NAME merely mentions the flag, or a
    // test binary whose name contains it, is not a `--doc` invocation.
    assert!(
        line_can_run_a_tests_dir_file("        run: cargo test -p cerulion_core --test doc_test"),
        "`--doc` must match as a whole token, not a substring"
    );

    // The shard lane names its package POSITIONALLY, and is serial by
    // construction (the script's own `exec cargo test "$@" -- --test-threads=1`).
    assert_eq!(
        shard_script_package("        run: ./tools/scripts/ci_test_shard.sh cerulion_core 1 4"),
        Some("cerulion_core".to_string())
    );
    assert_eq!(
        shard_script_package(
            "        run: ./tools/scripts/ci_test_shard.sh cerulion_core ${{ matrix.shard }} 4"
        ),
        Some("cerulion_core".to_string())
    );
    assert_eq!(
        shard_script_package("        run: ./tools/scripts/ci_test_shard.sh --check"),
        None
    );
    assert_eq!(
        shard_script_package("        run: ./tools/scripts/check_hot_path_allocs.sh"),
        None
    );
    let shard_only =
        "jobs:\n  a:\n    steps:\n      - run: ./tools/scripts/ci_test_shard.sh cerulion_core 0 4\n";
    let sharded_map = serialised_packages_from_texts(&repo_root(), [shard_only]);
    let core = sharded_map
        .get("cerulion_core")
        .copied()
        .expect("the shard lane names the package positionally");
    assert!(
        core.nextest && !core.parallel,
        "the shard lane is the one that runs the package's `tests/` files, and it now execs \
         `cargo nextest run` — so the package must read as NEXTEST (and never as parallel). \
         Got {core:?}; if `ci_test_shard.sh` moved back to `cargo test`, this reader and the \
         fence must move with it."
    );

    // A shard of a DIFFERENT package must not vouch for cerulion_core.
    let other_shard =
        "jobs:\n  a:\n    steps:\n      - run: ./tools/scripts/ci_test_shard.sh cerulion_viz 0 4\n";
    let other = serialised_packages_from_texts(&repo_root(), [other_shard]);
    assert!(other.get("cerulion_viz").is_some_and(|r| r.nextest));
    assert_eq!(other.get("cerulion_core"), None);

    // And with every evidence line gone the package is ABSENT, not assumed
    // serial — which is what makes the gate's anchor fail LOUDLY rather than
    // silently exempting every file below it.
    assert_eq!(
        serialised_packages_from_texts(
            &repo_root(),
            ["jobs:\n  a:\n    steps:\n      - run: echo hi\n"]
        )
        .get("cerulion_core"),
        None
    );
}

/// The three runner policies, told apart on hand-written command lines.
///
/// Driven directly rather than only through the workflow reader, because the
/// nextest arm is what decides whether a package is subject to the fence rule
/// — and reading a nextest line as libtest-serial is the silent-inversion
/// direction (it would EXEMPT the package from the one rule that protects it).
#[test]
fn the_line_reader_tells_the_three_runner_policies_apart() {
    assert_eq!(
        line_policy("        run: cargo test -p cerulion_core -- --test-threads=1"),
        RunnerPolicy::LibtestSerial
    );
    assert_eq!(
        line_policy("        run: cargo test -p cerulion_bag"),
        RunnerPolicy::Parallel
    );
    assert_eq!(
        line_policy("        run: cargo nextest run --profile ci -p cerulion_core"),
        RunnerPolicy::Nextest
    );
    // A line carrying BOTH reads as nextest: `--test-threads=1` is meaningless
    // to nextest, so believing it would exempt the package from the fence rule.
    assert_eq!(
        line_policy("        run: cargo nextest run -p x -- --test-threads=1"),
        RunnerPolicy::Nextest
    );
    // WHOLE tokens: a step NAME mentioning nextest is not an invocation.
    assert!(!is_nextest(
        "        name: move cargo test to nextest run later"
    ));
    assert!(is_nextest("  cargo nextest run -p x"));
    assert!(!is_nextest("  cargo nextest list -p x"));
    // Both runners count as running tests; neither `--doc` form does.
    assert!(runs_tests("cargo nextest run -p x"));
    assert!(runs_tests("cargo test -p x"));
    assert!(!runs_tests("cargo build -p x"));

    // The WEAKEST line wins when a script runs tests more than one way.
    assert_eq!(
        weaker(RunnerPolicy::Nextest, RunnerPolicy::LibtestSerial),
        RunnerPolicy::Nextest
    );
    assert_eq!(
        weaker(RunnerPolicy::Nextest, RunnerPolicy::Parallel),
        RunnerPolicy::Parallel
    );
    assert_eq!(
        weaker(RunnerPolicy::LibtestSerial, RunnerPolicy::LibtestSerial),
        RunnerPolicy::LibtestSerial
    );

    // The fold keeps EVERY policy a package is invoked under, so a package
    // running two ways is subject to both rules.
    let mixed = concat!(
        "jobs:\n  a:\n    steps:\n",
        "      - run: cargo nextest run -p cerulion_core\n",
        "      - run: cargo test -p cerulion_core --test x -- --test-threads=1\n",
    );
    let m = serialised_packages_from_texts(&repo_root(), [mixed]);
    let c = m.get("cerulion_core").copied().expect("named twice");
    assert!(c.nextest && c.serial && !c.parallel, "got {c:?}");
}

/// Every `.rs` file under `dir`, relative to `root`, skipping build output and
/// VCS state (`target/`, `.git/`) — those hold generated and vendored sources
/// that are nobody's test discipline.
fn collect_rs(root: &Path, dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if path.is_dir() {
            if name == "target" || name == ".git" || name == "node_modules" {
                continue;
            }
            // Another checkout's tree is not this branch's business.
            if common::is_other_checkout(&path) {
                continue;
            }
            collect_rs(root, &path, out);
        } else if name.ends_with(".rs") {
            if let Ok(rel) = path.strip_prefix(root) {
                out.push(rel.to_path_buf());
            }
        }
    }
}

/// Every `#[test]`/`#[tokio::test]` function in `src`, paired with whether its
/// OWN attribute block also carries `#[serial]`.
///
/// Returns `(total_tests, names_of_tests_without_serial)`.
///
/// This replaces a file-wide COUNT comparison (`serial >= tests`), which asked
/// a question one step away from the one that matters. Counts are equal in
/// cases the contract forbids: `#[serial]` on a non-test helper, or on a
/// `mod`, balances a `#[test]` that has none, and the violating test still runs
/// concurrently. Pairing each test with its own attributes cannot be fooled
/// that way — the attribute has to sit on the item it governs, which is also
/// the only place it does anything.
fn unserialised_tests(src: &str) -> (usize, Vec<String>) {
    let stripped = code_only(src);
    let lines: Vec<&str> = stripped.lines().collect();
    let mut attrs = String::new();
    let mut total = 0usize;
    let mut missing = Vec::new();
    let mut i = 0usize;
    while i < lines.len() {
        let trimmed = lines[i].trim();
        if trimmed.starts_with("#[") {
            let mut buf = trimmed.to_string();
            // An attribute list may wrap across lines.
            while buf.matches('[').count() > buf.matches(']').count() && i + 1 < lines.len() {
                i += 1;
                buf.push(' ');
                buf.push_str(lines[i].trim());
            }
            attrs.push(' ');
            attrs.push_str(&buf);
            i += 1;
            continue;
        }
        if trimmed.is_empty() || trimmed.starts_with("//") {
            i += 1;
            continue;
        }
        let is_fn = trimmed.starts_with("fn ")
            || trimmed.starts_with("async fn ")
            || trimmed.starts_with("pub fn ")
            || trimmed.starts_with("pub async fn ");
        if is_fn && count_attr(&attrs, &["test", "tokio::test"]) > 0 {
            total += 1;
            if count_attr(&attrs, &["serial"]) == 0 {
                let name = trimmed
                    .split("fn ")
                    .nth(1)
                    .and_then(|t| t.split(['(', '<']).next())
                    .unwrap_or("<unnamed>")
                    .trim()
                    .to_string();
                missing.push(name);
            }
        }
        attrs.clear();
        i += 1;
    }
    (total, missing)
}

#[test]
fn the_test_reader_pairs_each_test_with_its_own_serial_attribute() {
    // The shape the old count could not see: one `#[serial]` present, one
    // `#[test]` without it, and a `#[serial]` on a HELPER making the counts
    // equal. The file-wide comparison passed this; pairing must not.
    let src = "#[test]\n#[serial]\nfn good() {}\n\
               #[test]\nfn bad() {}\n\
               #[serial]\nfn helper() {}\n";
    assert_eq!(count_attr(src, &["test", "tokio::test"]), 2);
    assert_eq!(
        count_attr(src, &["serial"]),
        2,
        "counts are EQUAL — the old gate passed here"
    );
    let (total, missing) = unserialised_tests(src);
    assert_eq!(total, 2);
    assert_eq!(missing, vec!["bad".to_string()]);

    // Anti-tautology: a fully serial file reports nothing.
    let clean = "#[test]\n#[serial]\nfn a() {}\n#[tokio::test]\n#[serial]\nasync fn b() {}\n";
    let (t, m) = unserialised_tests(clean);
    assert_eq!((t, m.len()), (2, 0));

    // A non-test fn carrying `#[serial]` is not a test.
    let (t2, _) = unserialised_tests("#[serial]\nfn only_helper() {}\n");
    assert_eq!(t2, 0);
}

#[test]
fn a_singleton_touching_test_file_is_serial_unless_its_package_runs_single_threaded() {
    let root = repo_root();
    let serialised = ci_serialised_packages(&root);
    let by_runner: BTreeSet<&str> = SERIAL_BY_RUNNER.iter().map(|(p, _)| *p).collect();

    // The workflow reader must have actually read something, and it must be
    // able to tell the two policies apart — a reader that returned "serial"
    // for everything would make the whole gate vacuous.
    assert!(
        serialised.len() >= 10,
        "only {} package(s) were found on test-running lines in .github/workflows — the \
         workflow reader has broken, which would make this gate vacuous (found: {:?})",
        serialised.len(),
        serialised
    );
    let core = serialised.get("cerulion_core").copied().expect(
        "cerulion_core must be named on a test-running line somewhere in .github/workflows",
    );
    assert!(
        !core.parallel,
        "some workflow runs `cargo test -p cerulion_core` with NO `--test-threads=1` and NOT \
         under nextest — libtest would then thread the singleton binaries against each other. \
         Either restore the flag, move the step to `cargo nextest run`, or fix this reader."
    );
    assert!(
        core.nextest,
        "no workflow runs `cerulion_core` under `cargo nextest run` — the fence in \
         `{FENCE_FILE}` is then protecting nothing, and every fence assertion below is \
         describing a runner CI does not use. If the package was deliberately moved back to \
         `cargo test -- --test-threads=1`, delete the fence with it."
    );
    assert!(
        serialised.values().any(|r| r.parallel),
        "no package was classified as running in PARALLEL — the reader cannot tell the \
         policies apart, so every file below would be exempted silently"
    );
    assert!(
        serialised.values().any(|r| r.serial),
        "no package was classified as LIBTEST-SERIAL — the reader has collapsed, and the \
         `--test-threads=1` exemption below would never be granted to anyone"
    );

    // The fence, read from the config CI actually uses.
    let fence_cfg = fs::read_to_string(root.join(FENCE_FILE))
        .unwrap_or_else(|e| panic!("read {FENCE_FILE}: {e}"));
    let fence = fence_binaries(&fence_cfg, FENCE_GROUP);
    assert!(
        !fence.is_empty(),
        "the `{FENCE_GROUP}` group in {FENCE_FILE} names no binary — either the config lost \
         its override block or the reader has broken. Every fence assertion below would be \
         vacuous."
    );

    // A FILESYSTEM walk, deliberately not `git ls-files`: the file this gate
    // most needs to see is the one somebody just created, and an untracked
    // file is invisible to git. (Measured — the first mutation attempt for
    // this gate wrote a new violating test file, and a `git ls-files` walk
    // reported a clean sheet.)
    let mut all_rs: Vec<PathBuf> = Vec::new();
    collect_rs(&root, &root, &mut all_rs);
    all_rs.sort();

    let mut covered: Vec<String> = Vec::new();
    let mut violations: Vec<String> = Vec::new();
    let mut unfenced: Vec<String> = Vec::new();
    let mut helpers: Vec<String> = Vec::new();
    let mut singleton_callers_anywhere = 0usize;

    for path in &all_rs {
        let rel = path.to_string_lossy().to_string();
        let Ok(raw) = fs::read_to_string(root.join(path)) else {
            continue;
        };
        // LITERAL-BLANKING view: a file that NAMES an accessor in a string is
        // not a caller. Three files in this very directory do exactly that —
        // this one holds `SINGLETON_CALLS` as its watch list, and
        // `error_message_test.rs` asserts on an error message that mentions
        // `TransportManager::init()`. Under the preserving view all three read
        // as callers, which under the fence rule below would demand they be
        // fenced for calls they do not make.
        let src = code_only_blank_literals(&raw);
        let Some(found_call) = SINGLETON_CALLS.iter().find(|c| src.contains(**c)) else {
            continue;
        };
        singleton_callers_anywhere += 1;

        // Scope: a package's own `tests/` directory (see the module header
        // for why `src/` is deliberately out of scope).
        if !rel.contains("/tests/") {
            continue;
        }
        let Some(pkg) = package_of(&root, path) else {
            continue;
        };
        let runners = serialised.get(&pkg).copied().unwrap_or_default();

        // THE NEXTEST RULE, checked BEFORE the `--test-threads=1` exemption.
        //
        // Under nextest each test is its own process, so `#[serial]` (a mutex
        // nothing else shares) protects nothing and `--test-threads=1` is a
        // flag nextest does not take. The DEFAULT iceoryx2 namespace is a
        // MACHINE resource that a fresh process does not isolate, so the only
        // protection is fence membership — and a package running BOTH ways is
        // subject to this rule regardless of what its other lanes do.
        if runners.nextest {
            // Only a CREATING accessor can reach the default namespace here —
            // see `SINGLETON_CREATORS` for why `get()` alone cannot.
            if let Some(creator) = SINGLETON_CREATORS.iter().find(|c| src.contains(**c)) {
                // HELPER MODULES FIRST, and this ordering is the whole of the
                // fix. cargo builds a test BINARY only from a depth-1
                // `tests/*.rs`; a nested `tests/common/mod.rs` is compiled INTO
                // whichever binaries declare `mod common;`. Taking its
                // `file_stem` yields the binary name "mod", which names no
                // binary at all — so the diagnostic told the reader to fence
                // `mod`, and following that advice would have PASSED both the
                // membership check and the file-exists check (every `/tests/`
                // stem is in that set, "mod" among them) while fencing
                // nothing. An escape hatch that reads as protection and
                // protects nothing is worse than no hatch.
                //
                // `SINGLETON_HELPER_MODULES` is the mechanism written for this
                // shape, and it was unreachable on the nextest path.
                if !is_test_binary(&rel) {
                    if !SINGLETON_HELPER_MODULES.iter().any(|(f, _)| *f == rel) {
                        helpers.push(format!("  {rel} (package `{pkg}`, nextest)"));
                    }
                    continue;
                }
                let binary = Path::new(&rel)
                    .file_stem()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_default();
                if !fence.contains(&binary) {
                    unfenced.push(format!(
                        "  {rel} — package `{pkg}` runs under `cargo nextest run`, this file \
                         calls `{creator}`, and the binary `{binary}` is NOT in the \
                         `{FENCE_GROUP}` group"
                    ));
                }
            }
            continue;
        }

        if by_runner.contains(pkg.as_str()) || runners.serial {
            continue;
        }

        let (tests, unserialised) = unserialised_tests(&src);
        if tests == 0 {
            // A `tests/` file with NO test of its own but a singleton call is
            // a HELPER MODULE (`tests/common/mod.rs`), compiled INTO every
            // binary that declares `mod common;`. Its call runs under those
            // binaries' tests, so `continue` here dropped a real caller in
            // silence — the one shape where the file that does the calling
            // and the tests that must be `#[serial]` are different files.
            //
            // It is not classified automatically because the answer depends
            // on which binaries include it, which a text walk cannot know.
            if !SINGLETON_HELPER_MODULES.iter().any(|(f, _)| *f == rel) {
                helpers.push(format!("  {rel} (package `{pkg}`)"));
            }
            continue;
        }
        covered.push(format!("{rel} (package `{pkg}`, {tests} test(s))"));
        if !unserialised.is_empty() {
            violations.push(format!(
                "  {rel} — package `{pkg}` runs libtest in PARALLEL in CI, this file calls \
                 `{found_call}`, and {} of its {tests} test(s) carry no `#[serial]` of \
                 their own: {}",
                unserialised.len(),
                unserialised.join(", ")
            ));
        }
    }

    // If the trigger token ever stops appearing anywhere, this gate has gone
    // blind and would report a clean sheet forever.
    assert!(
        singleton_callers_anywhere >= 10,
        "only {singleton_callers_anywhere} file(s) in the repo call `{SINGLETON_CALL}` — the \
         accessor was probably renamed, and this gate is now watching a token that no longer \
         exists. FIX: update `SINGLETON_CALL`."
    );
    assert!(
        !covered.is_empty(),
        "no test file is COVERED by the `#[serial]` half of this gate (a `tests/` file calling \
         `{SINGLETON_CALL}` in a package CI runs libtest-parallel). `cerulion_cli`'s \
         `viz_verb_e2e_test` and `mp_consumer_first_spawn_e2e_test` are exactly that shape, so \
         an empty list means the package/workflow mapping has broken rather than that the \
         repo got tidier."
    );

    // The walk must never have left this checkout (M6): a sibling worktree's
    // in-progress or deliberately-mutated files are not this branch's code,
    // and reading them makes the verdict depend on the development machine.
    assert!(
        !all_rs
            .iter()
            .any(|p| p.to_string_lossy().contains(".claude/worktrees")),
        "the source walk escaped into another checkout: {:?}",
        all_rs
            .iter()
            .filter(|p| p.to_string_lossy().contains(".claude/worktrees"))
            .take(3)
            .collect::<Vec<_>>()
    );

    assert!(
        helpers.is_empty(),
        "a `tests/` HELPER module calls the process-global manager but declares no test of \
         its own, so the per-file `#[serial]` rule cannot speak for it:\n{}\n\nIts call runs \
         inside every binary that declares `mod <helper>;`, and those binaries' tests are \
         where `#[serial]` would have to go.\n\nFIX: classify it in `SINGLETON_HELPER_MODULES` \
         with the reason (e.g. it opens no transport), or move the call into the tests that \
         own it.",
        helpers.join("\n")
    );

    assert!(
        unfenced.is_empty(),
        "a test binary that drives the process-global iceoryx2 manager is NOT in the \
         `{FENCE_GROUP}` fence, and its package runs under nextest:\n{}\n\nUnder nextest every \
         test is its own PROCESS, so `#[serial]` serialises nothing (each process has its own \
         mutex) and `--test-threads=1` is not a flag nextest takes. A fresh process does not \
         isolate the DEFAULT iceoryx2 namespace — that is a MACHINE resource — so two such \
         binaries WILL run at the same time on one namespace. It does not fail cleanly; it \
         flakes.\n\nFIX: add the binary to the `{FENCE_GROUP}` override in `{FENCE_FILE}` AND \
         to `FENCE_INVENTORY` in this file with the reason. If the file does not really touch \
         the default namespace (it mints a per-test root with `init_for_test` /\n\
         `generate_isolated_config` / `build_for_test`), the right fix is to stop calling the \
         singleton instead — that is what keeps the fence small.",
        unfenced.join("\n")
    );

    assert!(
        violations.is_empty(),
        "a test that drives the process-global iceoryx2 manager runs concurrently with its \
         siblings:\n{}\n\nThis does not fail cleanly — it flakes, which is the phantom-pass \
         class. FIX: put `#[serial]` (from the `serial_test` dev-dependency) on every \
         `#[test]` in the file, or add `-- --test-threads=1` to the package's `cargo test` \
         step in .github/workflows.\n\nCurrently covered files:\n{}",
        violations.join("\n"),
        covered
            .iter()
            .map(|c| format!("  {c}"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Collect every `#[ignore]`d arm under `dir`, RECURSIVELY, as
/// `(path relative to tests/, module-qualified fn path)`.
///
/// Recursive because a Rust integration test compiles nested modules:
/// `tests/common/mod.rs` is declared by two binaries in this crate and carries
/// `#[test]` arms of its own. A top-level-only walk cannot see them, so an
/// `#[ignore]` added there would enter no inventory and be owned by nobody —
/// silently, which is the one failure mode this whole file exists to prevent.
///
/// Module-QUALIFIED because two arms in different modules may legally share a
/// bare name; keyed on the bare name they would collapse to one identity and a
/// single declaration would satisfy both. The path is tracked over the
/// literal-blanked view so a brace inside a string cannot move the depth.
///
/// `tests/ui/` is skipped, and the skip is GUARDED rather than assumed: those
/// are trybuild FIXTURES — input data compiled by `trybuild`, never linked as
/// test modules — so an `#[ignore]` there would run nowhere and mean nothing.
/// The guard asserts they contain no `#[test]`, so if that ever stops being
/// true the exclusion fails loudly instead of quietly hiding real arms.
fn ignored_arms_under(dir: &Path, rel: &str, out: &mut BTreeSet<(String, String)>) {
    let mut entries: Vec<PathBuf> = fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .map(|e| e.expect("dir entry").path())
        .collect();
    entries.sort();

    for path in entries {
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .expect("utf-8 path component")
            .to_string();
        let child_rel = if rel.is_empty() {
            name.clone()
        } else {
            format!("{rel}/{name}")
        };

        if path.is_dir() {
            if name == "ui" {
                guard_trybuild_fixtures(&path);
                continue;
            }
            ignored_arms_under(&path, &child_rel, out);
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }

        let src =
            fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        for arm in ignored_arms_in_source(&src, &child_rel) {
            assert!(
                out.insert((child_rel.clone(), arm.clone())),
                "{child_rel} declares `{arm}` twice — the module tracker collapsed two distinct \
                 arms into one identity, which would let ONE inventory entry satisfy BOTH"
            );
        }
    }
}

/// `tests/ui/` holds trybuild fixtures, never compiled as test modules.
fn guard_trybuild_fixtures(dir: &Path) {
    for entry in fs::read_dir(dir).unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display())) {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            guard_trybuild_fixtures(&path);
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        let src =
            fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        assert!(
            !code_only_blank_literals(&src).contains("#[test]"),
            "{} carries a #[test] attribute, so `tests/ui/` is no longer fixtures-only and this \
             walk must stop skipping it — an #[ignore]d arm in there would be owned by nobody",
            path.display()
        );
    }
}

/// Module-qualified names of the `#[ignore]`d arms in one source file.
fn ignored_arms_in_source(src: &str, rel: &str) -> Vec<String> {
    let view = code_only_blank_literals(src);
    let mut mods: Vec<(String, i32)> = Vec::new();
    let mut depth: i32 = 0;
    let mut pending = false;
    let mut found = Vec::new();

    for raw in view.lines() {
        let line = raw.trim_start();

        while let Some((_, entered_at)) = mods.last() {
            if depth <= *entered_at {
                mods.pop();
            } else {
                break;
            }
        }

        if let Some(rest) = line
            .strip_prefix("pub mod ")
            .or_else(|| line.strip_prefix("mod "))
        {
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() && rest.contains('{') {
                mods.push((name, depth));
            }
        } else if line.starts_with("#[ignore") {
            pending = true;
        } else if pending && !(line.starts_with("#[") || line.is_empty()) {
            let after_vis = line.strip_prefix("pub ").unwrap_or(line);
            let after_async = after_vis.strip_prefix("async ").unwrap_or(after_vis);
            let fn_name = after_async.strip_prefix("fn ").unwrap_or_else(|| {
                panic!(
                    "{rel}: an #[ignore] attribute is followed by `{line}`, a shape this walk does \
                     not understand. Teach it that shape rather than leaving the arm unclassified."
                )
            });
            let name: String = fn_name
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            assert!(
                !name.is_empty(),
                "{rel}: could not read a fn name after #[ignore]"
            );
            let mut qualified: Vec<&str> = mods.iter().map(|(m, _)| m.as_str()).collect();
            qualified.push(&name);
            found.push(qualified.join("::"));
            pending = false;
        }

        depth += raw.matches('{').count() as i32;
        depth -= raw.matches('}').count() as i32;
    }

    found
}

/// Every `#[ignore]`d arm is declared in [`IGNORED_ARM_INVENTORY`] with a class
/// drawn from [`IGNORE_CLASSES`], and every declared arm still exists.
///
/// Both directions, for the same reason the fence gate needs both. An arm added
/// without an entry is the gap this closes — `#[ignore]` is silent, so nothing
/// else would ever ask who runs it. An entry whose arm is gone is the more
/// insidious half: the inventory would read as though a machine somewhere still
/// owes that run.
///
/// The CLASS is checked against a closed set rather than left as free text,
/// because a length-checked rationale is not a contract — any long-enough
/// string satisfied it, so an arm could be misclassified, or effectively
/// unowned, and still read as declared.
#[test]
fn every_ignored_arm_declares_its_class_and_owning_lane() {
    let tests_dir = repo_root()
        .join("crates")
        .join("cerulion_core")
        .join("tests");
    let mut actual: BTreeSet<(String, String)> = BTreeSet::new();
    ignored_arms_under(&tests_dir, "", &mut actual);

    assert!(
        !actual.is_empty(),
        "no #[ignore]d arm found anywhere under {} — the walk has broken, and every assertion \
         below would be vacuous",
        tests_dir.display()
    );

    let known: BTreeSet<&str> = IGNORE_CLASSES.iter().map(|(c, _)| *c).collect();
    assert_eq!(
        IGNORE_CLASSES.len(),
        known.len(),
        "IGNORE_CLASSES names a class twice"
    );

    let declared: BTreeSet<(String, String)> = IGNORED_ARM_INVENTORY
        .iter()
        .map(|(f, a, _, _)| (f.to_string(), a.to_string()))
        .collect();
    assert_eq!(
        IGNORED_ARM_INVENTORY.len(),
        declared.len(),
        "IGNORED_ARM_INVENTORY lists the same arm twice — a duplicate makes the set comparison \
         below pass while the reasons disagree"
    );
    for (f, a, class, why) in IGNORED_ARM_INVENTORY {
        assert!(
            known.contains(class),
            "{f}::{a} declares class {class:?}, which is not in IGNORE_CLASSES. The class is what \
             names the owning lane, so an unknown one leaves the arm unowned. Known: {known:?}"
        );
        assert!(
            why.len() > 20,
            "IGNORED_ARM_INVENTORY's reason for `{f}::{a}` is too thin to review: {why:?}"
        );
    }

    let undeclared: Vec<&(String, String)> = actual.difference(&declared).collect();
    assert!(
        undeclared.is_empty(),
        "these #[ignore]d arms are declared nowhere: {undeclared:?}.\n\nFIX: add each to \
         IGNORED_ARM_INVENTORY with a CLASS from IGNORE_CLASSES — the class names the lane that \
         owns running it. If no lane can, say so with the owed/on-demand class: `#[ignore]` with \
         no owner is a test nobody runs and nobody knows nobody runs."
    );

    let vanished: Vec<&(String, String)> = declared.difference(&actual).collect();
    assert!(
        vanished.is_empty(),
        "IGNORED_ARM_INVENTORY declares arms that no longer carry #[ignore]: {vanished:?}.\n\n\
         This is the dangerous direction — the inventory reads as though a machine still owes \
         those runs.\n\nFIX: drop the entry if the arm was deleted, rename it if the arm was \
         renamed, or remove the entry if the arm now runs by default."
    );
}

/// The fence in `.config/nextest.toml` must be EXACTLY the declared inventory.
///
/// The gate above asks "is every singleton caller fenced?"; this one asks the
/// other direction, "is everything in the fence something we meant to put
/// there?". Both are needed. A filter is not reviewable by eye — the shipped
/// one is thirty `binary(=…)` terms on one line — so an entry added by a
/// copy-paste, or lost by an edit, would otherwise pass unread.
#[test]
fn the_fence_membership_equals_the_declared_inventory() {
    let root = repo_root();
    let cfg = fs::read_to_string(root.join(FENCE_FILE))
        .unwrap_or_else(|e| panic!("read {FENCE_FILE}: {e}"));

    for (label, actual, declared) in [
        (
            FENCE_GROUP,
            fence_binaries(&cfg, FENCE_GROUP),
            FENCE_INVENTORY,
        ),
        (
            "threads-required = num-cpus",
            threads_required_binaries(&cfg),
            TIMING_INVENTORY,
        ),
    ] {
        let want: BTreeSet<String> = declared.iter().map(|(b, _)| b.to_string()).collect();
        assert_eq!(
            declared.len(),
            want.len(),
            "`{label}`'s inventory lists a binary twice — a duplicate makes the set comparison \
             below pass while the reasons disagree"
        );
        assert!(
            !actual.is_empty(),
            "`{label}` names no binary in {FENCE_FILE} — the override block is gone or the \
             reader has broken, and this assertion would be vacuous"
        );

        let extra: Vec<&String> = actual.difference(&want).collect();
        assert!(
            extra.is_empty(),
            "`{label}` in {FENCE_FILE} fences binaries the inventory does not declare: \
             {extra:?}.\n\nFIX: add each to the inventory in this file WITH THE REASON it \
             cannot run beside a sibling, or remove it from the config. A fence entry nobody \
             can justify is one somebody will delete."
        );
        let missing: Vec<&String> = want.difference(&actual).collect();
        assert!(
            missing.is_empty(),
            "the inventory declares binaries `{label}` in {FENCE_FILE} does NOT fence: \
             {missing:?}.\n\nThis is the dangerous direction — the inventory reads as though \
             those binaries are protected while the config leaves them free to run beside each \
             other.\n\nFIX: restore the `binary(=name)` term, or drop the inventory line if the \
             file genuinely no longer needs fencing."
        );
    }
}

// ===========================================================================
// The DOCTEST half.
//
// `cargo test --doc` COMPILES AND RUNS doctests, and it runs them in parallel:
// each is its own binary, and libtest threads them. So a doctest that calls a
// singleton accessor is a default-namespace caller with no `#[serial]` to put
// on it and no fence to add it to — the two mechanisms this file is otherwise
// built around are both unavailable, because a doctest is not a test binary.
//
// It is also the one caller class the main gate above is STRUCTURALLY blind to:
// that walk reads `code_only`, which strips doc comments, so a doctest's body
// is invisible to it BY CONSTRUCTION. The module header already places `src/`
// out of scope for the `#[serial]` rule; this is the part of `src/` that runs
// anyway.
//
// The remedy is `no_run`: the doctest still COMPILES (so it cannot rot), it
// just does not execute. Zero offenders exist today, which is what makes this a
// pure regression guard — and why it ships with a PLANTED offender proving it
// bites and a clean-corpus control proving the walk runs at all.
//
// SCOPE, deliberately narrow. The cdylib `NODES` registry,
// a counting allocator and env mutation are also
// process-globals a doctest could reach. Only the SINGLETON is watched here,
// for two reasons: the singleton is this gate's subject (the "forbidden
// constructors"), and the others are either unreachable from a doctest (a
// `#[global_allocator]` cannot be installed by one) or would need a token list
// broad enough to flag a doctest that merely DOCUMENTS the API — `set_var`
// being the obvious example. Watching a token because it is cheap, on a corpus
// with no offenders, is how a gate earns a reputation for false alarms. If one
// of those ever appears, the list to extend is `SINGLETON_CALLS` and the walk
// below needs no other change.
// ===========================================================================

/// A ```` ``` ```` fence info string that means "this block never executes".
///
/// `no_run` compiles but does not run. `ignore` is not even compiled. `text`
/// (and any non-Rust language tag) is prose. `compile_fail` is expected not to
/// build, so it cannot reach a `main`. Anything else — a bare fence, `rust`,
/// `should_panic`, `edition2021` — RUNS.
fn fence_never_executes(info: &str) -> bool {
    // Attributes are comma- or space-separated; `rust,no_run` is common.
    let tokens: Vec<&str> = info
        .split(|c: char| c == ',' || c.is_whitespace())
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .collect();
    if tokens
        .iter()
        .any(|t| matches!(*t, "no_run" | "ignore" | "compile_fail") || t.starts_with("ignore-"))
    {
        return true;
    }
    // A language tag that is not Rust is prose, not a doctest. An EMPTY info
    // string means Rust, so it executes.
    if tokens.is_empty() {
        return false;
    }
    let known_rust = [
        "rust",
        "should_panic",
        "edition2015",
        "edition2018",
        "edition2021",
        "edition2024",
    ];
    !tokens
        .iter()
        .any(|t| known_rust.contains(t) || t.starts_with("edition"))
}

/// Every EXECUTING doctest body in `src`, as `(line_number, body)`.
///
/// Reads the raw file: doc comments are exactly what `code_only` removes, so
/// this walk cannot use it. Both doc syntaxes are read (`///` outer and `//!`
/// inner), and a hidden line (`# ` inside a doctest) is KEPT — hidden lines
/// execute like any other, which is precisely where a setup call would sit.
///
/// KNOWN BLIND SPOTS, all false-NEGATIVE (a missed doctest, never a fabricated
/// one), and all verified absent from the tree today — the reader's output was
/// cross-checked against rustdoc's own classification and matches it exactly:
///
/// * a four-backtick fence with an empty info string is an executing Rust
///   doctest to rustdoc; here the extra backtick reads as a language tag and
///   the block is skipped;
/// * a 4-space-INDENTED code block in a doc comment is a doctest to rustdoc
///   and is invisible to a fence-only reader;
/// * `#[doc = include_str!(…)]` pulls doctests in from another file entirely
///   (grepped: the tree has none).
///
/// Each is worth widening the reader for if one ever appears; none is worth
/// speculative machinery now, and a missed doctest fails toward silence rather
/// than toward a false accusation.
fn executing_doctests(src: &str) -> Vec<(usize, String)> {
    // THREE states, not two. A two-state version (`Option<block>`) reads the
    // CLOSING fence of a skipped block as an OPENING one, because both are bare
    // ```` ``` ```` and both are seen with no block open — so every
    // ```` ```compile_fail ```` / ```` ```text ```` block in the repo minted a
    // phantom "executing" doctest starting at its closing fence. MEASURED
    // against rustdoc's own classification: that bug reported 6 executing
    // doctests in `cerulion_core` where rustdoc runs 2.
    enum St {
        Outside,
        Collecting(usize, String),
        Skipping,
    }
    let mut out = Vec::new();
    let mut st = St::Outside;
    for (idx, raw) in src.lines().enumerate() {
        let t = raw.trim_start();
        let Some(rest) = t.strip_prefix("///").or_else(|| t.strip_prefix("//!")) else {
            // A doc block cannot span a non-doc line; an unterminated one ends
            // here rather than swallowing the rest of the file.
            st = St::Outside;
            continue;
        };
        let body = rest.strip_prefix(' ').unwrap_or(rest);
        let fence = body.trim_start();
        if let Some(info) = fence.strip_prefix("```") {
            st = match st {
                // Closing fence of a block we were collecting.
                St::Collecting(start, text) => {
                    out.push((start, text));
                    St::Outside
                }
                // Closing fence of a block we deliberately skipped — NOT an
                // opening fence.
                St::Skipping => St::Outside,
                St::Outside => {
                    if fence_never_executes(info) {
                        St::Skipping
                    } else {
                        St::Collecting(idx + 1, String::new())
                    }
                }
            };
            continue;
        }
        if let St::Collecting(_, text) = &mut st {
            // `# ` marks a HIDDEN line — hidden from rustdoc's rendering, but
            // compiled and run. Strip only the marker.
            let line = body.strip_prefix("# ").unwrap_or(body);
            text.push_str(line);
            text.push('\n');
        }
    }
    // An unterminated block at EOF is still a block.
    if let St::Collecting(start, text) = st {
        out.push((start, text));
    }
    out
}

#[test]
fn the_doctest_reader_finds_only_the_blocks_that_execute() {
    // A bare fence executes.
    let bare = "/// ```\n/// let x = 1;\n/// ```\npub fn f() {}\n";
    assert_eq!(executing_doctests(bare).len(), 1);
    assert!(executing_doctests(bare)[0].1.contains("let x = 1;"));

    // `no_run` / `ignore` / `compile_fail` do not.
    for tag in ["no_run", "ignore", "compile_fail", "rust,no_run", "text"] {
        let s = format!("/// ```{tag}\n/// CALL_ME();\n/// ```\npub fn f() {{}}\n");
        assert!(
            executing_doctests(&s).is_empty(),
            "```{tag} must not be read as executing"
        );
    }
    // …but `rust` and `should_panic` DO.
    for tag in ["rust", "should_panic", "edition2021"] {
        let s = format!("/// ```{tag}\n/// CALL_ME();\n/// ```\npub fn f() {{}}\n");
        assert_eq!(
            executing_doctests(&s).len(),
            1,
            "```{tag} executes and must be read"
        );
    }

    // A HIDDEN line executes and must be part of the body — the exact place a
    // setup call hides.
    let hidden = "/// ```\n/// # CALL_ME();\n/// let x = 1;\n/// ```\npub fn f() {}\n";
    assert!(executing_doctests(hidden)[0].1.contains("CALL_ME()"));

    // Inner doc comments (`//!`) are read too.
    let inner = "//! ```\n//! CALL_ME();\n//! ```\n";
    assert_eq!(executing_doctests(inner).len(), 1);

    // Ordinary code between two doctests does not merge them, and a `#[test]`
    // fn body is NOT a doctest.
    let two =
        "/// ```\n/// A();\n/// ```\npub fn f() {}\n/// ```\n/// B();\n/// ```\npub fn g() {}\n";
    let got = executing_doctests(two);
    assert_eq!(got.len(), 2);
    assert!(got[0].1.contains("A()") && !got[0].1.contains("B()"));

    // A plain (non-doc) comment holding a fence is not a doctest.
    assert!(executing_doctests("// ```\n// CALL_ME();\n// ```\n").is_empty());

    // THE BUG THIS READER SHIPPED WITH, caught by cross-checking against
    // rustdoc rather than by review: the CLOSING fence of a skipped block is
    // bare, so a two-state reader saw "a fence, with no block open" and OPENED
    // one — turning the prose AFTER every `compile_fail` / `text` block into a
    // phantom executing doctest. `cerulion_core/src/state.rs` has three
    // `compile_fail` blocks in a row and produced three phantoms.
    let after_skipped = concat!(
        "//! ```compile_fail\n",
        "//! needs_state::<Instant>();\n",
        "//! ```\n",
        "//!\n",
        "//! Ordinary prose that mentions CALL_ME but is not code.\n",
    );
    assert!(
        executing_doctests(after_skipped).is_empty(),
        "the closing fence of a skipped block must not open a new one; got {:?}",
        executing_doctests(after_skipped)
    );

    // …and a REAL doctest following a skipped one is still found, so the fix
    // is not "stop reading after the first skip".
    let skipped_then_real = concat!(
        "//! ```text\n",
        "//! a diagram\n",
        "//! ```\n",
        "//! ```\n",
        "//! REAL_CALL();\n",
        "//! ```\n",
    );
    let got = executing_doctests(skipped_then_real);
    assert_eq!(got.len(), 1, "got {got:?}");
    assert!(got[0].1.contains("REAL_CALL()"));

    // Three skipped blocks in a row (the state.rs shape) still yield nothing.
    let three = "//! ```compile_fail\n//! a\n//! ```\n//! ```compile_fail\n//! b\n//! ```\n//! ```compile_fail\n//! c\n//! ```\n";
    assert!(executing_doctests(three).is_empty());
}

// ===========================================================================
// THE SECOND NAME PLANE — POSIX SHM.
//
// Everything above this point is about ONE machine-global namespace: the
// DEFAULT iceoryx2 SHM root. There is a SECOND, and it bit before this gate
// existed. A doorbell's backing object is `/cer_db_{ns}_{fnv(topic)}` and a
// barrier's is `/cer_bar_{fnv(ns/topic)}` — pure functions of the namespace
// string, carrying no pid and no randomness — so two PROCESSES that pass the
// same `ns` map the same `/dev/shm` page no matter how carefully isolated
// their iceoryx2 roots are.
//
// Under `cargo test -- --test-threads=1` that was unreachable: one binary is
// one process running its tests in sequence. Under nextest each test is its
// own process and they run CONCURRENTLY, which is how
// `monitor_wait_park_iox2_test`'s Linux-only doorbell RINGER (ringing every
// 200 us) came to share a page with the sibling test that asserts the doorbell
// counter is ZERO: 2 CI failures out of 2, byte-identical, never reproducible
// on macOS — where the ring is a stub and the ringer is `cfg`-compiled out.
//
// The repo already knew the rule and wrote it down TWICE — `doorbell.rs`'s own
// `test_ns` and `barrier_park_wake_iox2_test.rs`'s `barrier_ns`, both
// pid-scoped, both explaining why in a comment. What it did not have was
// anything that NOTICED a third site not following it. This gate found four
// more the day it was written.
//
// SCOPE: a package's own `tests/` tree, matching the `#[serial]` rule's scope
// above. `#[cfg(test)] mod tests` inside `src/` is NOT walked — `doorbell.rs`'s
// `test_ns` lives there and is already correct, but a new `src/` unit test
// could hardcode a namespace unseen. Closing that needs the same
// production-vs-test region split the module header defers, and the risk is
// lower: a unit test beside the implementation is written by someone looking at
// `test_ns` on the screen.
// ===========================================================================

/// The constructors that take a POSIX-SHM NAMESPACE, with the ZERO-BASED index
/// of the argument that carries it.
///
/// `MonitorWaitPolicy::new(park, doorbell, ns)` is the odd one: its namespace is
/// the THIRD argument, and it reaches the doorbell only indirectly (the runtime
/// passes `policy.ns()` to `enable_doorbell` / `DoorbellRegistry::open`). It is
/// watched anyway, and that is the point — the collision that motivated this
/// gate was between a `MonitorWaitPolicy` ns and a `Doorbell::open_owned` ns,
/// the same string reaching the same page by two different roads.
/// Kept complete by [`the_shm_constructor_inventory_covers_every_ns_taking_door`],
/// which DERIVES the set from `doorbell.rs`, `barrier.rs` and `credit.rs`
/// rather than trusting this list — `Doorbell::open_unowned` was missing from
/// the first version of this gate, and the credit word's `MappedCredit` pair was
/// missing from the second: a hand list develops exactly this hole, once per
/// new SHM family, and the derivation is what closes it without anyone
/// remembering to.
const SHM_NS_CONSTRUCTORS: &[(&str, usize)] = &[
    ("MonitorWaitPolicy::new(", 2),
    ("Doorbell::open_owned(", 0),
    ("Doorbell::open_unowned(", 0),
    ("DoorbellRegistry::open(", 0),
    ("MappedBarrier::create_owned(", 0),
    ("MappedBarrier::open_unowned(", 0),
    ("MappedCredit::create_owned(", 0),
    ("MappedCredit::open_unowned(", 0),
];

/// Namespace-taking `pub fn`s that DERIVE A NAME and open nothing — excluded
/// from the constructor inventory above, each with the reason.
///
/// The distinction the gate cares about is whether a call can NAME A PAGE INTO
/// EXISTENCE. A deriver returns a `String`: the doors it names are the ones
/// already watched, and a test that derives a name and then opens the object by
/// hand does so through `shm_open`, which this gate cannot see by construction
/// (and which `credit_test.rs`'s `RawSegment` fixture does deliberately, to
/// craft the hostile shapes `open_unowned` must refuse).
///
/// Gated in BOTH directions: an unwatched non-deriver fails above, and an entry
/// here that the derivation no longer finds fails too.
///
/// The key is `<nearest impl above>::<fn name>`, NOT `<free>::…`: the
/// derivation attributes a free `pub fn` to the last `impl` header preceding
/// it, so spelling this entry the way the function is DECLARED would leave it
/// silently unmatched — and the two-way gate would then fail on a stale entry
/// rather than on the thing it watches.
const SHM_NS_NAME_DERIVERS: &[(&str, &str)] = &[(
    "ParkedEdgeGuard::credit_shm_name",
    "pure name derivation — takes (ns, id), returns the POSIX object name as a String, maps \
     nothing. `pub` so an out-of-crate test can craft the hostile segment shapes \
     `MappedCredit::open_unowned` refuses without a second copy of the recipe.",
)];

/// Byte offset just PAST the last `impl` keyword in `code`, as a whole word.
///
/// A plain `rfind("impl ")` misses `impl<'a> T {` and then attributes that
/// block's functions to whatever type the PREVIOUS `impl` named.
fn last_impl_header(code: &str) -> Option<usize> {
    let mut from = code.len();
    while let Some(i) = code[..from].rfind("impl") {
        let before_ok = i == 0
            || !code[..i]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_alphanumeric() || c == '_');
        let after = &code[i + "impl".len()..];
        let after_ok = after.starts_with(' ') || after.starts_with('<');
        if before_ok && after_ok {
            return Some(i + "impl".len());
        }
        from = i;
    }
    None
}

/// The type an `impl` header names, as the CALLER spells it.
///
/// `impl Doorbell {` and `impl Drop for Doorbell {` both answer `Doorbell`;
/// generics on the `impl` itself are skipped. Text-level, which is all this
/// walk needs — the two modules it reads carry plain inherent impls.
fn impl_header_type(after_impl: &str) -> Option<String> {
    /// Consume a balanced `<..>` group at the head of `s`, if present.
    fn skip_generics(s: &str) -> &str {
        if !s.starts_with('<') {
            return s;
        }
        let mut depth = 0usize;
        for (i, c) in s.char_indices() {
            match c {
                '<' => depth += 1,
                '>' => {
                    depth -= 1;
                    if depth == 0 {
                        return s[i + 1..].trim_start();
                    }
                }
                _ => {}
            }
        }
        s
    }
    /// Consume one path (`std::ops::Deref`), returning `(whole, last segment)`.
    fn path(s: &str) -> (usize, String) {
        let taken: String = s
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_' || *c == ':')
            .collect();
        let last = taken.rsplit("::").next().unwrap_or("").to_string();
        (taken.len(), last)
    }

    // `impl<'a, T>` — the impl's own generics come before the type.
    let rest = skip_generics(after_impl.trim_start());
    let (len, first) = path(rest);
    if first.is_empty() {
        return None;
    }
    // `impl Trait for Type` — the TYPE is what a caller names, and the trait
    // may be a PATH and may carry its own generic arguments.
    let tail = skip_generics(rest[len..].trim_start());
    if let Some(after_for) = tail.strip_prefix("for ") {
        let (_, ty) = path(after_for.trim_start());
        return (!ty.is_empty()).then_some(ty);
    }
    Some(first)
}

/// Every `pub fn` in `src` whose FIRST parameter is a namespace (`ns: &str`),
/// reported TYPE-QUALIFIED as a caller would spell it (`Doorbell::open_unowned`).
///
/// Text-derived so a new door cannot arrive unwatched. **Qualification is what
/// makes that true**: the first version of this helper returned bare names, so
/// `MappedBarrier::open_unowned` covered `Doorbell::open_unowned` and dropping
/// the latter from the inventory left the guard GREEN — the exact hole this
/// derivation exists to close.
///
/// `MonitorWaitPolicy::new` is deliberately outside the derivation — its
/// namespace is the THIRD argument and it reaches a doorbell only indirectly,
/// so it is inventoried by hand and the assertion below says so.
fn ns_taking_fns(src: &str) -> BTreeSet<String> {
    let code = code_only(src);
    let mut out = BTreeSet::new();
    let mut from = 0usize;
    while let Some(at) = code[from..].find("pub fn ") {
        let start = from + at + "pub fn ".len();
        let name: String = code[start..]
            .chars()
            .take_while(|c| c.is_alphanumeric() || *c == '_')
            .collect();
        let rest = &code[start + name.len()..];
        if let Some(open) = rest.find('(') {
            let args = &rest[open + 1..];
            let first = args.split(',').next().unwrap_or("");
            if first.replace(' ', "").starts_with("ns:&str") {
                // The nearest `impl` header ABOVE this fn owns it. Matched as a
                // WHOLE WORD followed by a space or `<`, because `impl<'a> Y {`
                // carries no space and a `"impl "` search silently walks past it
                // onto the previous block's type (measured: `Y::open_unowned`
                // was reported as `X::open_unowned`).
                let ty = last_impl_header(&code[..start])
                    .and_then(|i| impl_header_type(&code[i..]))
                    .unwrap_or_else(|| "<free>".to_string());
                out.insert(format!("{ty}::{name}"));
            }
        }
        from = start;
    }
    out
}

/// The inventory must cover every namespace-taking door the code actually has.
#[test]
fn the_shm_constructor_inventory_covers_every_ns_taking_door() {
    let root = repo_root();
    let watched: String = SHM_NS_CONSTRUCTORS
        .iter()
        .map(|(c, _)| *c)
        .collect::<Vec<_>>()
        .join(" ");

    let mut derived: BTreeSet<String> = BTreeSet::new();
    for rel in [
        "crates/cerulion_core/src/doorbell.rs",
        "crates/cerulion_core/src/barrier.rs",
        // The credit word is the THIRD family to take a namespace,
        // and `/cer_crd_{ns}_{fnv(id)}` has no pid in it either.
        "crates/cerulion_core/src/credit.rs",
    ] {
        let src = fs::read_to_string(root.join(rel)).unwrap_or_else(|e| panic!("read {rel}: {e}"));
        derived.extend(ns_taking_fns(&src));
    }

    // Vacuity guard: the derivation must find the doors everybody knows about,
    // QUALIFIED — two types share the name `open_unowned`, and a bare-name
    // derivation lets either one cover the other (measured).
    for known in [
        "Doorbell::open_owned",
        "Doorbell::open_unowned",
        "MappedBarrier::create_owned",
        "MappedBarrier::open_unowned",
        "MappedCredit::create_owned",
        "MappedCredit::open_unowned",
    ] {
        assert!(
            derived.contains(known),
            "the ns-taking derivation found {derived:?} and not even `{known}` — it has gone \
             stale and every claim below is vacuous"
        );
    }

    // A DERIVER is not a door: it takes a namespace and returns a String,
    // mapping nothing. Excluding it is what keeps this gate about the calls
    // that can actually name a page into existence — and the exclusion is
    // itself gated below, so it cannot quietly grow.
    let derivers: BTreeSet<&str> = SHM_NS_NAME_DERIVERS.iter().map(|(n, _)| *n).collect();
    let unwatched: Vec<&String> = derived
        .iter()
        .filter(|n| !watched.contains(&format!("{n}(")) && !derivers.contains(n.as_str()))
        .collect();
    assert!(
        unwatched.is_empty(),
        "`doorbell.rs`/`barrier.rs`/`credit.rs` expose namespace-taking constructor(s) this \
         gate does not watch: {unwatched:?}. A test calling one names a `/dev/shm` page under a \
         namespace nothing checks.\n\nFIX: add `<Type>::<name>(` to `SHM_NS_CONSTRUCTORS` with \
         the zero-based index of its namespace argument — or, if it only DERIVES a name and \
         opens nothing, declare it in `SHM_NS_NAME_DERIVERS` with that reason."
    );

    // The exclusion list must stay live: an entry naming a fn the derivation no
    // longer finds is a pre-authorised hole, exactly like a stale exemption.
    let stale_derivers: Vec<&str> = SHM_NS_NAME_DERIVERS
        .iter()
        .map(|(n, _)| *n)
        .filter(|n| !derived.contains(*n))
        .collect();
    assert!(
        stale_derivers.is_empty(),
        "`SHM_NS_NAME_DERIVERS` names fn(s) the derivation no longer finds: {stale_derivers:?}. \
         Delete them — an exclusion for something that does not exist pre-authorises the next \
         thing that takes the name."
    );
}

#[test]
fn the_ns_taking_derivation_reads_the_first_parameter_only() {
    let src = "\
impl X {
    pub fn open_owned(ns: &str, topic: &str) -> R { }
    pub fn ring(&self) { }
    pub fn seq(&self) -> u64 { }
    pub fn open_unowned(ns : &str, t: &str) -> R { }
    pub fn not_first(topic: &str, ns: &str) -> R { }
    fn private_door(ns: &str) -> R { }
}
impl Drop for X {
    fn drop(&mut self) { }
}
impl<'a> Y {
    pub fn open_unowned(ns: &str, id: &str) -> R { }
}
";
    // EXACT set, not a bag of `contains` probes — the property under test is
    // what the derivation reports and what it must NOT, and a set equality says
    // both at once. In order of what each element pins:
    //
    //  * `X::open_owned`   — the plain shape.
    //  * `X::open_unowned` — whitespace in the parameter must not hide it.
    //  * `Y::open_unowned` — THE QUALIFICATION PIN. Two types share one fn NAME
    //    and both must be reported. A bare-name derivation answers
    //    `{open_unowned}` once, which is exactly what let
    //    `Doorbell::open_unowned` be dropped from the inventory with the
    //    coverage guard still GREEN (measured), and `Y` additionally sits behind
    //    an `impl<'a>` header with no space, which a `"impl "` search walks past
    //    and mis-attributes to `X`.
    //  * `not_first` ABSENT — `ns` in a LATER position is a different shape,
    //    inventoried by hand (that is `MonitorWaitPolicy::new`).
    //  * `private_door` ABSENT — only PUBLIC doors are callable from a test.
    //  * `ring`/`seq`/`drop` ABSENT — they take no namespace.
    let got = ns_taking_fns(src);
    let want: BTreeSet<String> = ["X::open_owned", "X::open_unowned", "Y::open_unowned"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    assert_eq!(got, want);
}

#[test]
fn the_impl_header_reader_names_the_type_a_caller_would_write() {
    // Hand vectors: the header shapes the two walked modules actually carry,
    // plus the two that would mis-attribute a fn if parsed naively.
    assert_eq!(impl_header_type("Doorbell {").as_deref(), Some("Doorbell"));
    assert_eq!(
        impl_header_type("Drop for Doorbell {").as_deref(),
        Some("Doorbell")
    );
    assert_eq!(
        impl_header_type("std::ops::Deref for MappedBarrier {").as_deref(),
        Some("MappedBarrier")
    );
    assert_eq!(
        impl_header_type("<'a> ParkedRankGuard<'a> {").as_deref(),
        Some("ParkedRankGuard")
    );
    assert_eq!(impl_header_type("{").as_deref(), None);
}

/// A namespace EXPRESSION that is not pid-scoped, and why that is safe.
///
/// Keyed by the expression as it appears, optionally FILE-SCOPED as
/// `<file stem>:<expr>`. Two shapes need declaring: a bare LITERAL, and an
/// expression this walk resolves to nothing (a function PARAMETER, or a value
/// handed to a SUBPROCESS through the environment). File scoping matters for
/// the second — a bare identifier like `ns` is far too common to clear globally.
///
/// The exemption is granted on EVIDENCE, not on a promise, and the gate CHECKS
/// what it can: a declared entry that no call site uses is STALE and fails (a
/// dead exemption pre-authorises a future collision), and a declared LITERAL
/// must appear at exactly ONE call site across the whole walked corpus. A
/// collision needs two live users of one name, so a genuinely unique literal
/// cannot collide — and the moment somebody adds a second use, the count
/// changes and the exemption stops applying.
///
/// Deliberately weaker than pid-scoping, and it should not grow: a unique
/// literal is still not safe against the same test running twice at once (a
/// retry, two shards sharing one machine). Prefer a pid-scoped helper.
///
/// NOTE what is NOT here: a `MonitorWaitPolicy::new(.., doorbell = false, ..)`
/// namespace needs no entry at all. That is decided automatically from the
/// call's own second argument — see [`shm_ns_sites`] — because a policy with the
/// doorbell off never opens a doorbell, so its namespace names no page. An
/// automatic rule cannot rot the way a list of five files would.
const SHM_NS_EXEMPTIONS: &[(&str, &str)] = &[
    (
        "\"pvl\"",
        "polled_vs_live_iox2_test: doorbell ON, but ONE call site inside ONE test, so no sibling \
         opens this page",
    ),
    (
        "\"ext_bar\"",
        "non_trigger_hold_iox2_test: one barrier, one call site, distinct from its sibling",
    ),
    (
        "\"sib_bar\"",
        "non_trigger_hold_iox2_test: one barrier, one call site, distinct from its sibling",
    ),
    (
        "non_trigger_hold_iox2_test:barrier_ns",
        "a function PARAMETER, so the namespace is chosen by each caller and judged at those \
         call sites, not here",
    ),
    (
        "credit_death_e2e_test:ns",
        "the namespace is READ, never composed: the arms parse `ns=` off the supervisor's own \
         `credit words created` log line and map a page the PRODUCTION run minted (the \
         supervisor pid-scopes it as `cerdep_{graph}_{pid}_{nonce}`). A test-side \
         `process::id()` would name a page nobody created — the peer-mapping case the \
         `barrier_level_gate_subprocess_iox2_test:ns` entry below exempts for the same reason",
    ),
    (
        "barrier_level_gate_subprocess_iox2_test:ns",
        "the value IS pid-scoped — `barrier_ns(tag)` = `barrier_subproc_{pid}_{tag}` — but it \
         reaches these sites through an environment variable into a SUBPROCESS, which no \
         single-file resolver can follow",
    ),
    (
        "rmw_wait_event_test:shared_publisher_ns",
        "DELIBERATELY the shared production namespace: the site unlinks THE doorbell page the \
         rmw publisher armed under `default_namespace()` (modelling an owned creator's \
         lifecycle for the stale-bell re-map pin) — a pid-scoped namespace would unlink a \
         page nobody armed and the pin would never fire. Cross-process page uniqueness rides \
         the TOPIC instead: the file's `unique_suffix()` embeds `process::id()`, so the \
         `/cer_db_{ns}_{fnv(topic)}` name is pid-scoped through its topic term",
    ),
];

/// Split a call's argument list at TOP-LEVEL commas.
///
/// `text` starts just past the opening paren. Nested calls, generics, closures
/// and string literals all carry commas that are not argument separators, so a
/// plain `split(',')` picks the wrong argument — and picking the wrong argument
/// means reading a topic name where a namespace belongs, which fails OPEN.
fn split_call_args(text: &str) -> Option<Vec<String>> {
    let ch: Vec<char> = text.chars().collect();
    let (mut depth, mut i) = (0i32, 0usize);
    let (mut in_str, mut esc) = (false, false);
    let mut args: Vec<String> = Vec::new();
    let mut cur = String::new();
    while i < ch.len() {
        let c = ch[i];
        if in_str {
            if esc {
                esc = false;
            } else if c == '\\' {
                esc = true;
            } else if c == '"' {
                in_str = false;
            }
            cur.push(c);
            i += 1;
            continue;
        }
        match c {
            '"' => {
                in_str = true;
                cur.push(c);
            }
            '(' | '[' | '<' | '{' => {
                depth += 1;
                cur.push(c);
            }
            ')' if depth == 0 => {
                args.push(cur.trim().to_string());
                return Some(args);
            }
            ')' | ']' | '>' | '}' => {
                depth -= 1;
                cur.push(c);
            }
            ',' if depth == 0 => {
                args.push(cur.trim().to_string());
                cur.clear();
            }
            _ => cur.push(c),
        }
        i += 1;
    }
    // Unterminated: fail CLOSED (the caller reports rather than guessing).
    None
}

/// Every POSIX-SHM namespace ARGUMENT in `src`, as `(line, expression)`.
///
/// TWO VIEWS, and needing both is not incidental. Call sites are LOCATED in the
/// literal-BLANKED view, because a constructor named inside a string is not a
/// call — this very file holds all five of them in its oracle vectors and in
/// `SHM_NS_CONSTRUCTORS`, so a single-view walk reports the gate itself and the
/// only way out is a self-exclusion that then blinds it to its own tree. The
/// namespace is then READ from the literal-PRESERVING view, because a bare
/// literal namespace is exactly what the exemption table identifies by value.
///
/// The two views agree CHARACTER FOR CHARACTER (`code_only_blank_literals`
/// blanks content in place, and its own oracle asserts the equal char count), so
/// an index found in one addresses the same token in the other. They do NOT
/// agree on BYTE offsets — a multi-byte character inside a literal blanks to a
/// one-byte space — so every index here is a CHAR index.
fn shm_ns_sites(src: &str) -> Vec<(usize, String)> {
    let code: Vec<char> = code_only(src).chars().collect();
    let mask: Vec<char> = code_only_blank_literals(src).chars().collect();
    debug_assert_eq!(code.len(), mask.len(), "the two views must align");
    let mut out = Vec::new();
    for (ctor, idx) in SHM_NS_CONSTRUCTORS {
        let pat: Vec<char> = ctor.chars().collect();
        let mut i = 0usize;
        while i + pat.len() <= mask.len() {
            if mask[i..i + pat.len()] != pat[..] {
                i += 1;
                continue;
            }
            let open = i + pat.len();
            let line = mask[..open].iter().filter(|c| **c == '\n').count() + 1;
            let rest: String = code[open..].iter().collect();
            match split_call_args(&rest) {
                Some(args) => {
                    // A `MonitorWaitPolicy` with the DOORBELL OFF opens no
                    // doorbell, so its namespace names no `/dev/shm` object and
                    // cannot collide with anything. Decided from the call's own
                    // argument rather than from a list of files, because the
                    // list is what goes stale.
                    let doorbell_off = *ctor == "MonitorWaitPolicy::new("
                        && args.get(1).map(|a| a.trim()) == Some("false");
                    if !doorbell_off {
                        if let Some(a) = args.get(*idx) {
                            out.push((line, a.clone()));
                        }
                    }
                }
                // An unparseable call list is REPORTED, not skipped — a silent
                // skip is how a namespace slips past a gate that looks green.
                None => out.push((line, format!("<unparseable {ctor} argument list>"))),
            }
            i = open;
        }
    }
    out.sort();
    out
}

/// A namespace expression reduced to the part that IDENTIFIES the namespace.
///
/// Strips a leading borrow and the conversion tails these constructors are
/// called through, so one literal has ONE key however the call site spells it.
/// The `MonitorWaitPolicy` sites say `"pvl".into()` while a barrier takes a bare
/// `"ext_bar"`; keying on raw text makes an exemption read STALE while
/// its call site sits right there, which points the reader at the wrong problem.
fn normalise_ns_expr(expr: &str) -> String {
    let mut s = expr.trim().trim_start_matches('&').trim();
    loop {
        let before = s;
        for tail in [".into()", ".to_string()", ".as_str()", ".to_owned()"] {
            s = s.strip_suffix(tail).unwrap_or(s).trim();
        }
        if s == before {
            return s.to_string();
        }
    }
}

/// Does `expr` resolve to something PID-SCOPED, in this file?
///
/// Resolved ONE level, deliberately: an inline `process::id()`, a call to a
/// local `fn` whose body has one, or a `let` binding whose right-hand side does
/// (directly or through such a call). Deeper chains are REPORTED rather than
/// followed — an unresolvable namespace is exactly what needs a closer look,
/// and a walk that guesses is worse than one that asks.
fn ns_is_pid_scoped(src: &str, expr: &str) -> bool {
    // LITERAL-BLANKED, not `code_only`. This gate's whole job is to not be
    // fooled, and searching raw text for `process::id()` is fooled by the two
    // cheapest things in the world: a comment mentioning it, and a string
    // CONTAINING it. `fn mk() -> String { "process::id()".to_string() }` is
    // pid-scoped by textual inspection and fixed in fact — the exact shape that
    // would sail through while naming one shared page for every process.
    let code = code_only_blank_literals(src);
    if code_only_blank_literals(expr).contains("process::id()") {
        return true;
    }
    let head: String = expr
        .trim_start_matches('&')
        .chars()
        .take_while(|c| c.is_alphanumeric() || *c == '_')
        .collect();
    if head.is_empty() {
        return false;
    }
    // A local `fn head(..) { .. }` whose body is pid-scoped.
    if let Some(at) = code.find(&format!("fn {head}(")) {
        if let Some(open) = code[at..].find('{') {
            let body = &code[at + open..];
            let mut depth = 0i32;
            for (i, c) in body.char_indices() {
                if c == '{' {
                    depth += 1;
                } else if c == '}' {
                    depth -= 1;
                    if depth == 0 {
                        return body[..i].contains("process::id()");
                    }
                }
            }
        }
    }
    // A `let head = <expr>;` whose right-hand side is pid-scoped, following one
    // further hop through a local fn.
    if let Some(at) = code.find(&format!("let {head} =")) {
        let rest = &code[at..];
        let end = rest.find(';').unwrap_or(rest.len());
        let rhs = &rest[..end];
        if rhs.contains("process::id()") {
            return true;
        }
        for cand in rhs.split(|c: char| !(c.is_alphanumeric() || c == '_')) {
            if cand.is_empty() || cand == head {
                continue;
            }
            if code.contains(&format!("fn {cand}(")) && ns_is_pid_scoped(src, cand) {
                return true;
            }
        }
    }
    false
}

#[test]
fn the_shm_ns_reader_finds_the_namespace_argument_and_judges_its_scoping() {
    // The namespace is the THIRD argument of `MonitorWaitPolicy::new` and the
    // FIRST of the others. (Doorbell ON, or the automatic rule below skips it.)
    let policy = "MonitorWaitPolicy::new(true, true, mwp_ns(\"tag\"))";
    assert_eq!(shm_ns_sites(policy)[0].1, "mwp_ns(\"tag\")");
    let db = "Doorbell::open_owned(&ns_for(\"t\"), TOPIC)";
    assert_eq!(shm_ns_sites(db)[0].1, "&ns_for(\"t\")");

    // A nested call's commas are NOT argument separators — picking the wrong
    // argument reads a topic where a namespace belongs.
    let nested = "MonitorWaitPolicy::new(f(a, b), g(c, d), h(e, f))";
    assert_eq!(shm_ns_sites(nested)[0].1, "h(e, f)");
    // Nor are commas inside a string literal.
    let strlit = "MappedBarrier::create_owned(\"a,b\", \"g\", 1)";
    assert_eq!(shm_ns_sites(strlit)[0].1, "\"a,b\"");

    // DOORBELL OFF opens no doorbell, so the namespace names no page and is not
    // a site at all — decided from the call's own second argument.
    assert!(shm_ns_sites("MonitorWaitPolicy::new(true, false, \"fixed\")").is_empty());
    // …and the SAME namespace with the doorbell ON is very much a site.
    assert_eq!(
        shm_ns_sites("MonitorWaitPolicy::new(true, true, \"fixed\")")[0].1,
        "\"fixed\""
    );
    // The rule is `MonitorWaitPolicy`-specific: a barrier has no doorbell flag,
    // and its second argument must never be read as one.
    assert_eq!(
        shm_ns_sites("MappedBarrier::create_owned(\"n\", false, 1)")[0].1,
        "\"n\""
    );

    // A comment naming a constructor is not a call site.
    assert!(shm_ns_sites("// MonitorWaitPolicy::new(a, b, \"x\")\n").is_empty());
    // Nor is a constructor named inside a STRING — which is what this very file
    // does in its oracle vectors and its constant table, so a single-view walk
    // reports the gate itself.
    assert!(shm_ns_sites("let s = \"MappedBarrier::create_owned(\\\"x\\\", 1)\";").is_empty());
    // An unterminated argument list is REPORTED, never skipped.
    assert_eq!(shm_ns_sites("Doorbell::open_owned(\"x\"").len(), 1);

    // Scoping: inline, via a local fn, via a `let`, and the negative.
    let inline = "fn f() { let n = format!(\"a{}\", std::process::id()); }";
    assert!(ns_is_pid_scoped(inline, "n"));
    let helper = "fn mk(t: &str) -> String { format!(\"a_{}_{t}\", std::process::id()) }";
    assert!(ns_is_pid_scoped(helper, "mk(\"x\")"));
    assert!(ns_is_pid_scoped(helper, "&mk(\"x\")"));
    let via_let = "fn mk() -> String { format!(\"{}\", std::process::id()) }\nlet tag = mk();";
    assert!(ns_is_pid_scoped(via_let, "&tag"));
    assert!(!ns_is_pid_scoped(
        "let tag = \"fixed\".to_string();",
        "&tag"
    ));
    assert!(!ns_is_pid_scoped("", "\"literal\""));

    // NOT FOOLED by the two cheapest fakes. A guard that reads raw text accepts
    // both, and both name ONE shared page for every process.
    let in_string = "fn mk() -> String { \"process::id()\".to_string() }";
    assert!(
        !ns_is_pid_scoped(in_string, "mk()"),
        "a `process::id()` inside a STRING is not a call"
    );
    let in_comment = "fn mk() -> String { /* process::id() */ \"fixed\".to_string() }";
    assert!(
        !ns_is_pid_scoped(in_comment, "mk()"),
        "a `process::id()` in a COMMENT is not a call"
    );
    // …and the namespace ARGUMENT itself cannot be a string saying it.
    assert!(!ns_is_pid_scoped("", "\"process::id()\""));
    // The positive control still holds, so the tightening did not blind it.
    let real = "fn mk() -> String { format!(\"a{}\", std::process::id()) }";
    assert!(ns_is_pid_scoped(real, "mk()"));

    // One literal, ONE key, however the call site spells it.
    for spelling in [
        "\"pvl\"",
        "\"pvl\".into()",
        "&\"pvl\".to_string()",
        "  \"pvl\".into()  ",
    ] {
        assert_eq!(
            normalise_ns_expr(spelling),
            "\"pvl\"",
            "spelling {spelling:?}"
        );
    }
    // A helper CALL is not a conversion tail and must survive intact.
    assert_eq!(normalise_ns_expr("mwp_ns(\"t\")"), "mwp_ns(\"t\")");
}

/// A POSIX-SHM namespace in a `tests/` file must be PID-SCOPED, or declared.
#[test]
fn a_posix_shm_namespace_is_pid_scoped_or_declared_unique() {
    let root = repo_root();
    let mut all_rs: Vec<PathBuf> = Vec::new();
    collect_rs(&root, &root, &mut all_rs);
    all_rs.sort();

    let exempt: BTreeMap<&str, &str> = SHM_NS_EXEMPTIONS.iter().copied().collect();
    let mut exempt_uses: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut violations: Vec<String> = Vec::new();
    let mut scoped_seen = 0usize;
    let mut sites_seen = 0usize;

    for path in &all_rs {
        let rel = path.to_string_lossy().to_string();
        if !rel.contains("/tests/") {
            continue;
        }
        let Ok(src) = fs::read_to_string(root.join(path)) else {
            continue;
        };
        let stem = path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default();
        for (line, expr) in shm_ns_sites(&src) {
            sites_seen += 1;
            if ns_is_pid_scoped(&src, &expr) {
                scoped_seen += 1;
                continue;
            }
            let key = normalise_ns_expr(&expr);
            let key = key.as_str();
            // A declared entry may be FILE-SCOPED (`<file stem>:<expr>`); a bare
            // identifier like `ns` is far too common to clear globally.
            let scoped = format!("{stem}:{key}");
            let matched = if exempt.contains_key(scoped.as_str()) {
                Some(scoped)
            } else if exempt.contains_key(key) {
                Some(key.to_string())
            } else {
                None
            };
            match matched {
                Some(k) => exempt_uses
                    .entry(k)
                    .or_default()
                    .push(format!("{rel}:{line}")),
                None if key.starts_with('"') => violations.push(format!(
                    "  {rel}:{line} — namespace {key} is a bare literal, not pid-scoped"
                )),
                None => violations.push(format!(
                    "  {rel}:{line} — namespace `{key}` is not pid-scoped and this walk could \
                     not resolve it"
                )),
            }
        }
    }

    // Vacuity guards: the walk must have found the real corpus AND at least one
    // correctly-scoped site, or every judgement below means nothing.
    assert!(
        sites_seen >= 10,
        "the POSIX-SHM walk found only {sites_seen} namespace site(s) — it has broken, and a \
         clean sheet would mean nothing"
    );
    assert!(
        scoped_seen >= 5,
        "the walk judged only {scoped_seen} site(s) PID-SCOPED, but `monitor_wait_park_iox2_test` \
         and `barrier_park_wake_iox2_test` carry more than that between them — the scoping \
         resolver has broken and is about to report correct code as violations"
    );

    // Every declared exemption must be USED, and every declared LITERAL must
    // still be the only user of its name.
    let mut stale: Vec<&str> = Vec::new();
    for (key, _) in SHM_NS_EXEMPTIONS {
        match exempt_uses.get(*key) {
            None => stale.push(key),
            Some(uses) if key.starts_with('"') && uses.len() > 1 => violations.push(format!(
                "  namespace {key} is exempt ONLY because it is unique, and it now has {} call \
                 sites: {}",
                uses.len(),
                uses.join(", ")
            )),
            Some(_) => {}
        }
    }
    assert!(
        stale.is_empty(),
        "`SHM_NS_EXEMPTIONS` declares namespace(s) no call site uses: {stale:?}.\n\nAn exemption \
         for a namespace nobody names pre-authorises a future collision. Delete it."
    );

    assert!(
        violations.is_empty(),
        "a POSIX-SHM namespace in a `tests/` file is not pid-scoped:\n{}\n\nA doorbell object is \
         `/cer_db_{{ns}}_{{fnv(topic)}}` and a barrier's is `/cer_bar_{{fnv(ns/topic)}}` — pure \
         functions of the namespace, with no pid in them. Under nextest each test is its own \
         PROCESS running CONCURRENTLY with its siblings, so two tests passing the same namespace \
         map the SAME `/dev/shm` page however isolated their iceoryx2 roots are. That is not \
         hypothetical: it is what made `monitor_wait_park_iox2_test`'s doorbell ringer collide \
         with the sibling asserting the doorbell counter is ZERO — 2 of 2 CI runs, Linux only.\
         \n\nFIX: derive the namespace from `std::process::id()`, as `doorbell.rs`'s `test_ns` \
         and `barrier_park_wake_iox2_test.rs`'s `barrier_ns` already do:\n\n    fn my_ns(tag: \
         &str) -> String {{ format!(\"mine_{{}}_{{tag}}\", std::process::id()) }}\n\nA per-test \
         `tag` is not redundant with the pid: under a plain `cargo test` every test in a binary \
         shares ONE process.",
        violations.join("\n")
    );
}

/// A doctest that runs must not touch the process-global manager.
#[test]
fn no_executing_doctest_calls_the_singleton() {
    let root = repo_root();
    let mut all_rs: Vec<PathBuf> = Vec::new();
    collect_rs(&root, &root, &mut all_rs);
    all_rs.sort();

    let mut offenders: Vec<String> = Vec::new();
    let mut executing_seen = 0usize;
    let mut core_executing = 0usize;
    let mut files_seen = 0usize;

    for path in &all_rs {
        let rel = path.to_string_lossy().to_string();
        // `src/` only: a doctest is a `src` artefact, and this file's own
        // fixtures live under `tests/`.
        if !rel.contains("/src/") {
            continue;
        }
        let Ok(raw) = fs::read_to_string(root.join(path)) else {
            continue;
        };
        files_seen += 1;
        for (line, body) in executing_doctests(&raw) {
            executing_seen += 1;
            if rel.starts_with("crates/cerulion_core/") {
                core_executing += 1;
            }
            // The body is Rust source; a call inside a STRING there is still
            // not a call, so the same blanking view applies.
            let code = code_only_blank_literals(&body);
            if let Some(found) = SINGLETON_CALLS.iter().find(|c| code.contains(**c)) {
                offenders.push(format!(
                    "  {rel}:{line} — an executing doctest calls `{found}`"
                ));
            }
        }
    }

    // Vacuity guards. Zero offenders is the expected state, so the walk has to
    // prove it LOOKED — at real files, and at real EXECUTING doctests.
    //
    // The floors are MEASURED, not guessed. Cross-checked against rustdoc's own
    // classification (`cargo test -p cerulion_core --doc`, which labels each
    // doctest `ignored` / `- compile` (i.e. `no_run`) / `- compile fail` /
    // nothing-at-all-for-executing): `cerulion_core` has 15 doctests of which
    // exactly TWO execute, and this reader names those two. Repo-wide the
    // count is 4. That cross-check is how the reader's original three-vs-two
    // state bug was found, and it is worth repeating after any change here.
    assert!(
        files_seen >= 100,
        "the doctest walk read only {files_seen} `src/` file(s) — it has broken, and a clean \
         sheet here would mean nothing"
    );
    assert!(
        core_executing >= 1,
        "the doctest walk found NO executing doctest in `cerulion_core`, which had two when \
         this guard was written (`state.rs` and `state/shape.rs`). Either the reader has \
         broken or both became `no_run`; check against `cargo test -p cerulion_core --doc` \
         before relaxing this."
    );
    assert!(
        executing_seen >= 2,
        "the doctest walk found only {executing_seen} EXECUTING doctest(s) repo-wide, against \
         4 when this guard was written. The floor is deliberately below that count so a \
         deliberate `no_run` conversion is not a gate failure — but a reader that has \
         collapsed reports 0 or 1, and that is what this catches."
    );

    assert!(
        offenders.is_empty(),
        "a doctest that RUNS calls the process-global iceoryx2 manager:\n{}\n\n\
         `cargo test --doc` compiles AND RUNS doctests, in parallel, so this is a \
         default-namespace caller — and it is one neither of this file's other mechanisms can \
         reach: a doctest takes no `#[serial]` and is not a test BINARY, so it cannot join the \
         `{FENCE_GROUP}` fence either.\n\nFIX: mark the fence ```` ```no_run ```` . The example \
         still COMPILES, so it cannot rot; it just does not execute.",
        offenders.join("\n")
    );
}

/// Every binary named in either inventory must EXIST as a test file.
///
/// A renamed or deleted test file leaves a fence entry that matches nothing:
/// the membership check above still passes (config and inventory agree), and
/// the fence quietly protects a binary that is not there while the RENAMED one
/// runs unfenced. The `unfenced` arm of the main gate catches the rename only
/// if the new file still calls a singleton accessor directly, so this is the
/// independent half.
#[test]
fn every_fenced_binary_names_a_test_file_that_exists() {
    let root = repo_root();
    let mut all_rs: Vec<PathBuf> = Vec::new();
    collect_rs(&root, &root, &mut all_rs);
    // DEPTH-1 `tests/*.rs` only — the same rule cargo uses to decide what is a
    // test target. Collecting every `/tests/` stem would put "mod" (from
    // `tests/common/mod.rs`) and every trybuild fixture name into the set, so a
    // fence entry naming one of those would pass this check while matching no
    // binary nextest can run.
    let stems: BTreeSet<String> = all_rs
        .iter()
        .filter(|p| is_test_binary(&p.to_string_lossy()))
        .filter_map(|p| p.file_stem().map(|s| s.to_string_lossy().to_string()))
        .collect();
    assert!(
        !stems.contains("mod"),
        "the test-binary walk admitted `mod`, which means it is collecting nested \
         `tests/**` files again — a fence entry naming a module would then read as \
         protected while matching no binary"
    );
    assert!(
        stems.contains("transport_test"),
        "the test-file walk found no `transport_test` — it has broken, and every check below \
         would be vacuous"
    );

    for (label, inv) in [("fence", FENCE_INVENTORY), ("timing", TIMING_INVENTORY)] {
        let ghosts: Vec<&str> = inv
            .iter()
            .map(|(b, _)| *b)
            .filter(|b| !stems.contains(*b))
            .collect();
        assert!(
            ghosts.is_empty(),
            "the {label} inventory names binaries with no test file: {ghosts:?}.\n\nA fence \
             entry that matches nothing protects nothing. If the file was RENAMED, rename the \
             entry (in {FENCE_FILE} and in this file); if it was deleted, delete the entry."
        );
        // Every declared reason must actually say something.
        for (b, why) in inv {
            assert!(
                why.len() > 20,
                "the {label} inventory's reason for `{b}` is too thin to review: {why:?}"
            );
        }
    }
}
