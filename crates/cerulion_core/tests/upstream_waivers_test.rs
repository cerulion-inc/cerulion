// SPDX-License-Identifier: AGPL-3.0-only
//! The expiry guard for the two upstream waivers this tree carries.
//!
//! A waiver describes a defect in a SPECIFIC release of a dependency. The
//! failure mode to design against is not the waiver itself, it is the waiver
//! that outlives its cause: the day the dependency moves, nobody thinks to go
//! back and re-run the arms that were marked, and a fixed defect quietly
//! becomes permission to ship a regression. So both waivers are pinned to a
//! version, both are declared in an inventory, and this file fails closed on
//! either drifting.
//!
//! # The two waivers
//!
//! **2034, macOS only.** On macOS a process that has run one graph containing a
//! plugin node cannot create any further iceoryx2 resource. Eighteen arms
//! across twelve binaries carry
//! `#[cfg_attr(target_os = "macos", ignore = "...")]`. They run normally on
//! Linux, which is the point of the `cfg_attr`: the coverage is not lost, it is
//! lost on one platform.
//!
//! **2035, every platform.** `Publisher::loan` allocates once per sample. Eight
//! arms across three binaries subtract exactly the known loans and then assert
//! their ORIGINAL bound on the remainder, so they still fail on any NEW
//! allocation. That is the whole difference between a narrowed gate and a
//! disabled one.
//!
//! # The 2034 inventory is the set that fails under the harness CI uses
//!
//! `cargo nextest` gives every test its own PROCESS, and 2034 is a per-process
//! defect: it bites the SECOND plugin graph in a process. So under nextest only
//! the arms that build more than one plugin graph inside themselves fail, and
//! that set, eighteen arms, is what is waived here. CI shards `cerulion_core`
//! under nextest, so CI is green.
//!
//! `cargo test --test <binary>` runs a whole binary in ONE process, so on macOS
//! it will additionally fail arms that nextest proves pass, because the second
//! plugin graph in that process is the defect. `cdylib_unified_drain_test` is
//! the clearest example: 4 passed and 1 skipped under nextest, 3 passed and 1
//! failed under `cargo test`. That is the documented limit doing exactly what
//! the release note says it does, not a broken tree, and it is why the waiver
//! is not widened to cover it: widening would hide arms that nextest proves are
//! fine, and the inventory would stop describing the defect.
//!
//! # What this file will not do
//!
//! It does not check that the defects are still present. A test that asserted
//! "the bug is still there" would be a test that fails when someone fixes it,
//! which is the wrong way round. It checks that the CONDITIONS the waivers were
//! written under still hold, and it names what to do when they stop.

use std::path::{Path, PathBuf};

/// `<binary>::<test>` for every arm carrying the 2034 macOS waiver.
const WAIVED_2034: &[(&str, &str)] = &[
    (
        "cdylib_block_degrade_test",
        "dylib_block_degrade_is_deterministic",
    ),
    (
        "cdylib_collapse_no_publish_test",
        "cdylib_pre_first_delivery_never_publishes_is_deterministic",
    ),
    (
        "cdylib_dropoldest_overflow_test",
        "slow_cdylib_consumer_overflow_is_deterministic",
    ),
    (
        "cdylib_non_trigger_hold_test",
        "cdylib_hold_replay_is_deterministic",
    ),
    (
        "cdylib_portwrite_e2e_test",
        "cdylib_and_in_process_twin_produce_byte_identical_payloads",
    ),
    (
        "cdylib_qos_behavioral_test",
        "qos_watchdog_counters_are_deterministic",
    ),
    (
        "cdylib_qos_behavioral_test",
        "sample_decimation_on_dylib_is_deterministic",
    ),
    (
        "cdylib_sync_backpressure_parity_test",
        "a_block_producer_stops_at_the_declared_depth_through_the_ffi",
    ),
    (
        "cdylib_sync_backpressure_parity_test",
        "a_sample_gate_decides_which_frames_are_eligible_to_align_through_the_ffi",
    ),
    (
        "cdylib_sync_backpressure_parity_test",
        "the_block_contract_is_identical_in_process_and_through_the_ffi",
    ),
    (
        "cdylib_sync_backpressure_parity_test",
        "the_sample_contract_is_identical_in_process_and_through_the_ffi",
    ),
    (
        "cdylib_sync_backpressure_parity_test",
        "two_dylib_runs_are_byte_identical",
    ),
    (
        "cdylib_sync_nontrigger_test",
        "determinism_two_dylib_runs_byte_identical",
    ),
    (
        "cdylib_sync_nontrigger_test",
        "in_process_twin_parity_identical_sequence",
    ),
    (
        "cdylib_unbounded_sync_fire_test",
        "fire_and_delivery_are_deterministic",
    ),
    (
        "cdylib_unified_drain_test",
        "seam_forces_cdylib_back_to_separate_byte_identical",
    ),
    (
        "chunk_c_ffi_error_test",
        "cdylib_tick_lifecycle_codes_1_and_2",
    ),
    (
        "rayon_fire_cdylib_serial_test",
        "cdylib_gated_serial_trace_byte_identical_parallel_vs_serial",
    ),
];

/// Binaries whose zero-allocation arms subtract the 2035 publish allocation,
/// with the number of arms each carries.
const WAIVED_2035: &[(&str, usize)] = &[
    ("step_zero_alloc_test", 6),
    ("zero_copy_hot_path_test", 2),
    ("trace_ring_hook_zero_alloc_test", 1),
];

fn tests_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests")
}

fn read(binary: &str) -> String {
    let p: PathBuf = tests_dir().join(format!("{binary}.rs"));
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{} is readable: {e}", p.display()))
}

/// The resolved iceoryx2 version both waivers are written against.
///
/// Read from the workspace lockfile rather than from a constant in this file,
/// so that bumping the dependency is what trips this and not a second thing
/// someone also has to remember to edit.
fn locked_iceoryx2_version() -> String {
    let lock = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .map(|a| a.join("Cargo.lock"))
        .find(|p| p.exists())
        .expect("the workspace lockfile is findable from this crate");
    let text = std::fs::read_to_string(&lock).expect("the lockfile is readable");
    let mut in_pkg = false;
    for line in text.lines() {
        let line = line.trim();
        if line == "[[package]]" {
            in_pkg = false;
            continue;
        }
        if line == "name = \"iceoryx2\"" {
            in_pkg = true;
            continue;
        }
        if in_pkg {
            if let Some(v) = line
                .strip_prefix("version = \"")
                .and_then(|v| v.strip_suffix('"'))
            {
                return v.to_string();
            }
        }
    }
    panic!("no iceoryx2 package found in the workspace lockfile");
}

/// BOTH waivers expire together, because both name one release.
///
/// When this fails, the fix is not to edit the version: it is to delete the
/// waivers, run the arms they marked, and keep only what is still broken.
#[test]
fn both_waivers_are_pinned_to_the_iceoryx2_release_they_describe() {
    let locked = locked_iceoryx2_version();
    let named = cerulion_core::testing::UPSTREAM_WAIVER_IOX2_VERSION;
    assert_eq!(
        locked, named,
        "the waivers in this tree describe iceoryx2 {named}, and the workspace now resolves \
         iceoryx2 {locked}. Both defects may be fixed. DELETE the waivers, run the arms they \
         marked (the inventories are in this file), and re-add only what is still broken. \
         2034 is the macOS plugin defect; 2035 is the per-loan allocation."
    );
}

/// The 2034 arms are marked, and marked in one shape: a `cfg_attr` ignore,
/// scoped to macOS, whose reason string names the defect.
#[test]
fn every_declared_2034_arm_carries_a_macos_scoped_ignore_that_names_the_defect() {
    for (binary, test) in WAIVED_2034 {
        let src = read(binary);
        let needle = format!("\nfn {test}(");
        assert!(
            src.contains(&needle),
            "{binary}::{test} is declared waived but no longer exists. If the arm was renamed or \
             deleted, update this inventory in the same change"
        );
        let at = src.find(&needle).expect("checked above");
        // Collapse whitespace before matching, and match the parts separately
        // rather than as one contiguous string. `cargo fmt` wraps a long
        // `cfg_attr` across three lines, which broke an exact substring form of
        // this check the first time it ran against a formatted tree. A guard
        // that fails on a reformat is a guard somebody eventually deletes.
        let head: String = src[at.saturating_sub(1500)..at]
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        for part in ["cfg_attr(", "target_os = \"macos\"", "ignore", "2034"] {
            assert!(
                head.contains(part),
                "{binary}::{test} is declared waived for upstream 2034 but the attributes above \
                 it do not contain `{part}`. The waiver must be a macOS scoped ignore whose \
                 reason names the defect: a bare `#[ignore]` would hide the arm on Linux too, \
                 and an unnamed reason is a waiver nobody can retire"
            );
        }
    }
}

/// The waiver did NOT widen to every platform.
///
/// The whole value of scoping it to macOS is that Linux keeps running these
/// arms. A `cfg` slip that dropped the `target_os` would be invisible on the
/// platform that is already skipping them, so the platform that is supposed to
/// be running them is the one that checks.
#[test]
#[cfg(not(target_os = "macos"))]
fn on_this_platform_no_waived_arm_is_actually_ignored() {
    for (binary, test) in WAIVED_2034 {
        let src = read(binary);
        let at = src
            .find(&format!("\nfn {test}("))
            .unwrap_or_else(|| panic!("{binary}::{test} exists"));
        let head = &src[at.saturating_sub(1200)..at];
        let lines: Vec<&str> = head.lines().collect();
        for line in lines.iter().rev().take(6) {
            let t = line.trim();
            assert!(
                !(t.starts_with("#[ignore") || t.starts_with("#[cfg_attr(test, ignore")),
                "{binary}::{test} carries an UNCONDITIONAL ignore. The 2034 waiver is macOS \
                 only; on this platform the arm must run, and it is the only place a widened \
                 waiver can be caught"
            );
        }
    }
}

/// The 2035 arms subtract the known loans rather than dropping their bound.
///
/// A gate that widened its bound would pass a second, unrelated allocation. A
/// gate that subtracts a known quantity and keeps its bound does not, and the
/// difference is the whole reason this is a narrowing and not a disabling.
#[test]
fn every_declared_2035_arm_subtracts_the_known_loans_and_keeps_its_bound() {
    let mut total = 0usize;
    for (binary, arms) in WAIVED_2035 {
        let src = read(binary);
        let uses = src.matches("upstream_2035_publish_allocs(").count();
        assert_eq!(
            uses, *arms,
            "{binary} is declared to carry {arms} subtraction(s) for upstream 2035 and carries \
             {uses}. An arm that stopped subtracting has either been fixed upstream, in which \
             case delete the waiver, or has quietly dropped its bound, which is worse"
        );
        assert!(
            src.contains("2035"),
            "{binary} subtracts the waived allocation without naming upstream 2035 anywhere. A \
             number with no issue behind it cannot be retired by anyone but its author"
        );
        total += uses;
    }
    assert_eq!(
        total, 9,
        "the 2035 waiver covers nine subtraction sites across eight arms (the wide-level arm \
         subtracts twice, once per width). A change in that count needs a change here and a \
         reason in the commit"
    );
}

/// The subtraction is a constant this tree owns, not a literal sprinkled about.
///
/// When upstream ships the fix, one constant goes to zero and every arm above
/// returns to asserting its original bound with no other edit. That is the
/// property that makes this waiver cheap to retire, and it only holds while
/// nobody hard-codes the number.
#[test]
fn the_waived_amount_is_one_constant_and_the_arms_do_not_hard_code_it() {
    assert_eq!(
        cerulion_core::testing::UPSTREAM_2035_ALLOCS_PER_LOAN,
        1,
        "the measured cost is one allocation per loan; a different value needs a fresh \
         measurement and a note on upstream 2035"
    );
    assert_eq!(
        cerulion_core::testing::upstream_2035_publish_allocs(200),
        200,
        "the helper must be a plain multiplication, so that zeroing the constant retires the \
         whole waiver"
    );
}
