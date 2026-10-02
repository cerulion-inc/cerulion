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
//! plugin node cannot create any further iceoryx2 resource. The arms carry
//! `#[cfg_attr(target_os = "macos", ignore = "...")]`. They run normally on
//! Linux, which is the point of the `cfg_attr`: the coverage is not lost, it is
//! lost on one platform.
//!
//! **2035, every platform.** `Publisher::loan` allocates once per sample. The
//! subtracting arms subtract exactly the known loans and then assert
//! their ORIGINAL bound on the remainder, so they still fail on any NEW
//! allocation. That is the whole difference between a narrowed gate and a
//! disabled one.
//!
//! # The 2034 inventory is the set that fails under the harness CI uses
//!
//! THE MECHANISM, once. On macOS a process that has loaded a plugin linking
//! iceoryx2 cannot create any further event resource, whichever process that is:
//! the test process itself, or a `cerulion` child it spawns. So the defect bites
//! the SECOND resource creation after a plugin load, and nothing about it is
//! specific to a particular arm.
//!
//! That is why this inventory has TWO units. Where the arms run under
//! `cargo nextest`, which gives each its own process, the arm is the unit and the
//! ones that build more than one plugin graph inside themselves are declared one by
//! one: 26 arms across 13 binaries in two crates. Where the arms run under plain
//! `cargo test`, many arms share one process and the victim is whichever reaches
//! the second creation first, so a per-arm list cannot converge: two runs of the
//! same tree with the same fixtures on one macOS host disagreed on six arms out of
//! thirty nine in each direction while the failing BINARIES stayed the same. Those
//! binaries are gated whole, 3 of them in `cerulion_cli`. CI shards
//! `cerulion_core` under nextest, so CI is green.
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

use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};

/// The waiver reason's stable head, as every waived arm's `cfg_attr` spells it.
///
/// A FLOOR, not an identity. It tells a binary built from a source that already
/// had the waiver from one restored out of the build cache from before it, and
/// nothing more: a binary carrying several waived arms still holds this literal
/// when one of them loses its `cfg_attr`, which is why the source scan and not
/// this is what pins each arm. The long form is deliberate, since a short token
/// would match a binary that merely mentions the number.
///
/// It exists ONLY on macOS. Every occurrence in a waived source is inside
/// `#[cfg_attr(target_os = "macos", ignore = ...)]`, which does not expand
/// off-target, so a Linux binary never carries it and the check that reads it is
/// gated to macOS. Reading it unconditionally made every waived stem look
/// unaskable on Linux and the arm passed having asked nothing.
const WAIVER_2034_MARKER: &str = "upstream iceoryx2 0.10.0 defect 2034";

/// Every arm carrying the 2034 macOS waiver, as `(path from the workspace root,
/// test name)`. 26 arms across 13 binaries in two crates, with the binaries that
/// are gated whole declared separately below; the counts are checked, not just
/// written down, by the both-directions inventory further down.
///
/// Workspace rooted for the same reason the 2035 list is: a waiver inventory
/// scoped to the crate where a defect was first seen is incomplete by
/// construction. This one was core only until CI found eight more in the CLI
/// engine, whose profile runs build several plugin graphs in one process, which
/// is the defect's exact trigger.
const WAIVED_2034: &[(&str, &str)] = &[
    (
        "crates/cerulion_core/tests/cdylib_block_degrade_test.rs",
        "dylib_block_degrade_is_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_collapse_no_publish_test.rs",
        "cdylib_pre_first_delivery_never_publishes_is_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_dropoldest_overflow_test.rs",
        "slow_cdylib_consumer_overflow_is_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_non_trigger_hold_test.rs",
        "cdylib_hold_replay_is_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_portwrite_e2e_test.rs",
        "cdylib_and_in_process_twin_produce_byte_identical_payloads",
    ),
    (
        "crates/cerulion_core/tests/cdylib_qos_behavioral_test.rs",
        "qos_watchdog_counters_are_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_qos_behavioral_test.rs",
        "sample_decimation_on_dylib_is_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs",
        "a_block_producer_stops_at_the_declared_depth_through_the_ffi",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs",
        "a_sample_gate_decides_which_frames_are_eligible_to_align_through_the_ffi",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs",
        "the_block_contract_is_identical_in_process_and_through_the_ffi",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs",
        "the_sample_contract_is_identical_in_process_and_through_the_ffi",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_backpressure_parity_test.rs",
        "two_dylib_runs_are_byte_identical",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_nontrigger_test.rs",
        "determinism_two_dylib_runs_byte_identical",
    ),
    (
        "crates/cerulion_core/tests/cdylib_sync_nontrigger_test.rs",
        "in_process_twin_parity_identical_sequence",
    ),
    (
        "crates/cerulion_core/tests/cdylib_unbounded_sync_fire_test.rs",
        "fire_and_delivery_are_deterministic",
    ),
    (
        "crates/cerulion_core/tests/cdylib_unified_drain_test.rs",
        "seam_forces_cdylib_back_to_separate_byte_identical",
    ),
    (
        "crates/cerulion_core/tests/chunk_c_ffi_error_test.rs",
        "cdylib_tick_lifecycle_codes_1_and_2",
    ),
    (
        "crates/cerulion_core/tests/rayon_fire_cdylib_serial_test.rs",
        "cdylib_gated_serial_trace_byte_identical_parallel_vs_serial",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "auto_derive_costs_low_rate_graph_not_isolated",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "auto_mode_early_stop_costs_sampled_nodes_not_isolated",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "auto_mode_ring_sized_from_max_samples_no_capped_warn",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "cap_hit_isolates_undersampled_nodes_and_still_writes_artifact",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "capped_ring_warns_and_still_costs_flooding_nodes",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "gate_met_profile_writes_full_artifact_no_isolated",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "pre_stopped_running_flag_still_writes_isolated_artifact",
    ),
    (
        "crates/cerulion_cli_engine/tests/graph_profile_iox2_test.rs",
        "two_runs_agree_on_artifact_shape",
    ),
];

/// The three function-name families that spawn a `cerulion` SUPERVISOR child. This
/// is the waiver's DECLARED INPUT: the set of whole-binary waivers below is derived
/// from it by `the_whole_binary_inventory_equals_the_spawn_family_walk`, so the list
/// cannot drift from the code it describes, and a new suite that spawns a supervisor
/// fails that arm until it is declared.
const SUPERVISOR_SPAWN_FAMILIES: &[&str] = &["spawn_graph_run", "spawn_mp_record", "spawn_run"];

/// Binaries waived WHOLE on macOS, as `(path from the workspace root, the upstream
/// issue)`.
///
/// THE MECHANISM, and it is the same one for every row: each of these spawns a
/// `cerulion` supervisor child that loads plugin nodes, and on macOS a process that
/// has loaded a plugin linking iceoryx2 cannot create any further event resource.
/// The child is the process that loaded the plugin, so no runner arrangement on this
/// side helps: a per-arm list cannot converge either, because under plain
/// `cargo test` many arms share one process and the victim is whichever reaches the
/// second creation first. Two runs of one macOS host disagreed on six arms out of
/// thirty nine in each direction while the failing BINARIES stayed the same.
///
/// THE COVERAGE THIS COSTS, stated once: 16 binaries of `cerulion_cli` do not run on
/// macOS at all. They run on Linux, where the defect does not arise. Two of the 16,
/// `ros2_graph_e2e_test` and `network_gateway_e2e_test`, are in the set BY THE RULE
/// rather than by measurement: they were not observed failing, because their own
/// environment checks already skip them on macOS. The cure is an upstream release
/// carrying the fix, or the pinned fork carrying a patch for it; nothing in this
/// repository can shorten the list.
const WAIVED_2034_BINARIES: &[(&str, &str)] = &[
    (
        "crates/cerulion_cli/tests/credit_death_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/flashback_argv_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/graph_run_validate_gate_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/graph_start_order_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_auto_partition_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_consumer_first_spawn_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_default_ns_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_execution_mode_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_record_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/mp_record_replay_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/network_gateway_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/plain_run_resim_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/ros2_graph_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/signal_matrix_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/topic_introspect_cli_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
    (
        "crates/cerulion_cli/tests/wedge_alarm_e2e_test.rs",
        "eclipse-iceoryx/iceoryx2#2034",
    ),
];

/// Binaries whose zero-allocation arms subtract the 2035 publish allocation,
/// with the number of SUBTRACTION SITES each carries: ten sites across nine
/// arms in four binaries, because the wide level arm subtracts once per width.
///
/// Paths are relative to the WORKSPACE root, not to this crate: the ninth arm
/// lives in `cerulion_cli_engine`, and CI found it after a core-only inventory
/// missed it. A waiver inventory scoped to one crate is a waiver inventory that
/// will be incomplete again.
const WAIVED_2035: &[(&str, usize)] = &[
    ("crates/cerulion_core/tests/step_zero_alloc_test.rs", 6),
    ("crates/cerulion_core/tests/zero_copy_hot_path_test.rs", 2),
    (
        "crates/cerulion_core/tests/trace_ring_hook_zero_alloc_test.rs",
        1,
    ),
    (
        "crates/cerulion_cli_engine/tests/bag_play_zero_alloc_test.rs",
        1,
    ),
];

/// The workspace root, found by walking up to the directory holding the
/// lockfile, so a waiver in another crate is nameable from here.
///
/// Panics IN the repository, because a walk that cannot find the root there is
/// a broken guard and must say so. Out of the repository there is no root and
/// no `crates/` tree to inventory, which is what [`skip_out_of_workspace`]
/// detects BEFORE any arm calls this.
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .find(|a| a.join("Cargo.lock").exists())
        .expect("the workspace root is findable from this crate")
        .to_path_buf()
}

/// Does the tree the inventories actually walk sit beside this crate?
///
/// The fact that separates the repository from BOTH ways this file can end up
/// outside it, and it is the fact the arms depend on: every inventory here
/// walks `crates/<other crate>`, so if the siblings are not there the arms have
/// nothing to read whatever any manifest says.
///
/// * A published crate has no `crates/` at all. Measured on this crate's own
///   `cargo package --list`: zero of the 131 entries are under `crates/`, and
///   zero are under `tests/` either, since `include` keeps them out.
/// * A crate directory COPIED out of the tree keeps its `tests/`, so this file
///   runs there, but it has no siblings.
///
/// Deliberately NOT a lockfile: a published crate SHIPS `Cargo.lock` (entry two
/// of those 131), so a lockfile in an ancestor is true in exactly the
/// configuration the skip serves and cannot tell the two apart. And
/// deliberately not this crate's own `tests/`, which a copied directory keeps.
fn has_sibling_crates(manifest_dir: &Path) -> bool {
    manifest_dir.parent().is_some_and(|crates| {
        crates
            .join("cerulion_cli_engine")
            .join("Cargo.toml")
            .is_file()
    })
}

/// Whether `manifest_dir` is a crate built OUTSIDE the repository, where every
/// arm in this file has nothing to inventory and must skip rather than panic.
///
/// The manifest two levels up declares `[workspace]`. `CARGO_MANIFEST_DIR` is
/// `<repo>/crates/cerulion_core`, so ONE level up is `crates/`, which holds no
/// manifest at all: keyed there, the read failed, the probe judged every build
/// out of workspace, and every arm gated on it returned immediately and passed.
/// TWO levels up is the repository root. The same correction main made to the
/// sibling probe in `iceoryx2_version_lockstep_test`.
///
/// Taken as an argument rather than read from the environment so the arms below
/// can hand it a scratch directory shaped like a packaged crate and watch it
/// answer, which is the only negative control available: `CARGO_MANIFEST_DIR`
/// is fixed at compile time.
fn looks_unpacked(manifest_dir: &Path) -> bool {
    !std::fs::read_to_string(manifest_dir.join("../../Cargo.toml"))
        .map(|t| t.contains("[workspace]"))
        .unwrap_or(false)
}

/// The gate every inventory arm calls first.
///
/// Worth knowing, and not a reason to delete this: since `tests/` is not
/// packaged, this file never reaches a PUBLISHED crate at all. What it does
/// reach is a crate directory COPIED out of the tree, which has the tests and
/// no workspace above them, and that is what the skip serves. The skip prints
/// its reason, so a build that starts skipping cannot do it silently.
fn skip_out_of_workspace() -> bool {
    let unpacked = looks_unpacked(Path::new(env!("CARGO_MANIFEST_DIR")));
    if unpacked {
        eprintln!(
            "skip: out-of-workspace build, no crates/ tree to inventory \
             (the manifest `=` pins still govern downstream resolution)"
        );
    }
    unpacked
}

/// Read a file named by a path relative to the workspace root.
fn read_at(rel: &str) -> String {
    let p = workspace_root().join(rel);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{} is readable: {e}", p.display()))
}

/// Every `.rs` file under any crate's `src` or `tests`, as workspace relative
/// paths. The inventories below are compared against what this finds, in BOTH
/// directions, because a hand list can only ever catch drift by deletion.
/// Files whose own text names the patterns these scans hunt for, and which
/// are therefore not evidence of a waiver: this guard, whose mutation arms
/// quote a widened `cfg_attr` and a 2034 reason verbatim and which calls the
/// helper to pin its arithmetic, and the module that DEFINES the helper, where
/// the name appears in a signature rather than at a call site. Naming them
/// here is deliberate: a scan that silently skipped files could be defeated by
/// moving a waiver into one.
const NOT_EVIDENCE: &[&str] = &[
    "crates/cerulion_core/tests/upstream_waivers_test.rs",
    "crates/cerulion_core/src/testing.rs",
];

fn every_rust_source() -> Vec<String> {
    fn walk(dir: &Path, root: &Path, out: &mut Vec<String>) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        for e in entries.flatten() {
            let p = e.path();
            if p.is_dir() {
                // `target` holds build output, and a vendored tree is not ours.
                let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
                if name != "target" && !name.starts_with('.') {
                    walk(&p, root, out);
                }
            } else if p.extension().and_then(|e| e.to_str()) == Some("rs") {
                if let Ok(rel) = p.strip_prefix(root) {
                    let rel = rel.to_string_lossy().replace('\\', "/");
                    if !NOT_EVIDENCE.contains(&rel.as_str()) {
                        out.push(rel);
                    }
                }
            }
        }
    }
    let root = workspace_root();
    let crates = root.join("crates");
    let mut out = Vec::new();
    let Ok(entries) = std::fs::read_dir(&crates) else {
        panic!("the crates directory is readable");
    };
    for e in entries.flatten() {
        let c = e.path();
        if !c.is_dir() {
            continue;
        }
        for sub in ["src", "tests"] {
            let d = c.join(sub);
            if d.is_dir() {
                walk(&d, &root, &mut out);
            }
        }
    }
    out.sort();
    assert!(
        out.len() > 500,
        "the workspace walk found only {} source files, which is implausibly few: the walk is \
         broken and every set comparison below would pass vacuously",
        out.len()
    );
    out
}

/// The CONTIGUOUS attribute run directly above `fn <test>(`, whitespace
/// collapsed to single spaces.
///
/// Contiguous is the whole point. A lookback of N bytes reaches over a blank
/// line into the NEIGHBOURING arm's attributes, so an arm whose own attribute
/// was deleted still passes on its neighbour's. The walk therefore stops at the
/// first line that is not part of an attribute: blank, a comment (which is
/// where a doc block starts), or the closing brace of the item above.
///
/// `cargo fmt` wraps a long `cfg_attr` across several lines, so the run cannot
/// be recognised line by line either; it is collected first and collapsed
/// after.
fn attribute_run(src: &str, test: &str) -> Option<String> {
    let at = src.find(&format!("\nfn {test}("))?;
    let mut lines: Vec<&str> = Vec::new();
    for line in src[..at].lines().rev() {
        let t = line.trim();
        if t.is_empty() || t.starts_with("//") || t == "}" || t.starts_with("mod ") {
            break;
        }
        lines.push(line);
    }
    lines.reverse();
    let collapsed = lines
        .join(" ")
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ");
    // `cargo fmt` wraps a long attribute, so collapsing leaves a space after the
    // opening paren that the unwrapped form does not have. Normalise it, or the
    // check reads one formatting as correct and the other as a missing waiver.
    Some(collapsed.replace("( ", "(").replace(" )", ")"))
}

/// Whether an attribute run is a correct 2034 waiver, or why it is not.
///
/// Split out from the test so the three mutations that defeated the previous
/// form can be run against it directly, below. A guard nobody has attacked is a
/// guard nobody knows the strength of.
fn verdict_2034(run: &str) -> Result<(), String> {
    const WANTED: &str = "#[cfg_attr(target_os = \"macos\", ignore = \"";
    let ignores = run.matches("ignore").count();
    if ignores != 1 {
        return Err(format!(
            "the attribute run holds {ignores} occurrences of `ignore`, expected exactly one: a \
             second one can widen or replace the waiver without removing it"
        ));
    }
    let Some(start) = run.find(WANTED) else {
        return Err(format!(
            "the attribute run has no contiguous `{WANTED}`: a waiver that is widened past macOS, \
             or written as prose, stops the arm running where it should and no longer says so"
        ));
    };
    let reason_end = run[start + WANTED.len()..]
        .find('"')
        .ok_or_else(|| "the ignore reason is unterminated".to_string())?;
    let reason = &run[start + WANTED.len()..start + WANTED.len() + reason_end];
    if !reason.contains("2034") {
        return Err(format!(
            "the ignore reason does not name 2034, so nobody can retire it: {reason}"
        ));
    }
    Ok(())
}

/// The resolved iceoryx2 version both waivers are written against, read from
/// the workspace lockfile so that bumping the dependency is what trips this and
/// not a second thing someone also has to remember to edit.
fn locked_iceoryx2_version() -> String {
    let lock = workspace_root().join("Cargo.lock");
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
#[test]
fn both_waivers_are_pinned_to_the_iceoryx2_release_they_describe() {
    if skip_out_of_workspace() {
        return;
    }
    let locked = locked_iceoryx2_version();
    let named = cerulion_core::testing::UPSTREAM_WAIVER_IOX2_VERSION;
    assert_eq!(
        locked, named,
        "the waivers in this tree describe iceoryx2 {named}, and the workspace now resolves \
         iceoryx2 {locked}. Both defects may be fixed. DELETE the waivers, run the arms they \
         marked, and re-add only what is still broken."
    );
}

/// Every declared 2034 arm carries its own macOS scoped ignore, in its own
/// contiguous attribute run, naming the defect.
#[test]
fn the_whole_binary_inventory_equals_the_spawn_family_walk() {
    if skip_out_of_workspace() {
        return;
    }
    // DECLARED INPUT, PARSED OUTPUT: the families are declared above, the set is
    // walked out of the sources here, and the two must agree. Writing the list by
    // hand is what let the earlier per-arm waiver crawl across three heads.
    let mut found: Vec<String> = Vec::new();
    for crate_dir in ["cerulion_cli", "cerulion_cli_engine"] {
        let dir = workspace_root()
            .join("crates")
            .join(crate_dir)
            .join("tests");
        let Ok(entries) = std::fs::read_dir(&dir) else {
            panic!(
                "{} is not readable, so this walk would pass having seen nothing",
                dir.display()
            );
        };
        for e in entries.flatten() {
            let p = e.path();
            if p.extension().and_then(|x| x.to_str()) != Some("rs") {
                continue;
            }
            let rel = format!(
                "crates/{crate_dir}/tests/{}",
                p.file_name()
                    .and_then(|x| x.to_str())
                    .expect("a test file name")
            );
            let src = read_at(&rel);
            if SUPERVISOR_SPAWN_FAMILIES
                .iter()
                .any(|fam| src.contains(&format!("{fam}(")) || src.contains(&format!("{fam}_")))
            {
                found.push(rel);
            }
        }
    }
    found.sort();
    let mut declared: Vec<String> = WAIVED_2034_BINARIES
        .iter()
        .map(|(f, _)| (*f).to_string())
        .collect();
    declared.sort();
    let missing: Vec<&String> = found.iter().filter(|f| !declared.contains(f)).collect();
    let stale: Vec<&String> = declared.iter().filter(|f| !found.contains(f)).collect();
    assert!(
        missing.is_empty(),
        "these test binaries call a supervisor-spawn family and are NOT declared \
         waived whole: {missing:?}. On macOS they will run and fail on the upstream \
         defect. Declare them, or stop spawning a supervisor from them."
    );
    assert!(
        stale.is_empty(),
        "these binaries are declared waived whole but no longer call any \
         supervisor-spawn family: {stale:?}. The waiver outlived its reason, which is \
         the failure mode this file exists to prevent."
    );
    assert!(
        found.len() >= 10,
        "the walk found only {} supervisor-spawning binaries, which is fewer than the \
         suites known to exist: the families are not matching, so this arm would pass \
         having derived almost nothing",
        found.len()
    );
}

#[test]
fn every_waived_binary_is_gated_whole_and_names_the_defect() {
    if skip_out_of_workspace() {
        return;
    }
    for (file, issue) in WAIVED_2034_BINARIES {
        let src = read_at(file);
        assert!(
            src.contains("#![cfg(not(target_os = \"macos\"))]"),
            "{file} is declared waived WHOLE but carries no file-level \
             `#![cfg(not(target_os = \"macos\"))]`, so on macOS its arms still build \
             and run. Gate the file or declare its arms one by one."
        );
        assert!(
            src.contains(WAIVER_2034_MARKER),
            "{file} is gated whole but does not name the defect: a reader meeting \
             the gate has nothing to look up. Name `{WAIVER_2034_MARKER}` beside it."
        );
        assert!(
            !src.contains("ignore = \"upstream iceoryx2 0.10.0 defect 2034"),
            "{file} carries BOTH the file-level gate and a per-arm ignore for the \
             same defect. One unit per file: the per-arm attributes are redundant \
             once the file cannot compile on macOS at all."
        );
        assert!(
            !WAIVED_2034.iter().any(|(f, _)| f == file),
            "{file} is declared in BOTH inventories, which makes the counts and the \
             per-crate rule say two different things about one binary."
        );
        assert!(
            issue.contains("2034"),
            "{file} is waived without naming the upstream issue (got `{issue}`)"
        );
    }
}

/// On macOS a whole-binary waiver is only real if the binary really lists nothing.
/// The source check above reads the gate; this one reads what libtest does with it.
#[test]
fn a_waived_binary_lists_no_test_on_macos() {
    if skip_out_of_workspace() {
        return;
    }
    if !cfg!(target_os = "macos") {
        // Off macOS the gate is inert BY DESIGN and the arms run, which the
        // per-platform assertion in the arm below is what checks. Written to the
        // stdout handle so a passing run says which platform it judged.
        let mut out = std::io::stdout().lock();
        writeln!(
            out,
            "skip: whole-binary waivers are only observable on macOS; this run is not macOS"
        )
        .expect("report the platform skip on stdout");
        out.flush().expect("flush the platform skip");
        return;
    }
    let pairs: Vec<(&str, &str)> = WAIVED_2034_BINARIES
        .iter()
        .map(|(f, _)| {
            (
                *f,
                Path::new(f)
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .expect("a waived binary path names a .rs file"),
            )
        })
        .collect();
    let stems: Vec<&str> = pairs.iter().map(|(_, s)| *s).collect();
    let built = candidates_by_stem(&stems);
    let mut asked = 0usize;
    for (file, stem) in &pairs {
        let Some(found) = built.get(*stem) else {
            continue;
        };
        let Some(bin) = found.first() else {
            continue;
        };
        // STALENESS FIRST. A binary built before the file was gated still lists its
        // arms, and reading that as "the gate did not take" is a false red: the
        // binary predates the gate. Compare mtimes and skip, with the skip visible,
        // which is the same rule the per-arm side applies through its marker check.
        let src_path = workspace_root().join(*file);
        let src_time = std::fs::metadata(&src_path).and_then(|m| m.modified()).ok();
        let bin_time = std::fs::metadata(bin).and_then(|m| m.modified()).ok();
        if let (Some(s), Some(b)) = (src_time, bin_time) {
            if b < s {
                let mut out = std::io::stdout().lock();
                writeln!(
                    out,
                    "skip: {stem}'s built binary predates its own source, so it cannot \
                     answer what the gate does (rebuild it to ask)"
                )
                .expect("report the stale-binary skip on stdout");
                out.flush().expect("flush the stale-binary skip");
                continue;
            }
        }
        let listed = arms_of(bin, false);
        assert!(
            listed.is_empty(),
            "{stem} is declared waived WHOLE on macOS, but libtest lists {} arm(s) \
             in the built binary ({}): the file-level gate did not take, so those \
             arms will RUN here.",
            listed.len(),
            bin.display()
        );
        asked += 1;
    }
    let mut out = std::io::stdout().lock();
    writeln!(
        out,
        "whole-binary waivers asked on macOS: {asked} of {} (the rest were not built \
         by this invocation)",
        stems.len()
    )
    .expect("report the asked count on stdout");
    out.flush().expect("flush the asked count");
}

#[test]
fn every_declared_2034_arm_carries_a_macos_scoped_ignore_that_names_the_defect() {
    if skip_out_of_workspace() {
        return;
    }
    for (file, test) in WAIVED_2034 {
        let src = read_at(file);
        let run = attribute_run(&src, test).unwrap_or_else(|| {
            panic!("{file}::{test} is declared waived but no longer exists under that name")
        });
        if let Err(why) = verdict_2034(&run) {
            panic!("{file}::{test}: {why}\nattribute run was: {run}");
        }
    }
}

/// The three mutations that the previous form of the check above let through.
///
/// Each is a real way the waiver decays, and each must be refused. They run
/// against `verdict_2034` and `attribute_run` directly, so the check is tested
/// rather than merely used.
#[test]
fn the_2034_check_refuses_the_three_ways_a_waiver_decays() {
    // 1. Widened past macOS: the arm silently stops running on Linux too.
    let widened = "#[cfg_attr(any(target_os = \"macos\", target_os = \"linux\"), ignore = \
                   \"upstream 2034\")] #[test]";
    assert!(
        verdict_2034(widened).is_err(),
        "a waiver widened past macOS must be refused: it stops the arm on the platform that is \
         supposed to keep running it"
    );

    // 2. Prose carrying the tokens instead of an attribute.
    let prose = "#[test] #[serial]";
    assert!(
        verdict_2034(prose).is_err(),
        "an attribute run with no cfg_attr must be refused however much prose nearby mentions \
         2034 and macos"
    );

    // 3. The neighbour's attribute, reached by a byte lookback rather than a
    //    contiguous walk. Two arms, the first waived, the second not.
    let neighbours = "\
#[cfg_attr(target_os = \"macos\", ignore = \"upstream 2034 defect\")]
#[test]
fn first_arm() {
    body();
}

#[test]
fn second_arm() {
    body();
}
";
    let run = attribute_run(neighbours, "second_arm").expect("the second arm is found");
    assert!(
        !run.contains("cfg_attr"),
        "the walk reached over the blank line into the previous arm's attributes; it must stop \
         at the first line that is not part of this arm's own run. Run was: {run}"
    );
    assert!(
        verdict_2034(&run).is_err(),
        "an arm whose own waiver was deleted must be refused even when its neighbour has one"
    );

    // A positive control beside the three negatives, so a checker that refuses
    // everything cannot pass this test.
    let good = "#[cfg_attr(target_os = \"macos\", ignore = \"upstream iceoryx2 0.10.0 defect \
                2034: one plugin graph per process\")] #[test] #[serial]";
    assert!(
        verdict_2034(good).is_ok(),
        "the check must still accept a correct waiver: {:?}",
        verdict_2034(good)
    );
}

/// The 2034 inventory equals what the workspace actually carries, in BOTH
/// directions.
///
/// A one directional hand list cannot see drift by ADDITION, and that class has
/// already bitten this series twice: a waiver added in a crate the list did not
/// name would ship green.
#[test]
fn the_2034_inventory_equals_what_the_workspace_carries() {
    if skip_out_of_workspace() {
        return;
    }
    let mut found: Vec<(String, String)> = Vec::new();
    for file in every_rust_source() {
        let src = read_at(&file);
        if !src.contains("2034") {
            continue;
        }
        for (i, _) in src.match_indices("\nfn ") {
            let rest = &src[i + 4..];
            let Some(paren) = rest.find('(') else {
                continue;
            };
            let name = &rest[..paren];
            if !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') || name.is_empty() {
                continue;
            }
            if let Some(run) = attribute_run(&src, name) {
                if run.contains("cfg_attr") && run.contains("ignore") && run.contains("2034") {
                    found.push((file.clone(), name.to_string()));
                }
            }
        }
    }
    found.sort();
    let mut declared: Vec<(String, String)> = WAIVED_2034
        .iter()
        .map(|(f, t)| ((*f).to_string(), (*t).to_string()))
        .collect();
    declared.sort();
    assert_eq!(
        found, declared,
        "the 2034 waivers in the workspace and the inventory in this file disagree. Anything in \
         the first list and not the second is an UNDECLARED waiver, which is how a silently \
         skipped arm ships; anything in the second and not the first is a stale line that \
         outlived its arm."
    );
}

/// The waiver did NOT widen to every platform, read off the SOURCE.
///
/// Kept beside the runtime oracle below rather than replaced by it, because the
/// two fail on different things and neither subsumes the other: this one runs
/// whether or not a sibling binary has been built, and it is exact because of
/// the contiguous attribute walk the 2034 verdict uses, so an `ignore` anywhere
/// in the arm's own attribute run is visible here and one on a neighbour is not
/// attributed to it. What it cannot see is what libtest will actually DO.
#[test]
#[cfg(not(target_os = "macos"))]
fn on_this_platform_every_waived_arm_actually_runs() {
    if skip_out_of_workspace() {
        return;
    }
    for (file, test) in WAIVED_2034 {
        let src = read_at(file);
        let run = attribute_run(&src, test).expect("the arm exists");
        assert!(
            !run.contains("#[ignore"),
            "{file}::{test} carries an unconditional ignore. The 2034 waiver is macOS only; on \
             this platform the arm must run, and this is the only place a widened waiver is \
             visible. Run was: {run}"
        );
        assert!(
            run.contains("target_os = \"macos\""),
            "{file}::{test} is waived without naming macOS, so it is skipped here too. Run was: \
             {run}"
        );
    }
}

/// Strip the message string literals out of a window of source, so a bound
/// check reads the EXPRESSION a test asserts and not the sentence it prints.
/// Every assert in the waived arms carries a message that mentions `waived` in
/// braces, which would otherwise satisfy any textual check by itself.
fn without_message_strings(window: &str) -> String {
    let mut out = String::with_capacity(window.len());
    let mut in_string = false;
    let mut escaped = false;
    for ch in window.chars() {
        match (in_string, ch) {
            (false, '"') => {
                in_string = true;
                out.push(' ');
            }
            (false, _) => out.push(ch),
            (true, _) if escaped => escaped = false,
            (true, '\\') => escaped = true,
            (true, '"') => in_string = false,
            (true, _) => {}
        }
    }
    out
}

/// Whether `name` appears in `text` as a whole identifier rather than as the
/// prefix of a longer one (`waived` must not match `waived_8`).
fn mentions_ident(text: &str, name: &str) -> bool {
    let ident = |c: char| c.is_alphanumeric() || c == '_';
    let mut from = 0usize;
    while let Some(i) = text[from..].find(name) {
        let at = from + i;
        let before_ok = at == 0 || !text[..at].chars().next_back().is_some_and(ident);
        let after = at + name.len();
        let after_ok = !text[after..].chars().next().is_some_and(ident);
        if before_ok && after_ok {
            return true;
        }
        from = after;
    }
    false
}

/// Does the arm at this subtraction site still keep a BOUND on the remainder?
///
/// Counting the call sites says the subtraction is there; it says nothing about
/// what is done with the result, and widening `assert_eq!(allocs, waived)` to
/// `assert!(allocs < waived + 1000)` keeps the call and throws the gate away.
/// That is exactly the "narrowed versus disabled" line the waiver rests on, so
/// it is read here rather than assumed.
///
/// The rule: inside the window, some `assert` must test an expression that is
/// the waived binding or something derived from it BY SUBTRACTION (which is how
/// `bag_play` does it: the waiver comes off a delta before the band is applied),
/// and no assert may add slack to the waived amount.
fn bound_verdict(window: &str, name: &str) -> Result<(), String> {
    let code = without_message_strings(window);

    // Bindings derived from the waived amount by subtracting it.
    let mut derived = vec![name.to_string()];
    loop {
        let mut grew = false;
        for line in code.lines() {
            let t = line.trim();
            let Some(rest) = t.strip_prefix("let ") else {
                continue;
            };
            let Some((lhs, rhs)) = rest.split_once('=') else {
                continue;
            };
            let id = lhs.trim().trim_start_matches("mut ").trim();
            if id.is_empty() || derived.iter().any(|d| d == id) {
                continue;
            }
            let subtracts = derived
                .iter()
                .any(|d| mentions_ident(rhs, d) && (rhs.contains('-') || rhs.contains("_sub(")));
            if subtracts {
                derived.push(id.to_string());
                grew = true;
            }
        }
        if !grew {
            break;
        }
    }

    let mut bounded = false;
    let mut from = 0usize;
    while let Some(i) = code[from..].find("assert") {
        let at = from + i;
        // The asserted EXPRESSION is everything up to the message, and every
        // assert in these arms carries one.
        let tail = &code[at..];
        let end = tail.find(");").map_or(tail.len(), |e| e + 2);
        let args = &tail[..end];
        for d in &derived {
            if mentions_ident(args, d) {
                bounded = true;
            }
        }
        if args.contains(&format!("{name} +")) || args.contains(&format!("+ {name}")) {
            return Err(format!(
                "an assert adds slack to the waived amount (`{name} +`), which keeps the \
                 subtraction and throws the bound away: {}",
                args.split_whitespace().collect::<Vec<_>>().join(" ")
            ));
        }
        from = at + "assert".len();
    }
    if !bounded {
        return Err(format!(
            "no assert in the window tests `{name}` or anything subtracted from it, so the \
             waived amount is computed and then ignored"
        ));
    }
    Ok(())
}

/// Every `let <name> = ...upstream_2035_publish_allocs(...)` binding in a file,
/// as `(binding name, the window of source it governs)`.
fn subtraction_sites(src: &str) -> Vec<(String, String)> {
    let lines: Vec<&str> = src.lines().collect();
    let mut out = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        if !line.contains("upstream_2035_publish_allocs(") {
            continue;
        }
        let name = line
            .trim()
            .strip_prefix("let ")
            .and_then(|r| r.split('=').next())
            .map(|n| n.trim().trim_start_matches("mut ").trim().to_string())
            .unwrap_or_default();
        if name.is_empty() {
            continue;
        }
        // Far enough to reach the assert, short enough not to borrow the next
        // arm's: the longest of the ten sites puts fourteen lines between the
        // binding and its bound.
        let end = (i + 26).min(lines.len());
        out.push((name, lines[i..end].join("\n")));
    }
    out
}

/// The directory holding this run's test binaries, which is where a sibling
/// waived binary is if it was built at all.
fn deps_dir() -> PathBuf {
    std::env::current_exe()
        .expect("the running test binary")
        .parent()
        .expect("a test binary lives in a directory")
        .to_path_buf()
}

/// Every `<stem>-<hash>` in this run's deps directory, bucketed by stem and
/// ordered NEWEST FIRST, from ONE directory walk.
///
/// One walk because the directory is large: 124,302 entries in a working
/// checkout, 7 seconds warm and 22 cold. The previous shape walked it once per
/// stem and then once more for every stem it could not answer, which on a
/// platform where no stem can be answered is twice the stems.
fn candidates_by_stem(stems: &[&str]) -> BTreeMap<String, Vec<PathBuf>> {
    let mut found: BTreeMap<String, Vec<(std::time::SystemTime, PathBuf)>> = BTreeMap::new();
    if let Ok(entries) = std::fs::read_dir(deps_dir()) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            let Some(name) = path.file_name().map(|n| n.to_string_lossy().into_owned()) else {
                continue;
            };
            for stem in stems {
                // `<stem>-<hash>` and nothing else: not `<stem>-<hash>.d`, not
                // an object file, not a different test whose name merely starts
                // with this one.
                let Some(tail) = name.strip_prefix(&format!("{stem}-")) else {
                    continue;
                };
                if tail.is_empty() || !tail.chars().all(|c| c.is_ascii_hexdigit()) {
                    continue;
                }
                let Ok(when) = entry.metadata().and_then(|m| m.modified()) else {
                    // This entry, not the next stem: the name already matched
                    // this one, so trying it against the others would bucket it
                    // under a stem it does not belong to.
                    break;
                };
                found
                    .entry((*stem).to_string())
                    .or_default()
                    .push((when, path.clone()));
                break;
            }
        }
    }
    found
        .into_iter()
        .map(|(stem, mut v)| {
            // Newest first, and by PATH when the timestamps tie: a cache restore
            // stamps many files with one time, and `read_dir` order is arbitrary,
            // so without the tie-break the pick among them is nondeterministic.
            v.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
            (stem, v.into_iter().map(|(_, p)| p).collect())
        })
        .collect()
}

/// The first candidate, newest first, whose BYTES carry `marker`, or `None`.
///
/// The marker is what makes the choice correspond to the source rather than to a
/// timestamp. A shard that does not build a waived binary still finds one in the
/// deps directory, restored from the build cache and built from an older source,
/// and newest-by-mtime picks it: the cache sets every restored file's mtime to
/// the restore, so a stale binary can be newer than the checkout it contradicts.
/// Asking libtest about that binary answers a question about a source nobody is
/// testing, which is how this gate reported a waived arm as unignored on macOS
/// while the arm at that source is ignored, proved by running `--list --ignored`
/// on the binary built from it.
///
/// Newest first, so the marker filters an ordered walk instead of acting as a
/// second clock.
fn newest_carrying(candidates: &[PathBuf], marker: &str) -> Option<PathBuf> {
    candidates
        .iter()
        .find(|p| {
            std::fs::read(p)
                .is_ok_and(|bytes| bytes.windows(marker.len()).any(|w| w == marker.as_bytes()))
        })
        .cloned()
}

/// The newest built binary for `stem` in this run's own deps directory, or
/// `None` when no candidate is there at all.
///
/// Newest by modification time ONLY, which is weaker than it looks: a
/// cache-restored binary carries the restore's timestamp, so newest does not mean
/// built from this source. The waiver arm does not use this for its answer, it
/// uses [`newest_carrying`], which reads the source's own text out of the
/// candidate. This remains for the anti-vacuity check, where the question is
/// merely whether any candidate exists.
fn built_test_binary(stem: &str) -> Option<PathBuf> {
    candidates_by_stem(&[stem]).remove(stem)?.into_iter().next()
}

/// Ask a test binary what libtest knows. `ignored` selects the `--ignored`
/// filter, which prints ONLY the arms libtest would SKIP.
fn arms_of(bin: &Path, ignored: bool) -> BTreeSet<String> {
    let mut cmd = std::process::Command::new(bin);
    cmd.arg("--list");
    if ignored {
        cmd.arg("--ignored");
    }
    let out = cmd
        .output()
        .unwrap_or_else(|e| panic!("{} --list: {e}", bin.display()));
    assert!(
        out.status.success(),
        "{} --list exited {:?}",
        bin.display(),
        out.status.code()
    );
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|l| l.strip_suffix(": test"))
        .map(str::to_string)
        .collect()
}

/// What libtest will actually DO with the waived arms, asked of the binaries
/// rather than inferred from their source.
///
/// `--list --ignored` prints ONLY the arms libtest would skip, which makes this
/// a real oracle in BOTH directions: on macOS every declared waived arm must be
/// in that set, and on every other platform none of them may be. The source
/// scan above cannot reach either fact, because a `cfg_attr` is a claim about
/// what the compiler did and this is a reading of what it produced.
///
/// It also catches a declared arm whose NAME no longer exists, since libtest
/// lists what it will run and a renamed arm is simply absent.
#[test]
fn libtests_own_list_agrees_with_the_2034_waiver_on_this_platform() {
    // Anti-vacuity for the discovery itself: this binary is always built, so if
    // the search cannot find IT the search is looking in the wrong place and a
    // zero below would be a lie rather than a fact.
    assert!(
        built_test_binary("upstream_waivers_test").is_some(),
        "the binary search cannot find this very test binary in {}, so it would \
         report every waived binary as unbuilt",
        deps_dir().display()
    );

    let mut by_binary: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for (file, test) in WAIVED_2034 {
        let stem = Path::new(file)
            .file_stem()
            .and_then(|s| s.to_str())
            .expect("a waived path names a .rs file");
        by_binary.entry(stem).or_default().push(test);
    }

    let mut checked_binaries = 0usize;
    let mut checked_arms = 0usize;
    let mut unbuilt: Vec<&str> = Vec::new();
    let mut stale: Vec<&str> = Vec::new();
    // ONE directory walk for every stem, rather than one per stem and another
    // for every stem that cannot be answered.
    let stems: Vec<&str> = by_binary.keys().copied().collect();
    let candidates = candidates_by_stem(&stems);
    let empty: Vec<PathBuf> = Vec::new();
    for (stem, tests) in &by_binary {
        let found = candidates.get(*stem).unwrap_or(&empty);
        // On macOS the newest candidate carrying the waiver's own reason text,
        // because a cache-restored binary from before the waiver would otherwise
        // answer for a source nobody is testing. Off macOS the marker cannot
        // exist, since `cfg_attr(target_os = "macos", ...)` does not expand
        // there, so the choice stays what it was: the newest candidate.
        let chosen = if cfg!(target_os = "macos") {
            newest_carrying(found, WAIVER_2034_MARKER)
        } else {
            found.first().cloned()
        };
        let Some(bin) = chosen else {
            if found.is_empty() {
                unbuilt.push(stem);
            } else {
                // Present, and on macOS carrying no waiver text, so built before
                // the waiver landed. Asserted below: a binary that cannot answer
                // must not read as an answer.
                stale.push(stem);
            }
            continue;
        };
        checked_binaries += 1;
        let listed = arms_of(&bin, false);
        let skipped = arms_of(&bin, true);
        for test in tests {
            assert!(
                listed.contains(*test),
                "{stem}::{test} is declared waived but libtest does not list it. \
                 Either the arm was renamed and this inventory is stale, or the \
                 built binary is older than the source: rebuild and re-run."
            );
            checked_arms += 1;
            if cfg!(target_os = "macos") {
                assert!(
                    skipped.contains(*test),
                    "{stem}::{test} carries the 2034 waiver but libtest will RUN it \
                     on macOS. The waiver exists because the arm cannot pass here, \
                     so a running arm means the `cfg_attr` did not take."
                );
            } else {
                assert!(
                    !skipped.contains(*test),
                    "{stem}::{test} is SKIPPED on this platform. The 2034 waiver is \
                     macOS only and the coverage is supposed to be lost on one \
                     platform, not everywhere: this is the widened-waiver failure."
                );
            }
        }
    }

    // A binary that is PRESENT but cannot answer is a red, not a skip. Before
    // this assertion the arm reported such a binary through an `eprintln`, which
    // libtest captures on a pass, so the one case this whole arm exists for
    // printed `ok`.
    //
    // The companion rule is below: asking SOME of the stems and silently
    // skipping the rest is also a red. Only asking NONE is a legitimate skip,
    // which is a checkout where this invocation built no waived binary at all.
    assert!(
        stale.is_empty(),
        "{stale:?}: a binary for each of these waived stems is in {} but carries \
         no 2034 waiver text, so it was built before the waiver and cannot answer \
         what libtest will do with the arm at this source. The shard that runs \
         this gate must build the binaries it asks. FIX: rebuild those test \
         binaries in this invocation.",
        deps_dir().display()
    );
    if checked_binaries == 0 {
        // Written to the stdout HANDLE rather than through `println!`. libtest
        // captures what the print macros emit and shows it only for a FAILING
        // test, so a notice sent through them is absent from the one run it has
        // to appear in: the green one, where the arm passed having asked libtest
        // nothing. A write to the handle is not captured and lands in the log
        // beside the `ok` line. Measured both ways on a passing test: without
        // `--nocapture` only the handle write appears, with it all three do.
        let mut out = std::io::stdout().lock();
        writeln!(
            out,
            "skip: this invocation built none of the {} waived binaries, so there \
             is nothing to ask libtest (the source scan still covers them)",
            by_binary.len()
        )
        .expect("report the vacuous pass on stdout");
        out.flush().expect("flush the vacuous-pass notice");
        return;
    }
    // ALL or NONE, PER CRATE. A run that asked some of a crate's waived binaries
    // and silently skipped the rest reports a number that reads like coverage and
    // is not: the arms in the skipped binaries were never put to libtest.
    // `unbuilt` was reported and never asserted, which was the remaining way for
    // this arm to pass having checked less than it says.
    //
    // Per CRATE and not across the inventory, because the per-arm inventory spans
    // two and no single `-p` can build both: `cargo test -p cerulion_core` cannot build a
    // `cerulion_cli_engine` target, so an inventory-wide rule would make the
    // ordinary per-crate command unpassable while offering a remedy that does not
    // exist. Per crate, that run reads one crate fully asked and the other fully
    // unasked; neither of those overstates what was put to libtest, and CI, which
    // builds both, still gets the whole rule.
    // Distinct BINARIES per crate, not inventory rows: several arms share one
    // binary and the question here is which binaries were asked.
    let mut by_crate: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();
    for (file, _) in WAIVED_2034 {
        let stem = Path::new(file)
            .file_stem()
            .and_then(|s| s.to_str())
            .expect("a waived path names a .rs file");
        let crate_dir = file
            .strip_prefix("crates/")
            .and_then(|rest| rest.split('/').next())
            .expect("a waived path names a crate under crates/");
        by_crate.entry(crate_dir).or_default().insert(stem);
    }
    for (crate_dir, stems) in &by_crate {
        let missing: Vec<&str> = stems
            .iter()
            .copied()
            .filter(|s| unbuilt.contains(s))
            .collect();
        let asked = stems.len() - missing.len();
        assert!(
            asked == 0 || missing.is_empty(),
            "in `{crate_dir}` this invocation asked {asked} of {} waived binaries \
             and skipped {missing:?}, which it did not build. Asking some of a \
             crate's waived binaries and not the rest reports coverage this arm \
             does not have. FIX: build that crate's waived binaries in the same \
             invocation as this gate, or build none of them and take the skip.",
            stems.len()
        );
    }
    eprintln!(
        "libtest list oracle: {checked_arms} waived arm(s) across {checked_binaries} \
         of {} binaries; not built here: {unbuilt:?}; present but built before the \
         waiver, so not asked: {stale:?}",
        by_binary.len()
    );
}

/// The skip must NOT fire inside the repository, and this is the arm that makes
/// a silent one impossible.
///
/// A gate that returns early is invisible to libtest: the `eprintln` explaining
/// the skip is captured and shown only for a FAILING test, so an arm that checks
/// nothing still prints `ok`. That is exactly what happened, and it took a
/// `--nocapture` run to see it: the probe read `../Cargo.toml` where the root
/// manifest is two levels up, so five inventory arms passed in 0.00s having
/// inventoried nothing.
///
/// It asserts on `skip_out_of_workspace` ITSELF, not on the predicate behind
/// it. Pinning the predicate would leave the same hole one level down: a gate
/// that grew a second condition could start skipping in the repository while
/// this arm went on passing, which is the shape of the bug it exists to catch.
///
/// The anti-vacuity is the ASYMMETRY between two INDEPENDENT facts: the gate,
/// and whether the sibling crates the inventories walk are on disk. Neither a
/// published crate nor a copied crate directory has them; the repository does.
/// They may not disagree.
#[test]
fn the_out_of_workspace_skip_does_not_fire_inside_the_repository() {
    let here = Path::new(env!("CARGO_MANIFEST_DIR"));
    if !has_sibling_crates(here) {
        // A published crate (no `tests/`, so this never runs there) or a crate
        // directory copied out of the tree (which keeps its tests and loses its
        // siblings). Both are what the skip serves.
        eprintln!(
            "skip: no sibling crates beside {}, so there is no tree to inventory",
            here.display()
        );
        return;
    }
    assert!(
        !skip_out_of_workspace(),
        "the gate every inventory arm calls reports an out-of-workspace build while the \
         sibling crates those arms walk are on disk beside {}. Every one of them is \
         returning early and passing without inventorying anything.",
        here.display()
    );
    // And the thing the arms actually need: a file named relative to the root
    // must read back. A gate that says yes while reads fail would be worse than
    // one that says no.
    assert!(
        read_at("crates/cerulion_core/tests/upstream_waivers_test.rs").contains("WAIVED_2034"),
        "the workspace root resolves but this very file cannot be read back through \
         it, so every inventory arm would fail for the wrong reason"
    );
}

/// The negative control: the gate calls an out-of-tree layout out of tree.
///
/// Without this the arm above is one-sided. It proves the gate says "in the
/// repository" here, and a gate hardwired to say that would satisfy it. This
/// builds the other case on disk and watches the same predicates answer.
///
/// The scratch layout is what `cargo package --list` reports for this crate: a
/// manifest and `src/`, no `tests/`, no siblings, and no workspace two levels up.
#[test]
fn the_probe_calls_an_out_of_tree_layout_out_of_tree() {
    let scratch = std::env::temp_dir().join(format!(
        "cer_waiver_probe_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("a clock after 1970")
            .as_nanos()
    ));
    // `<scratch>/outer/cerulion_core-1.0.0` mirrors an unpacked crate: the
    // directory two levels up is `<scratch>`, which carries no manifest.
    let crate_dir = scratch.join("outer").join("cerulion_core-1.0.0");
    std::fs::create_dir_all(crate_dir.join("src")).expect("create the scratch crate");
    std::fs::write(
        crate_dir.join("Cargo.toml"),
        "[package]\nname = \"cerulion_core\"\nversion = \"1.0.0\"\n",
    )
    .expect("write the scratch manifest");
    std::fs::write(crate_dir.join("src/lib.rs"), "").expect("write the scratch lib");

    assert!(
        looks_unpacked(&crate_dir),
        "a crate with no `[workspace]` manifest two levels up must read as out of tree"
    );
    assert!(
        !has_sibling_crates(&crate_dir),
        "the scratch layout has no sibling crates, or this control proves nothing"
    );
    // The repository answers the other way on the same two predicates, which is
    // what makes this a control rather than a second assertion about nothing.
    let here = Path::new(env!("CARGO_MANIFEST_DIR"));
    assert!(
        has_sibling_crates(here) && !looks_unpacked(here),
        "the repository and the out-of-tree layout must not read the same"
    );

    std::fs::remove_dir_all(&scratch).expect("remove the scratch directory");
}

/// The 2035 arms subtract the known loans AND keep a bound on the remainder.
#[test]
fn every_declared_2035_arm_subtracts_the_known_loans_and_keeps_its_bound() {
    if skip_out_of_workspace() {
        return;
    }
    let mut total = 0usize;
    for (file, arms) in WAIVED_2035 {
        let src = read_at(file);
        let uses = src.matches("upstream_2035_publish_allocs(").count();
        assert_eq!(
            uses, *arms,
            "{file} is declared to carry {arms} subtraction(s) for upstream 2035 and carries \
             {uses}. An arm that stopped subtracting has either been fixed upstream, in which \
             case delete the waiver, or has quietly dropped its bound, which is worse."
        );
        assert!(
            src.contains("2035"),
            "{file} subtracts the waived allocation without naming upstream 2035 anywhere."
        );
        // Subtracting without bounding the remainder is a gate that measures and
        // then says nothing, which is the shape a widened arm takes: the call
        // site stays, the assertion stops discriminating. Read the assertion.
        let sites = subtraction_sites(&src);
        assert_eq!(
            sites.len(),
            *arms,
            "{file} carries {uses} call(s) to the waiver helper but {} of them bind a name this \
             check can follow. A site that does not bind its waived amount cannot be read, and an \
             unreadable site is an unchecked one.",
            sites.len()
        );
        for (name, window) in sites {
            if let Err(why) = bound_verdict(&window, &name) {
                panic!("{file}: the arm binding `{name}` no longer keeps its bound: {why}");
            }
        }
        total += uses;
    }
    assert_eq!(
        total, 10,
        "the 2035 waiver covers ten subtraction sites across nine arms (the wide level arm \
         subtracts twice, once per width). A change in that count needs a change here and a \
         reason in the commit."
    );
}

/// The 2035 inventory equals what the workspace actually carries, in BOTH
/// directions. Same reason as the 2034 one.
#[test]
fn the_2035_inventory_equals_what_the_workspace_carries() {
    if skip_out_of_workspace() {
        return;
    }
    let mut found: Vec<(String, usize)> = Vec::new();
    for file in every_rust_source() {
        let n = read_at(&file)
            .matches("upstream_2035_publish_allocs(")
            .count();
        if n > 0 {
            found.push((file, n));
        }
    }
    found.sort();
    let mut declared: Vec<(String, usize)> = WAIVED_2035
        .iter()
        .map(|(f, n)| ((*f).to_string(), *n))
        .collect();
    declared.sort();
    assert_eq!(
        found, declared,
        "the 2035 subtraction sites in the workspace and the inventory in this file disagree. An \
         undeclared site is a gate that was quietly widened; a declared site that no longer \
         exists is a stale line."
    );
}

/// The subtraction is one constant this tree owns, not a literal sprinkled
/// about, so upstream's fix retires the whole waiver by zeroing it.
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
