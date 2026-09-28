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
//! `cargo nextest` gives every test its own PROCESS, and 2034 is a per-process
//! defect: it bites the SECOND plugin graph in a process. So under nextest only
//! the arms that build more than one plugin graph inside themselves fail, and
//! that set, twenty six arms across thirteen binaries, is what is waived here. CI shards `cerulion_core`
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

/// Every arm carrying the 2034 macOS waiver, as `(path from the workspace root,
/// test name)`. Twenty six arms across thirteen binaries; the counts are
/// checked, not just written down, by the both-directions inventory below.
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
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .find(|a| a.join("Cargo.lock").exists())
        .expect("the workspace root is findable from this crate")
        .to_path_buf()
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
fn every_declared_2034_arm_carries_a_macos_scoped_ignore_that_names_the_defect() {
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

/// The waiver did NOT widen to every platform.
///
/// This reads the source rather than asking a test binary what it will run, and
/// that is a deliberate limit: libtest's `--list` prints ignored arms exactly
/// like the rest, so a list based check would report "the arm is there" and be
/// read as "the arm runs". The source scan at least says the true thing, and
/// what makes it exact is the contiguous attribute walk the 2034 verdict uses:
/// an `ignore` anywhere in the arm's own attribute run is visible here, and one
/// on a neighbour is not attributed to it.
#[test]
#[cfg(not(target_os = "macos"))]
fn on_this_platform_every_waived_arm_actually_runs() {
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

/// The 2035 arms subtract the known loans AND keep a bound on the remainder.
#[test]
fn every_declared_2035_arm_subtracts_the_known_loans_and_keeps_its_bound() {
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
