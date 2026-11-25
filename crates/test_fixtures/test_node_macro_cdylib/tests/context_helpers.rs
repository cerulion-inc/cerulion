// SPDX-License-Identifier: AGPL-3.0-only
//! The node context migration under the plugin (`cdylib`) expansion.
//!
//! One helper shape the migration refuses, checked by trybuild against its
//! `.stderr`, and the helper shapes that must compile: each of those is a
//! binary target of this package, which cargo builds for this test with the
//! default `cdylib` feature and no `cfg(test)`, and this test runs it.

use std::collections::BTreeSet;
use std::path::PathBuf;

/// The context-helper programs that must compile under the plugin expansion,
/// by file stem, each with the path cargo gives its binary (`pass_<stem>`).
const PROGRAMS: [(&str, &str); 3] = [
    (
        "annotated_context_helper",
        env!("CARGO_BIN_EXE_pass_annotated_context_helper"),
    ),
    (
        "data_only_helpers",
        env!("CARGO_BIN_EXE_pass_data_only_helpers"),
    ),
    (
        "qualified_context_helper",
        env!("CARGO_BIN_EXE_pass_qualified_context_helper"),
    ),
];

/// The one helper shape the migration refuses.
const REFUSED: &str = "free_context_helper";

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn a_free_helper_taking_the_native_context_is_refused() {
    let cases = trybuild::TestCases::new();
    // The stderr oracle pins rustc 1.99.1 context names; compiler upgrades
    // may require re-blessing while preserving the migration error.
    cases.compile_fail("tests/ui/free_context_helper.rs");
}

#[test]
fn the_listed_programs_are_the_ui_directory_and_the_manifest_binaries() {
    let ui = manifest_dir().join("tests/ui");
    let on_disk: BTreeSet<String> = std::fs::read_dir(&ui)
        .unwrap_or_else(|e| panic!("read {}: {e}", ui.display()))
        .map(|entry| entry.expect("a directory entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "rs"))
        .map(|path| {
            path.file_stem()
                .and_then(|stem| stem.to_str())
                .expect("a UTF-8 file stem")
                .to_owned()
        })
        .filter(|stem| stem != REFUSED)
        .collect();
    let listed: BTreeSet<String> = PROGRAMS.iter().map(|(name, _)| name.to_string()).collect();
    assert_eq!(
        on_disk, listed,
        "tests/ui/ and PROGRAMS differ: list a new program here, or drop a removed one"
    );
    let manifest = std::fs::read_to_string(manifest_dir().join("Cargo.toml"))
        .expect("read this crate's Cargo.toml");
    let mut declared = BTreeSet::new();
    let mut name: Option<String> = None;
    for line in manifest.lines().map(str::trim) {
        if line.starts_with('[') {
            name = None;
        } else if let Some(value) = line.strip_prefix("name = \"") {
            name = value.strip_suffix('"').map(str::to_owned);
        } else if let Some(stem) = line
            .strip_prefix("path = \"tests/ui/")
            .and_then(|rest| rest.strip_suffix(".rs\""))
        {
            assert_eq!(
                name.as_deref(),
                Some(format!("pass_{stem}").as_str()),
                "the [[bin]] for tests/ui/{stem}.rs must be named pass_{stem}"
            );
            declared.insert(stem.to_owned());
        }
    }
    assert_eq!(
        declared, listed,
        "Cargo.toml's [[bin]] entries and PROGRAMS differ: a program without an entry is never built"
    );
}

#[test]
fn every_context_helper_program_built_with_the_migration_runs_to_success() {
    let mut failures = Vec::new();
    for (name, binary) in PROGRAMS {
        let output = std::process::Command::new(binary)
            .output()
            .unwrap_or_else(|e| panic!("start {binary}: {e}"));
        if !output.status.success() {
            failures.push(format!(
                "{name} exited with {}\nstdout:\n{}\nstderr:\n{}",
                output.status,
                String::from_utf8_lossy(&output.stdout),
                String::from_utf8_lossy(&output.stderr)
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
