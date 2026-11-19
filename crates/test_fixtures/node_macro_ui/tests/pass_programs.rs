// SPDX-License-Identifier: AGPL-3.0-only
//! Runs every compile-pass program in `tests/ui/pass/`.
//!
//! Each program is a binary target of this package, which cargo builds for
//! this test as `cargo build` compiles a package that uses the macros: no
//! `cfg(test)` and no test harness. A program that stops compiling fails that
//! build. This test runs each built program and requires `main` to return
//! success, which is what trybuild's pass mode checked.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The programs, by file stem, each with the path cargo gives its binary
/// (`pass_<stem>`), written out so a program that is dropped from the build or
/// from this run is a failure rather than a smaller set.
const PROGRAMS: [(&str, &str); 12] = [
    (
        "determinism_allow_non_deterministic_passes",
        env!("CARGO_BIN_EXE_pass_determinism_allow_non_deterministic_passes"),
    ),
    (
        "determinism_fs_read_dir_warns",
        env!("CARGO_BIN_EXE_pass_determinism_fs_read_dir_warns"),
    ),
    (
        "determinism_method_receiver_no_false_positive",
        env!("CARGO_BIN_EXE_pass_determinism_method_receiver_no_false_positive"),
    ),
    (
        "determinism_uses_live_io_passes",
        env!("CARGO_BIN_EXE_pass_determinism_uses_live_io_passes"),
    ),
    (
        "determinism_warn_class_compiles",
        env!("CARGO_BIN_EXE_pass_determinism_warn_class_compiles"),
    ),
    (
        "external_source_host_driven_passes",
        env!("CARGO_BIN_EXE_pass_external_source_host_driven_passes"),
    ),
    (
        "generic_node_state_bounds_pass",
        env!("CARGO_BIN_EXE_pass_generic_node_state_bounds_pass"),
    ),
    (
        "param_fields_pass",
        env!("CARGO_BIN_EXE_pass_param_fields_pass"),
    ),
    (
        "port_field_hoisted_before_macro_passes",
        env!("CARGO_BIN_EXE_pass_port_field_hoisted_before_macro_passes"),
    ),
    (
        "port_write_in_result_closure_passes",
        env!("CARGO_BIN_EXE_pass_port_write_in_result_closure_passes"),
    ),
    (
        "shim_assign_and_fill_from_passes",
        env!("CARGO_BIN_EXE_pass_shim_assign_and_fill_from_passes"),
    ),
    (
        "state_derive_shapes",
        env!("CARGO_BIN_EXE_pass_state_derive_shapes"),
    ),
];

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn stems(names: impl IntoIterator<Item = String>) -> BTreeSet<String> {
    names.into_iter().collect()
}

fn on_disk(dir: &Path) -> BTreeSet<String> {
    let entries = std::fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
    stems(entries.map(|entry| {
        let path = entry.expect("a directory entry").path();
        assert_eq!(
            path.extension().and_then(|ext| ext.to_str()),
            Some("rs"),
            "{} is not a Rust program; tests/ui/pass/ holds only the compile-pass programs",
            path.display()
        );
        path.file_stem()
            .and_then(|stem| stem.to_str())
            .expect("a UTF-8 file stem")
            .to_owned()
    }))
}

/// The `[[bin]]` entries the manifest declares under `tests/ui/pass/`, by file
/// stem, each checked to be named `pass_<stem>`, the name `PROGRAMS` reads.
fn declared_binaries() -> BTreeSet<String> {
    let manifest = std::fs::read_to_string(manifest_dir().join("Cargo.toml"))
        .expect("read this package's Cargo.toml");
    let mut stems_seen = BTreeSet::new();
    let mut name: Option<String> = None;
    for line in manifest.lines().map(str::trim) {
        if line.starts_with('[') {
            name = None;
        } else if let Some(value) = line.strip_prefix("name = \"") {
            name = value.strip_suffix('"').map(str::to_owned);
        } else if let Some(stem) = line
            .strip_prefix("path = \"tests/ui/pass/")
            .and_then(|rest| rest.strip_suffix(".rs\""))
        {
            assert_eq!(
                name.as_deref(),
                Some(format!("pass_{stem}").as_str()),
                "the [[bin]] for tests/ui/pass/{stem}.rs must be named pass_{stem}"
            );
            stems_seen.insert(stem.to_owned());
        }
    }
    stems_seen
}

#[test]
fn the_listed_programs_are_the_pass_directory_and_the_manifest_binaries() {
    let listed = stems(PROGRAMS.iter().map(|(name, _)| name.to_string()));
    assert_eq!(
        listed.len(),
        PROGRAMS.len(),
        "PROGRAMS names a program twice"
    );
    assert_eq!(
        on_disk(&manifest_dir().join("tests/ui/pass")),
        listed,
        "tests/ui/pass/ and PROGRAMS differ: list a new program here, or drop a removed one"
    );
    assert_eq!(
        declared_binaries(),
        listed,
        "Cargo.toml's [[bin]] entries and PROGRAMS differ: a program without an entry is never built"
    );
}

#[test]
fn every_compile_pass_program_runs_to_success() {
    let mut failures = Vec::new();
    for (name, binary) in PROGRAMS {
        let output = Command::new(binary)
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
    assert!(
        failures.is_empty(),
        "{} of {} compile-pass programs failed:\n\n{}",
        failures.len(),
        PROGRAMS.len(),
        failures.join("\n\n")
    );
}
