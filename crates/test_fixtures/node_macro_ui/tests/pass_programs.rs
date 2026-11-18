// SPDX-License-Identifier: AGPL-3.0-only
//! Runs every compile-pass program in `tests/ui/pass/`.
//!
//! Each program is an example target of this package, so `cargo test -p
//! node_macro_ui` compiles it as `cargo build` compiles a package that uses the
//! macros: no `cfg(test)` and no test harness. A program that stops compiling fails that
//! build. This test runs each built program and requires `main` to return
//! success, which is what trybuild's pass mode checked.
//!
//! `cargo test --test pass_programs` alone does not build examples, so run the
//! whole package: `cargo test -p node_macro_ui`.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The programs, by file stem, written out so a program that is dropped from
/// the build or from this run is a failure rather than a smaller set.
const PROGRAMS: [&str; 12] = [
    "determinism_allow_non_deterministic_passes",
    "determinism_fs_read_dir_warns",
    "determinism_method_receiver_no_false_positive",
    "determinism_uses_live_io_passes",
    "determinism_warn_class_compiles",
    "external_source_host_driven_passes",
    "generic_node_state_bounds_pass",
    "param_fields_pass",
    "port_field_hoisted_before_macro_passes",
    "port_write_in_result_closure_passes",
    "shim_assign_and_fill_from_passes",
    "state_derive_shapes",
];

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// `target/<profile>/examples`, derived from this test binary's own path
/// (`target/<profile>/deps/<binary>`), so it follows any `CARGO_TARGET_DIR`.
fn examples_dir() -> PathBuf {
    let exe = std::env::current_exe().expect("this test binary's own path");
    let deps = exe
        .parent()
        .expect("the directory holding this test binary");
    assert_eq!(
        deps.file_name().and_then(|name| name.to_str()),
        Some("deps"),
        "{} is not in a cargo `deps` directory, so the examples cannot be found beside it",
        exe.display()
    );
    deps.parent()
        .expect("the profile directory above `deps`")
        .join("examples")
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

/// The `[[example]]` entries the manifest declares under `tests/ui/pass/`, by
/// file stem, each checked to be named `pass_<stem>`, the name the run below
/// looks for.
fn declared_examples() -> BTreeSet<String> {
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
                "the [[example]] for tests/ui/pass/{stem}.rs must be named pass_{stem}"
            );
            stems_seen.insert(stem.to_owned());
        }
    }
    stems_seen
}

#[test]
fn the_listed_programs_are_the_pass_directory_and_the_manifest_examples() {
    let listed = stems(PROGRAMS.iter().map(|name| name.to_string()));
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
        declared_examples(),
        listed,
        "Cargo.toml's [[example]] entries and PROGRAMS differ: a program without an entry \
         is never built, and a binary left from an older build would run in its place"
    );
}

#[test]
fn every_compile_pass_program_runs_to_success() {
    let dir = examples_dir();
    let mut failures = Vec::new();
    for name in PROGRAMS {
        let path = dir.join(format!("pass_{name}{}", std::env::consts::EXE_SUFFIX));
        assert!(
            path.is_file(),
            "{} was not built: run `cargo test -p node_macro_ui`, which builds every example",
            path.display()
        );
        let source = manifest_dir().join(format!("tests/ui/pass/{name}.rs"));
        let modified = |file: &Path| {
            std::fs::metadata(file)
                .and_then(|meta| meta.modified())
                .unwrap_or_else(|e| panic!("read the modification time of {}: {e}", file.display()))
        };
        assert!(
            modified(&path) >= modified(&source),
            "{} is older than {}: run `cargo test -p node_macro_ui`, which rebuilds it",
            path.display(),
            source.display()
        );
        let output = Command::new(&path)
            .output()
            .unwrap_or_else(|e| panic!("start {}: {e}", path.display()));
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
