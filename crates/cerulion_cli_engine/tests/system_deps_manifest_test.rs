// SPDX-License-Identifier: AGPL-3.0-only
//! Every REAL `[package.metadata.cerulion.optional-system-deps]`
//! declaration in the repo must parse.
//!
//! The decision core is oracle-tested inside
//! `cerulion_cli_engine::system_deps`; those tests use hand-written manifests.
//! This file is the STRUCTURAL half: it WALKS the repo for `Cargo.toml` files
//! carrying the block and asserts each one parses and is complete.
//!
//! It is a walk rather than a hand-maintained list on purpose. A list
//! reproduces exactly the failure mode an earlier sweep hit — it missed
//! `rerun_sink` because nothing enumerated the thing being swept. A node that
//! adds a declaration is covered here by construction, with no test edit.
//!
//! What a failure here means: `cerulion node build <that node>` would refuse to
//! build (a malformed declaration is a hard error, deliberately — see the
//! module docs), so this catches it before a user does.

use std::path::{Path, PathBuf};

use cerulion_cli_engine::system_deps::{
    declared_default_features, parse_optional_system_deps, OptionalSystemDep,
};

/// The marker a declaration starts with. Matched against the manifest text.
const BLOCK_MARKER: &str = "[package.metadata.cerulion.optional-system-deps";

fn repo_root() -> PathBuf {
    // tests/ -> cerulion_cli_engine/ -> repo root
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_cli_engine has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

/// Every `Cargo.toml` under `dir`, skipping build/VCS noise.
fn walk_manifests(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if path.is_dir() {
            if matches!(
                name.as_ref(),
                "target" | ".git" | "node_modules" | ".claude"
            ) {
                continue;
            }
            walk_manifests(&path, out);
        } else if name == "Cargo.toml" {
            out.push(path);
        }
    }
}

/// Manifests that carry a declaration, paired with their parsed deps.
fn declared_in_repo() -> Vec<(PathBuf, Vec<OptionalSystemDep>)> {
    let mut manifests = Vec::new();
    walk_manifests(&repo_root(), &mut manifests);
    assert!(
        manifests.len() > 10,
        "the walk found only {} manifests — it is not reaching the repo \
         (looked under {})",
        manifests.len(),
        repo_root().display()
    );

    let mut out = Vec::new();
    for m in manifests {
        let text = match std::fs::read_to_string(&m) {
            Ok(t) => t,
            Err(_) => continue,
        };
        if !text.contains(BLOCK_MARKER) {
            continue;
        }
        let label = m.display().to_string();
        let deps = parse_optional_system_deps(&label, &text).unwrap_or_else(|e| {
            panic!(
                "{label} declares an optional-system-deps block that \
                 does NOT parse, so `cerulion node build` would refuse to \
                 build that node:\n{e}"
            )
        });
        assert!(
            !deps.is_empty(),
            "{label} contains the marker `{BLOCK_MARKER}` but parsed to zero \
             deps — the block is being silently skipped, which is the exact \
             failure this guard exists to prevent"
        );
        out.push((m, deps));
    }
    out
}

#[test]
fn every_declared_optional_system_dep_in_the_repo_parses_and_is_complete() {
    let declared = declared_in_repo();

    // The walk must actually find something, or this test asserts nothing.
    // camera_jpeg is the first (and at time of writing only) declaration.
    assert!(
        !declared.is_empty(),
        "no manifest in the repo declares `{BLOCK_MARKER}` — either the walk \
         is broken or the declaration was deleted. If the last declaration is \
         genuinely gone, delete this test with it."
    );

    for (manifest, deps) in &declared {
        for dep in deps {
            let where_ = manifest.display();
            assert!(
                !dep.feature.is_empty(),
                "{where_}: a dep has an empty feature name"
            );
            assert!(
                !dep.modules.is_empty(),
                "{where_}: `{}` lists no pkg-config modules",
                dep.feature
            );
            assert!(
                !dep.summary.trim().is_empty(),
                "{where_}: `{}` has a blank summary — the FOUND/NOT-FOUND \
                 notice would name no capability",
                dep.feature
            );
            assert!(
                !dep.without_it.trim().is_empty(),
                "{where_}: `{}` has a blank `without-it` — the notice would \
                 not say what breaks",
                dep.feature
            );
            assert!(
                !dep.install.is_empty(),
                "{where_}: `{}` declares no install command",
                dep.feature
            );
            for (key, cmd) in &dep.install {
                assert!(
                    !cmd.trim().is_empty(),
                    "{where_}: `{}` install.{key} is blank — a copy-pasteable \
                     line is the whole point",
                    dep.feature
                );
            }
        }
    }
}

/// The declaring crate must NOT put the gated feature in `default`.
///
/// This is the point of the whole exercise: if the feature is default, the
/// plain `cargo build` every contributor runs still needs the system library,
/// and the probe buys nothing. A regression here is silent — the build simply
/// starts failing on machines without the library — so it is pinned.
#[test]
fn a_gated_feature_is_never_in_the_declaring_crates_default_features() {
    let declared = declared_in_repo();
    assert!(
        !declared.is_empty(),
        "nothing declared — see the sibling test"
    );

    for (manifest, deps) in &declared {
        let text = std::fs::read_to_string(manifest).expect("re-read");
        let default_features = declared_default_features(&manifest.display().to_string(), &text)
            .expect("manifest parses (the sibling test already parsed it)");

        for dep in deps {
            assert!(
                !default_features.contains(&dep.feature),
                "{}: feature `{}` is gated on a system library \
                 (pkg-config {:?}) but is ALSO in `default`. That makes the \
                 library a hard prerequisite for a plain `cargo build` of the \
                 whole workspace — the breakage an earlier change removed. Drop it from \
                 `default`; `cerulion node build` re-enables it whenever the \
                 library is present.",
                manifest.display(),
                dep.feature,
                dep.modules
            );
        }
    }
}
