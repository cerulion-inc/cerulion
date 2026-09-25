// SPDX-License-Identifier: AGPL-3.0-only
//! Version-lockstep guard: the iceoryx2 dependency family must be exact-pinned AND
//! resolve to a SINGLE version across the workspace.
//!
//! The cdylib node architecture statically links `cerulion_core` (hence
//! iceoryx2) into every node `.so`. If the host runtime and a node cdylib
//! resolve different iceoryx2 sub-crate versions, the per-version
//! `iceoryx2_bb_elementary::PackageVersion` they stamp into every
//! shared-memory zero-copy connection disagrees, producing a
//! `ZeroCopyCreationError::VersionMismatch` on EVERY publisher→subscriber
//! connection — a silent data-plane death (no node fires, no copy, no
//! actionable error). Pinning `iceoryx2 = "=0.9.1"` alone is NOT enough:
//! iceoryx2's deps on its own sub-crates are loose (`0.9`), so a freshly
//! generated lock floats them to a newer 0.9.x. These tests pin the contract.
//!
//! On an intentional family bump, update `PIN` below in lockstep with
//! `cerulion_core/Cargo.toml`.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

const PIN: &str = "0.10.0";

/// Extract the version string from an inline dependency line — both the bare
/// `name = "X"` and the inline-table `name = { version = "X", .. }` forms —
/// rather than substring-matching the whole line (which a trailing comment or
/// another key carrying the literal could mask).
fn dep_version(line: &str) -> Option<&str> {
    let rest = if let Some(i) = line.find("version = \"") {
        &line[i + "version = \"".len()..]
    } else {
        &line[line.find('"')? + 1..]
    };
    rest.split('"').next()
}

/// Every `iceoryx2*` entry in `cerulion_core`'s `[dependencies]` table is an
/// exact `=PIN` pin (so a sub-crate cannot float independently of the
/// top-level crate). This is the mechanism that propagates the constraint to
/// every dependent — the binary, the benches, and a user's scaffolded node
/// workspace.
#[test]
fn all_iceoryx2_deps_are_exact_pinned() {
    let toml =
        std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml"))
            .expect("read cerulion_core/Cargo.toml");
    // Force inline form: a `[...dependencies.iceoryx2-*]` TABLE header would
    // sit outside the inline-`[dependencies]` slice below and escape the
    // scanner (silently unpinned). Reject it so every iceoryx2 dep stays
    // visible to the exact-pin check.
    assert!(
        !toml.contains("dependencies.iceoryx2"),
        "iceoryx2 deps must be declared inline, not via a `[...dependencies.iceoryx2*]` \
         table (it escapes the exact-pin scanner)"
    );
    let deps = toml
        .split("\n[dependencies]\n")
        .nth(1)
        .expect("[dependencies] section present")
        .split("\n[")
        .next()
        .unwrap();
    let want = format!("={PIN}");
    let mut count = 0usize;
    for line in deps.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let name = line.split([' ', '=']).next().unwrap_or("");
        if name.starts_with("iceoryx2") {
            count += 1;
            assert_eq!(
                dep_version(line),
                Some(want.as_str()),
                "iceoryx2 dep `{name}` must be exact-pinned `{want}` (version skew): `{line}`"
            );
        }
    }
    // iceoryx2 + iceoryx2-log + the 20-crate sub-family = 22
    // (`iceoryx2-bb-flatbuffers` joined the family in 0.10.0).
    assert!(
        count >= 22,
        "expected the full iceoryx2 family exact-pinned (>=22), found {count}"
    );
}

/// Every `iceoryx2*` package in the workspace lockfile resolves to ONE
/// version. A split here IS the host-vs-cdylib skew that kills data flow.
#[test]
fn iceoryx2_family_resolves_to_single_version() {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    // In-workspace builds MUST have a resolvable workspace lock; an out-of-repo
    // (vendored/published-crate) build legitimately has none — there, the
    // manifest `=` pins (test 1) still govern downstream resolution.
    let in_workspace = std::fs::read_to_string(manifest.join("../Cargo.toml"))
        .map(|t| t.contains("[workspace]"))
        .unwrap_or(false);
    let lock = manifest.join("../../Cargo.lock");
    let text = match std::fs::read_to_string(&lock) {
        Ok(t) => t,
        Err(_) if !in_workspace => {
            eprintln!("skip: out-of-workspace build, no lock to guard");
            return;
        }
        // In-repo but the lock is missing/unreadable: the skew guard would be
        // INERT — fail loudly rather than silently self-disable.
        Err(e) => {
            panic!("workspace Cargo.lock unreadable in-repo (skew guard would be inert): {e}")
        }
    };
    let mut versions: BTreeMap<String, String> = BTreeMap::new();
    let mut pending: Option<String> = None;
    for line in text.lines() {
        let l = line.trim();
        if let Some(n) = l.strip_prefix("name = \"") {
            pending = Some(n.trim_end_matches('"').to_string());
        } else if let Some(v) = l.strip_prefix("version = \"") {
            if let Some(n) = pending.take() {
                if n.starts_with("iceoryx2") {
                    versions.insert(n, v.trim_end_matches('"').to_string());
                }
            }
        }
    }
    assert!(
        !versions.is_empty(),
        "no iceoryx2 packages found in lockfile"
    );
    let distinct: BTreeSet<&String> = versions.values().collect();
    assert_eq!(
        distinct.len(),
        1,
        "iceoryx2 family must resolve to ONE version (skew guard); got {versions:?}"
    );
    assert_eq!(
        versions.values().next().unwrap(),
        PIN,
        "iceoryx2 family must be pinned at {PIN}"
    );
}

/// EVERY committed lockfile in the repository resolves the iceoryx2 family to
/// `PIN`, not just the workspace one.
///
/// The workspace lock is the one the test above guards, and it is not the only
/// one that governs a build. The latency benches and every shipped example are
/// their OWN workspaces with their OWN committed locks, and each path-depends
/// on `cerulion_core`. A lock left behind on the previous family resolves a
/// DIFFERENT iceoryx2 into a binary that then meets a host built from this one,
/// and since 0.9.3 the two do not collide, they PARTITION: the package version
/// is part of the global management segment's name, so each side keeps its own
/// node registry, neither sees the other's services, and nothing reports an
/// error. The symptom is a demo that builds, runs, and moves no data.
///
/// Re-resolve a stale one with `cargo metadata` in its directory, which rewrites
/// the lock without building anything.
#[test]
fn every_committed_lockfile_resolves_the_family_to_the_pin() {
    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let Ok(workspace_manifest) = std::fs::read_to_string(repo.join("Cargo.toml")) else {
        eprintln!("skip: out-of-workspace build, no repository to walk");
        return;
    };
    assert!(
        workspace_manifest.contains("[workspace]"),
        "the path two levels above this crate is not the repository root"
    );

    let mut locks = Vec::new();
    collect_lockfiles(&repo, 0, &mut locks);
    assert!(
        locks.len() >= 10,
        "expected at least the workspace lock, the two bench locks and the seven          example locks (>=10), found {} ({locks:?}) — a walk that stops finding them          is an inert guard",
        locks.len()
    );

    let mut offenders = Vec::new();
    for lock in &locks {
        let text = std::fs::read_to_string(lock).expect("a committed lockfile is readable");
        let versions = iceoryx2_versions(&text);
        if versions.is_empty() {
            // A lockfile with no iceoryx2 at all is not a skew risk.
            continue;
        }
        let distinct: BTreeSet<&String> = versions.values().collect();
        if distinct.len() != 1 || versions.values().next().map(String::as_str) != Some(PIN) {
            offenders.push(format!("{}: {versions:?}", lock.display()));
        }
    }
    assert!(
        offenders.is_empty(),
        "these lockfiles do not resolve the iceoryx2 family to exactly {PIN}: {offenders:#?}"
    );
}

/// Every `Cargo.lock` under `dir`, skipping build output and version control.
/// Depth-bounded so a symlink loop cannot turn this into a hang.
fn collect_lockfiles(dir: &PathBuf, depth: usize, out: &mut Vec<PathBuf>) {
    if depth > 4 {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
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
            collect_lockfiles(&path, depth + 1, out);
        } else if name == "Cargo.lock" {
            out.push(path);
        }
    }
}

/// Every `iceoryx2*` package in a lockfile, as name to version.
fn iceoryx2_versions(text: &str) -> BTreeMap<String, String> {
    let mut versions = BTreeMap::new();
    let mut pending: Option<String> = None;
    for line in text.lines() {
        let l = line.trim();
        if let Some(n) = l.strip_prefix("name = \"") {
            pending = Some(n.trim_end_matches('"').to_string());
        } else if let Some(v) = l.strip_prefix("version = \"") {
            if let Some(n) = pending.take() {
                if n.starts_with("iceoryx2") {
                    versions.insert(n, v.trim_end_matches('"').to_string());
                }
            }
        }
    }
    versions
}
