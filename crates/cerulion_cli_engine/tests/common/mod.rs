// SPDX-License-Identifier: AGPL-3.0-only
//! Shared workspace/manifest helpers for the two lint-policy gates.
//!
//! `workspace_lints_manifest_test.rs` and `library_print_ban_test.rs` ask the
//! same three questions — where is the repo root, which members does the root
//! manifest DECLARE, and which members does cargo actually RESOLVE — and each
//! used to carry its own copy of the answer. Two copies of a walk is how the
//! resolved-vs-declared gap got into one gate and not the other in the first
//! place, so the walk lives here once and both gates import it.
//!
//! # Declared vs resolved
//!
//! Cargo adds a path dependency to the workspace whether or not `[workspace]
//! members` names it. `examples/go2/nodes/go2_tf_source` is exactly that: its
//! declaring workspace is `examples/go2`, and the ROOT workspace folds it in
//! through `cerulion_viz/lib/cerulion_viz`'s path dev-dependency. So it is
//! BUILT by `cargo check --workspace` while being invisible to a walk of the
//! declared list — which is why every policy gate here has to ask cargo, not
//! the manifest text.

// Each test binary compiles this module SEPARATELY and uses a subset of it, so
// the unused half is `dead_code` in that binary — the standard `tests/common`
// shape. Scoped to this module, so the workspace-wide `dead_code = "deny"`
// stays armed for the gates themselves.
#![allow(dead_code)]

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// How long `cargo metadata` may take before a gate fails LOUDLY.
///
/// Generous on purpose: it can block on the package-cache lock behind the cargo
/// that is running these tests. A gate that hangs is worse than one that fails.
pub const METADATA_BUDGET: Duration = Duration::from_secs(120);

pub fn repo_root() -> PathBuf {
    // tests/ -> crates/cerulion_cli_engine/ -> crates/ -> repo root
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cerulion_cli_engine has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

pub fn read_manifest(path: &Path) -> toml::Value {
    let text = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    // `toml::from_str`, not `text.parse()`: `<Value as FromStr>` parses a TOML
    // *value*, not a document, and rejects `[workspace]` on line 1.
    toml::from_str::<toml::Value>(&text)
        .unwrap_or_else(|e| panic!("{} does not parse as TOML: {e}", path.display()))
}

/// The `[workspace] members` a manifest DECLARES, with trailing `/*` globs
/// expanded against the filesystem.
///
/// `workspace` is a path relative to the repo root; `""` means the root itself.
/// Entries come back relative to the REPO ROOT, so a sub-workspace's members
/// are directly comparable with the root's.
pub fn declared_members_of(root: &Path, workspace: &str) -> Vec<String> {
    let manifest_dir = if workspace.is_empty() {
        root.to_path_buf()
    } else {
        root.join(workspace)
    };
    let manifest = read_manifest(&manifest_dir.join("Cargo.toml"));
    let raw = manifest
        .get("workspace")
        .and_then(|w| w.get("members"))
        .and_then(|m| m.as_array())
        .unwrap_or_else(|| panic!("{manifest_dir:?}/Cargo.toml has no [workspace] members array"));

    let prefix = |p: &str| {
        if workspace.is_empty() {
            p.to_string()
        } else {
            format!("{workspace}/{p}")
        }
    };

    let mut out = Vec::new();
    for entry in raw {
        let entry = entry.as_str().expect("every member entry is a string");
        match entry.strip_suffix("/*") {
            None => out.push(prefix(entry)),
            Some(parent) => {
                let dir = manifest_dir.join(parent);
                let read = std::fs::read_dir(&dir).unwrap_or_else(|e| {
                    panic!("member glob `{entry}` — cannot read {}: {e}", dir.display())
                });
                // Every step here is FAIL-LOUD, because every silent one omits a
                // member and a policy gate that skipped a crate still passes.
                // `.flatten()` would drop a per-entry `io::Error`, and
                // `Path::is_file()` answers `false` for BOTH "no such file" and
                // "cannot stat it" — so an unreadable directory used to read as
                // "not a workspace member" and left that crate unchecked.
                let mut expanded: Vec<String> = Vec::new();
                for dir_entry in read {
                    let dir_entry = dir_entry.unwrap_or_else(|e| {
                        panic!(
                            "member glob `{entry}` — cannot read an entry of {}: {e}. \
                             A member that cannot be enumerated must not be silently \
                             skipped: the gates that walk this list would pass while \
                             covering less than they report.",
                            dir.display()
                        )
                    });
                    let manifest = dir_entry.path().join("Cargo.toml");
                    let is_member = match std::fs::metadata(&manifest) {
                        Ok(meta) => meta.is_file(),
                        // Genuinely absent: the directory is not a crate. The only
                        // io::Error kind that is a legitimate NO.
                        Err(e) if e.kind() == std::io::ErrorKind::NotFound => false,
                        Err(e) => panic!(
                            "member glob `{entry}` — cannot stat {}: {e}. Treating \
                             this as 'not a member' would drop the crate from every \
                             gate that walks this list, silently.",
                            manifest.display()
                        ),
                    };
                    if is_member {
                        expanded.push(prefix(&format!(
                            "{parent}/{}",
                            dir_entry.file_name().to_string_lossy()
                        )));
                    }
                }
                expanded.sort();
                assert!(
                    !expanded.is_empty(),
                    "member glob `{entry}` expanded to nothing — the walk would \
                     silently skip that whole subtree"
                );
                out.extend(expanded);
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// The ROOT workspace's declared members.
pub fn declared_members(root: &Path) -> Vec<String> {
    declared_members_of(root, "")
}

/// `cargo metadata --no-deps` over the root manifest, BOUNDED.
///
/// Both streams go to temp FILES rather than pipes: a pipe of cargo metadata's
/// output (hundreds of KB here) fills long before the child exits, and a parent
/// polling `try_wait` without draining it would deadlock — so the bound has to
/// come with somewhere for the bytes to go.
pub fn cargo_metadata_json(root: &Path) -> String {
    cargo_metadata_json_at(root, "")
}

/// `cargo metadata --no-deps` over `<workspace>/Cargo.toml`, BOUNDED.
///
/// `workspace` is relative to the repo root; `""` is the root workspace itself.
pub fn cargo_metadata_json_at(root: &Path, workspace: &str) -> String {
    run_cargo_metadata(root, workspace, true)
}

/// `cargo metadata` over `<workspace>/Cargo.toml` WITH the resolved dependency
/// graph, BOUNDED.
///
/// The `--no-deps` form above reports only workspace members, which is all the
/// membership walks need. Deriving which third-party packages belong to a
/// dependency FAMILY needs the resolved graph: `Cargo.lock` carries names and
/// versions but no `repository`, so it cannot say which crates a tree publishes.
pub fn cargo_metadata_json_with_deps(root: &Path, workspace: &str) -> String {
    run_cargo_metadata(root, workspace, false)
}

fn run_cargo_metadata(root: &Path, workspace: &str, no_deps: bool) -> String {
    let manifest = if workspace.is_empty() {
        root.join("Cargo.toml")
    } else {
        root.join(workspace).join("Cargo.toml")
    };
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let out = tempfile::NamedTempFile::new().expect("temp file for cargo metadata stdout");
    let err = tempfile::NamedTempFile::new().expect("temp file for cargo metadata stderr");
    let (out_handle, err_handle) = (
        out.reopen().expect("reopen stdout temp file"),
        err.reopen().expect("reopen stderr temp file"),
    );

    // `--locked` ALWAYS; `--offline` only on the `--no-deps` form. The two halves
    // of `--frozen` are not equally safe here, and that is MEASURED in CI rather
    // than reasoned about:
    //
    //  * LOCKED, always. `cargo metadata` WITH the resolve graph may WRITE
    //    `Cargo.lock`. A test that silently updates the lockfile of the tree it is
    //    checking has a side effect on its own subject; `--locked` turns a lockfile
    //    that would need updating into a loud failure. It needs no network of its
    //    own, so it costs nothing here.
    //  * OFFLINE, only where it cannot need the network. A `--no-deps` read walks
    //    manifests and never resolves, so it can never want a `.crate` file.
    //
    //    The RESOLVE form can, and it failed in CI for exactly that reason:
    //    `failed to download android_system_properties v0.1.5 … attempting to make
    //    an HTTP request, but --frozen was specified`. A full resolve covers every
    //    TARGET in the graph, including platforms this machine never builds, so it
    //    wants crates that `cargo build --workspace` on Linux never downloads —
    //    an Android-only dependency being the one that surfaced it. Adding
    //    `--offline` there would make this gate's verdict depend on how warm the
    //    runner's registry cache happens to be, which is precisely what a gate must
    //    not do. A slower online resolve beats a fast one that reds on a cold
    //    container.
    let mut args: Vec<&str> = vec!["metadata", "--format-version", "1", "--locked"];
    if no_deps {
        args.push("--offline");
        args.push("--no-deps");
    }
    let mut child = Command::new(&cargo)
        .args(&args)
        .arg("--manifest-path")
        .arg(&manifest)
        .stdout(Stdio::from(out_handle))
        .stderr(Stdio::from(err_handle))
        .spawn()
        .unwrap_or_else(|e| {
            panic!(
                "cannot run `{cargo} metadata`: {e}\n\nThese gates ask CARGO which \
                 members it resolves, because the declared `[workspace] members` \
                 list does not name path-dependency-only members. If cargo is \
                 genuinely unavailable in this environment, that is a broken \
                 gate, not a passing one."
            )
        });

    let deadline = Instant::now() + METADATA_BUDGET;
    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) => {
                if Instant::now() >= deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    panic!(
                        "`{cargo} metadata` did not finish within {METADATA_BUDGET:?} \
                         — killed. It most likely blocked on the cargo \
                         package-cache lock."
                    );
                }
                std::thread::sleep(Duration::from_millis(25));
            }
            Err(e) => panic!("cannot wait for `{cargo} metadata`: {e}"),
        }
    };

    let stdout = std::fs::read_to_string(out.path()).expect("read cargo metadata stdout");
    assert!(
        status.success(),
        "`{cargo} metadata --locked` failed ({status}). The likely cause before you \
         suspect the gate: `Cargo.lock` needs updating — run a build and commit the \
         lock. `--locked` is deliberate (see the flag comment above): this walk must \
         not rewrite the lockfile of the tree it is checking. If the message instead \
         mentions `--offline`, note that only the `--no-deps` form carries it, \
         because a resolve can legitimately need to download.\n--- stderr ---\n{}",
        std::fs::read_to_string(err.path()).unwrap_or_default()
    );
    stdout
}

/// Every member cargo RESOLVES, as a path relative to the repo root.
pub fn resolved_members(root: &Path) -> Vec<String> {
    let json: serde_json::Value =
        serde_json::from_str(&cargo_metadata_json(root)).expect("cargo metadata emits JSON");

    let ids: BTreeSet<&str> = json["workspace_members"]
        .as_array()
        .expect("cargo metadata has workspace_members")
        .iter()
        .map(|v| v.as_str().expect("a member id is a string"))
        .collect();

    let mut out: Vec<String> = json["packages"]
        .as_array()
        .expect("cargo metadata has packages")
        .iter()
        .filter(|p| ids.contains(p["id"].as_str().expect("a package id is a string")))
        .map(|p| {
            let manifest = Path::new(p["manifest_path"].as_str().expect("manifest_path is a str"));
            let dir = manifest
                .parent()
                .expect("a manifest has a parent directory");
            // `manifest_path` is absolute; make it root-relative so it can be
            // compared with the declared `[workspace] members` entries.
            dir.strip_prefix(root)
                .unwrap_or_else(|_| {
                    panic!("resolved member {} is outside the repo root", dir.display())
                })
                .to_string_lossy()
                .into_owned()
        })
        .collect();
    out.sort();
    out.dedup();
    out
}

/// The workspace that DECLARES `member`: the nearest ancestor whose
/// `Cargo.toml` carries a `[workspace]` table, relative to the repo root.
///
/// Used to check that a cross-workspace member's OTHER workspace is one we
/// know about, so the mirrored-table inventory cannot silently go stale.
pub fn owning_workspace(root: &Path, member: &str) -> Option<String> {
    let mut dir = root.join(member);
    while dir.pop() && dir.starts_with(root) {
        let manifest = dir.join("Cargo.toml");
        if manifest.is_file() && read_manifest(&manifest).get("workspace").is_some() {
            let rel = dir
                .strip_prefix(root)
                .expect("still under the repo root")
                .to_string_lossy()
                .into_owned();
            return Some(rel);
        }
        if dir == root {
            break;
        }
    }
    None
}

/// True iff `<member>/Cargo.toml` carries `[lints] workspace = true`.
pub fn inherits_workspace_lints(root: &Path, member: &str) -> bool {
    read_manifest(&root.join(member).join("Cargo.toml"))
        .get("lints")
        .and_then(|l| l.get("workspace"))
        .and_then(|w| w.as_bool())
        .unwrap_or(false)
}

/// The `[workspace.lints]` table of `<workspace>/Cargo.toml`, if it has one.
pub fn workspace_lints_table(root: &Path, workspace: &str) -> Option<toml::Value> {
    let path = if workspace.is_empty() {
        root.join("Cargo.toml")
    } else {
        root.join(workspace).join("Cargo.toml")
    };
    read_manifest(&path)
        .get("workspace")
        .and_then(|w| w.get("lints"))
        .cloned()
}

/// The crate-root source file of `member`'s library target, if it has one.
///
/// Reads `[lib] path` rather than assuming `src/lib.rs`: several crates here
/// declare that key explicitly, and a gate that hardcodes the conventional path
/// would silently skip any crate that moved it — reporting "no library target"
/// for a crate that has one is a false PASS.
pub fn lib_target_path(root: &Path, member: &str) -> Option<PathBuf> {
    let dir = root.join(member);
    let declared = read_manifest(&dir.join("Cargo.toml"))
        .get("lib")
        .and_then(|l| l.get("path"))
        .and_then(|p| p.as_str())
        .map(|p| dir.join(p));
    match declared {
        Some(p) if p.is_file() => Some(p),
        // A declared-but-missing `[lib] path` is a broken manifest, not a crate
        // without a library — say so rather than silently reporting "no lib".
        Some(p) => panic!("{member}/Cargo.toml declares [lib] path = {p:?}, which does not exist"),
        None => {
            let conventional = dir.join("src").join("lib.rs");
            conventional.is_file().then_some(conventional)
        }
    }
}
