// SPDX-License-Identifier: AGPL-3.0-only
//! Every crate this workspace publishes must carry, inside its own directory
//! and inside its own crates.io package, the full text of the license it is
//! published under.
//!
//! WHY THIS IS A TEST AND NOT A CONVENTION. A crates.io download is a
//! self-contained tarball: a reader of `cerulion_core` 1.0.0 has the crate
//! directory and nothing else. The `license = "AGPL-3.0-only"` field is an
//! SPDX identifier, not a grant, and AGPL section 4 and the Apache-2.0 and
//! MIT terms each require the text itself to travel with a copy. Before this
//! file existed, no published crate shipped one: a `LICENSE` sat at the
//! workspace root, which is not in any package.
//!
//! And it cannot be caught by eye, because every publishable crate declares
//! an `include` list. A file ships only if it is BOTH in the crate directory
//! AND named in that list, so the two halves can drift apart silently in
//! either direction: a copied text that nobody included is invisible to a
//! user, and an included name whose file was moved away is a package that
//! still builds. Both halves are asserted here, for every publishable member,
//! against the canonical text at the workspace root.
//!
//! Copies, never symlinks. Cargo packages a symlink AS a link, so a symlinked
//! `LICENSE` arrives in the tarball pointing at a path the reader does not
//! have, and the text does not travel at all. That is a passing-looking
//! failure, so it gets its own complaint.
//!
//! The publishable set is read from the workspace manifest rather than
//! listed here: a crate added tomorrow is covered without anybody
//! remembering this file, which is the property that makes this a gate.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

/// One license text a crate owes its readers: the name it must carry inside
/// the crate directory, and the canonical copy (relative to the workspace
/// root) it must be byte-identical to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RequiredText {
    in_crate: &'static str,
    canonical: &'static str,
}

/// The texts a `license` field obliges a crate directory to carry.
///
/// Every arm is spelled out and an unrecognized expression PANICS rather than
/// returning an empty list. A license this function does not know is a crate
/// whose obligations nobody has worked out, and answering "nothing is
/// required" for it would turn the next dual-licensed or third-party-licensed
/// crate into a silent gap of exactly the kind this file exists to close.
fn required_license_texts(license: &str) -> Vec<RequiredText> {
    match license {
        "AGPL-3.0-only" => vec![RequiredText {
            in_crate: "LICENSE",
            canonical: "LICENSE",
        }],
        "MIT OR Apache-2.0" => vec![
            RequiredText {
                in_crate: "LICENSE-MIT",
                canonical: "docs/legal/LICENSE-MIT",
            },
            RequiredText {
                in_crate: "LICENSE-APACHE",
                canonical: "docs/legal/LICENSE-APACHE",
            },
        ],
        other => panic!(
            "a publishable crate declares license `{other}`, which this test does not know. \
             Work out which text(s) that license obliges the crate directory to carry, add \
             the arm here, and put the copies in place. Do NOT return an empty list: a \
             license with no required text is how a crate ships with no grant at all."
        ),
    }
}

/// A publishable crate as the workspace manifest and its own manifest
/// describe it.
struct Publishable {
    name: String,
    /// Relative to the workspace root, as `workspace.members` spells it.
    dir: String,
    license: String,
    /// `None` when the manifest declares no `include` key at all.
    include: Option<Vec<String>>,
}

/// The workspace root: two levels above this crate's directory.
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

fn read_manifest(path: &Path) -> toml::Table {
    let text = fs::read_to_string(path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}). This test pins what published crates carry, so \
             it must fail rather than skip",
            path.display()
        )
    });
    text.parse::<toml::Table>()
        .unwrap_or_else(|e| panic!("{} is not valid TOML: {e}", path.display()))
}

/// `license` as the crate declares it, resolving `license.workspace = true`
/// against the workspace default. `None` when the manifest declares neither.
fn resolve_license(package: &toml::Table, workspace_license: &str) -> Option<String> {
    match package.get("license") {
        Some(toml::Value::String(s)) => Some(s.clone()),
        Some(toml::Value::Table(t)) if t.get("workspace") == Some(&toml::Value::Boolean(true)) => {
            Some(workspace_license.to_string())
        }
        _ => None,
    }
}

/// Every workspace member cargo would publish: `publish = false` is out, and
/// so is a member with no `[package]` table at all.
fn publishable_members(root: &Path) -> Vec<Publishable> {
    let ws_manifest = read_manifest(&root.join("Cargo.toml"));
    let workspace = ws_manifest
        .get("workspace")
        .and_then(toml::Value::as_table)
        .expect("the root manifest has a [workspace] table");
    let workspace_license = workspace
        .get("package")
        .and_then(toml::Value::as_table)
        .and_then(|p| p.get("license"))
        .and_then(toml::Value::as_str)
        .expect("[workspace.package] declares a license")
        .to_string();
    let members = workspace
        .get("members")
        .and_then(toml::Value::as_array)
        .expect("[workspace] declares members");

    let mut out = Vec::new();
    for member in members {
        let dir = member
            .as_str()
            .expect("every workspace member is a path string")
            .to_string();
        let manifest = read_manifest(&root.join(&dir).join("Cargo.toml"));
        let Some(package) = manifest.get("package").and_then(toml::Value::as_table) else {
            continue;
        };
        if package.get("publish") == Some(&toml::Value::Boolean(false)) {
            continue;
        }
        let name = package
            .get("name")
            .and_then(toml::Value::as_str)
            .unwrap_or_else(|| panic!("{dir}/Cargo.toml declares no package name"))
            .to_string();
        let license = resolve_license(package, &workspace_license).unwrap_or_else(|| {
            panic!(
                "{name} ({dir}) is published but declares no license. Declare one (or \
                 `license.workspace = true`) before publishing it."
            )
        });
        let include = package
            .get("include")
            .and_then(toml::Value::as_array)
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_string))
                    .collect::<Vec<String>>()
            });
        out.push(Publishable {
            name,
            dir,
            license,
            include,
        });
    }
    out
}

/// Everything wrong with one crate's license texts, in the words a reader
/// needs to fix it. An empty vector is the whole contract.
///
/// `canon_root` is where the canonical texts live and `crate_dir` is the
/// directory that must carry the copies; both are parameters rather than
/// derived inside, so the positive controls below can run this exact function
/// over a scratch directory.
fn license_text_complaints(
    crate_dir: &Path,
    canon_root: &Path,
    license: &str,
    include: Option<&Vec<String>>,
) -> Vec<String> {
    let mut complaints = Vec::new();
    let required = required_license_texts(license);

    let Some(include) = include else {
        complaints.push(format!(
            "declares no `include` list, so this test cannot tell what its package carries. \
             Add one naming {}.",
            required
                .iter()
                .map(|r| r.in_crate)
                .collect::<Vec<_>>()
                .join(" and ")
        ));
        return complaints;
    };

    for req in &required {
        let copy = crate_dir.join(req.in_crate);
        match fs::symlink_metadata(&copy) {
            Err(_) => complaints.push(format!(
                "declares license `{license}` but carries no `{}`. Copy {} into the crate \
                 directory.",
                req.in_crate, req.canonical
            )),
            Ok(meta) if meta.file_type().is_symlink() => complaints.push(format!(
                "`{}` is a symlink. Cargo packages a symlink as a link, so the text would \
                 not travel in the tarball at all: replace it with a byte copy of {}.",
                req.in_crate, req.canonical
            )),
            Ok(_) => {
                let canonical = canon_root.join(req.canonical);
                let want = fs::read(&canonical).unwrap_or_else(|e| {
                    panic!("could not read the canonical {} ({e})", canonical.display())
                });
                let got = fs::read(&copy)
                    .unwrap_or_else(|e| panic!("could not read {} ({e})", copy.display()));
                if got != want {
                    complaints.push(format!(
                        "`{}` is not byte-identical to {} ({} bytes against {}). A license \
                         text is not paraphrasable: re-copy it.",
                        req.in_crate,
                        req.canonical,
                        got.len(),
                        want.len()
                    ));
                }
            }
        }

        if !include.iter().any(|entry| entry == req.in_crate) {
            complaints.push(format!(
                "`{}` is in the crate directory but not in the `include` list, so it does \
                 NOT ship. Add \"{}\" to `include`.",
                req.in_crate, req.in_crate
            ));
        }
    }
    complaints
}

#[test]
fn every_publishable_crate_ships_the_text_of_its_license() {
    let root = workspace_root();
    let members = publishable_members(&root);

    // Anti-tautology: a walk that found nothing would satisfy every assertion
    // below. The floor is the set that existed when this test was written; it
    // rises with the workspace and is never meant to be lowered.
    assert!(
        members.len() >= 17,
        "only {} publishable crates were found: the workspace manifest walk is broken, \
         and an empty walk passes every check below",
        members.len()
    );

    let mut failures: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut files_checked = 0usize;
    for m in &members {
        let complaints =
            license_text_complaints(&root.join(&m.dir), &root, &m.license, m.include.as_ref());
        files_checked += required_license_texts(&m.license).len();
        if !complaints.is_empty() {
            failures.insert(format!("{} ({})", m.name, m.dir), complaints);
        }
    }
    assert!(
        files_checked >= 19,
        "only {files_checked} license texts were required across {} crates: \
         `required_license_texts` is answering with empty lists",
        members.len()
    );

    assert!(
        failures.is_empty(),
        "published crates would ship without the text of their license:\n{}",
        failures
            .iter()
            .map(|(crate_name, cs)| format!(
                "  {crate_name}\n{}",
                cs.iter()
                    .map(|c| format!("    - {c}"))
                    .collect::<Vec<_>>()
                    .join("\n")
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// `native_ros2_messages` redistributes 254 ROS 2 interface definitions that
/// are not ours. Apache-2.0 section 4 and the BSD-3-Clause first condition
/// each require a recipient of those files to get the license text and the
/// attribution notice, so those three files are as load-bearing as the crate's
/// own `LICENSE` and are pinned separately: they are owed by what the crate
/// VENDORS, not by what its `license` field says, so the walk above cannot
/// see them.
#[test]
fn the_vendored_ros2_definitions_ship_their_upstream_licenses_and_notice() {
    let root = workspace_root();
    let dir = root.join("crates/native_ros2_messages");
    let manifest = read_manifest(&dir.join("Cargo.toml"));
    let package = manifest
        .get("package")
        .and_then(toml::Value::as_table)
        .expect("native_ros2_messages has a [package] table");
    let include: Vec<String> = package
        .get("include")
        .and_then(toml::Value::as_array)
        .expect("native_ros2_messages declares an include list")
        .iter()
        .filter_map(|v| v.as_str().map(str::to_string))
        .collect();

    // The per-package attribution table, which the NOTICE points at.
    assert!(
        include.iter().any(|e| e == "msg/"),
        "msg/README.md travels with the msg/ tree; `include` no longer names msg/"
    );

    for (in_crate, canonical) in [
        ("NOTICE", "docs/legal/NOTICE"),
        ("LICENSE-APACHE", "docs/legal/LICENSE-APACHE"),
        ("LICENSE-BSD-3-CLAUSE", "docs/legal/LICENSE-BSD-3-CLAUSE"),
    ] {
        let copy = dir.join(in_crate);
        let meta = fs::symlink_metadata(&copy).unwrap_or_else(|_| {
            panic!(
                "native_ros2_messages vendors third-party .msg files but carries no \
                 `{in_crate}`. Copy {canonical} into the crate directory."
            )
        });
        assert!(
            !meta.file_type().is_symlink(),
            "`{in_crate}` is a symlink; cargo packages a symlink as a link, so the text \
             would not travel. Replace it with a byte copy of {canonical}."
        );
        // Compared as a boolean rather than with `assert_eq!`, which would
        // dump two whole license texts into the failure output.
        let got = fs::read(&copy).expect("the copy is readable");
        let want = fs::read(root.join(canonical)).expect("the canonical text is readable");
        assert!(
            got == want,
            "`{in_crate}` is not byte-identical to {canonical} ({} bytes against {}). \
             A license text is not paraphrasable: re-copy it.",
            got.len(),
            want.len()
        );
        assert!(
            include.iter().any(|e| e == in_crate),
            "`{in_crate}` is in the crate directory but not in the `include` list, so it \
             does NOT ship. Add \"{in_crate}\" to `include`."
        );
    }
}

// ───────────────────────────── positive controls ──────────────────────────
//
// The assertions above are all of the form "this list is empty", which a
// broken check satisfies for free. Seven of the eight below drive the SAME
// `license_text_complaints` over a scratch crate directory whose faults are
// known by construction, so each failure mode is proven reachable; the last
// pins that an unknown license is refused rather than excused.

/// A scratch crate directory under the temp dir, removed on drop.
struct Scratch(PathBuf);

impl Scratch {
    fn new(tag: &str) -> Self {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir =
            std::env::temp_dir().join(format!("cer_license_{tag}_{}_{n}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("the scratch crate directory is creatable");
        Scratch(dir)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn include_of(manifest: &str) -> Vec<String> {
    manifest
        .parse::<toml::Table>()
        .expect("the scratch manifest is valid TOML")
        .get("package")
        .and_then(toml::Value::as_table)
        .and_then(|p| p.get("include"))
        .and_then(toml::Value::as_array)
        .expect("the scratch manifest declares an include list")
        .iter()
        .filter_map(|v| v.as_str().map(str::to_string))
        .collect()
}

#[test]
fn a_scratch_crate_that_carries_and_includes_its_license_has_no_complaints() {
    let root = workspace_root();
    let scratch = Scratch::new("ok");
    fs::copy(root.join("LICENSE"), scratch.0.join("LICENSE")).expect("the copy lands");
    let manifest = r#"
[package]
name = "scratch"
license = "AGPL-3.0-only"
include = ["src/", "Cargo.toml", "README.md", "LICENSE"]
"#;
    assert_eq!(
        license_text_complaints(
            &scratch.0,
            &root,
            "AGPL-3.0-only",
            Some(&include_of(manifest))
        ),
        Vec::<String>::new(),
        "a correct crate must produce NO complaints, or every assertion above is vacuous"
    );
}

#[test]
fn a_scratch_manifest_that_omits_the_license_from_include_fails() {
    let root = workspace_root();
    let scratch = Scratch::new("noinclude");
    // The file IS in the directory: only the include list is wrong, which is
    // the half a human reading the crate directory cannot see.
    fs::copy(root.join("LICENSE"), scratch.0.join("LICENSE")).expect("the copy lands");
    let manifest = r#"
[package]
name = "scratch"
license = "AGPL-3.0-only"
include = ["src/", "Cargo.toml", "README.md"]
"#;
    let complaints = license_text_complaints(
        &scratch.0,
        &root,
        "AGPL-3.0-only",
        Some(&include_of(manifest)),
    );
    assert_eq!(complaints.len(), 1, "got {complaints:?}");
    assert!(
        complaints[0].contains("not in the `include` list"),
        "got {complaints:?}"
    );
}

#[test]
fn a_scratch_crate_missing_the_license_file_fails() {
    let root = workspace_root();
    let scratch = Scratch::new("missing");
    let manifest = r#"
[package]
name = "scratch"
license = "AGPL-3.0-only"
include = ["src/", "Cargo.toml", "README.md", "LICENSE"]
"#;
    let complaints = license_text_complaints(
        &scratch.0,
        &root,
        "AGPL-3.0-only",
        Some(&include_of(manifest)),
    );
    assert_eq!(complaints.len(), 1, "got {complaints:?}");
    assert!(
        complaints[0].contains("carries no `LICENSE`"),
        "got {complaints:?}"
    );
}

#[test]
fn a_scratch_crate_whose_license_text_was_edited_fails() {
    let root = workspace_root();
    let scratch = Scratch::new("edited");
    let mut text = fs::read(root.join("LICENSE")).expect("the canonical text is readable");
    text.push(b'\n');
    fs::write(scratch.0.join("LICENSE"), &text).expect("the edited copy lands");
    let manifest = r#"
[package]
name = "scratch"
license = "AGPL-3.0-only"
include = ["src/", "Cargo.toml", "README.md", "LICENSE"]
"#;
    let complaints = license_text_complaints(
        &scratch.0,
        &root,
        "AGPL-3.0-only",
        Some(&include_of(manifest)),
    );
    assert_eq!(complaints.len(), 1, "got {complaints:?}");
    assert!(
        complaints[0].contains("not byte-identical"),
        "got {complaints:?}"
    );
}

#[cfg(unix)]
#[test]
fn a_scratch_crate_that_symlinks_its_license_fails() {
    let root = workspace_root();
    let scratch = Scratch::new("symlink");
    std::os::unix::fs::symlink(root.join("LICENSE"), scratch.0.join("LICENSE"))
        .expect("the symlink is creatable");
    let manifest = r#"
[package]
name = "scratch"
license = "AGPL-3.0-only"
include = ["src/", "Cargo.toml", "README.md", "LICENSE"]
"#;
    let complaints = license_text_complaints(
        &scratch.0,
        &root,
        "AGPL-3.0-only",
        Some(&include_of(manifest)),
    );
    assert_eq!(complaints.len(), 1, "got {complaints:?}");
    assert!(complaints[0].contains("is a symlink"), "got {complaints:?}");
}

#[test]
fn a_dual_licensed_scratch_crate_owes_both_texts() {
    let root = workspace_root();
    let scratch = Scratch::new("dual");
    fs::copy(
        root.join("docs/legal/LICENSE-MIT"),
        scratch.0.join("LICENSE-MIT"),
    )
    .expect("the MIT copy lands");
    let manifest = r#"
[package]
name = "scratch"
license = "MIT OR Apache-2.0"
include = ["src/", "Cargo.toml", "README.md", "LICENSE-MIT"]
"#;
    let complaints = license_text_complaints(
        &scratch.0,
        &root,
        "MIT OR Apache-2.0",
        Some(&include_of(manifest)),
    );
    // Both halves of the Apache arm are missing, and the MIT arm is clean.
    assert_eq!(complaints.len(), 2, "got {complaints:?}");
    assert!(
        complaints
            .iter()
            .all(|c| c.contains("LICENSE-APACHE") || c.contains("Apache")),
        "got {complaints:?}"
    );
}

#[test]
fn a_manifest_with_no_include_list_fails_rather_than_passing() {
    let root = workspace_root();
    let scratch = Scratch::new("noinclude_key");
    fs::copy(root.join("LICENSE"), scratch.0.join("LICENSE")).expect("the copy lands");
    let complaints = license_text_complaints(&scratch.0, &root, "AGPL-3.0-only", None);
    assert_eq!(complaints.len(), 1, "got {complaints:?}");
    assert!(
        complaints[0].contains("declares no `include` list"),
        "got {complaints:?}"
    );
}

#[test]
#[should_panic(expected = "which this test does not know")]
fn an_unknown_license_expression_is_refused_rather_than_excused() {
    required_license_texts("WTFPL");
}
