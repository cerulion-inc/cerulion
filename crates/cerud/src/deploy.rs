// SPDX-License-Identifier: AGPL-3.0-only
//! Deploy-bundle TYPES + digest verification + a PURE symlink-plan
//! computation (design level; the on-robot apply half comes later).
//!
//! A deploy bundle is content-addressed: its [`BundleManifest`] lists every
//! file (graph YAML + schemas + cdylibs) with a per-file SHA-256, plus the
//! versions, schema hashes, and target glibc. The on-robot layout is
//! `<state_root>/bundles/<hash>/` with a `<state_root>/current` symlink —
//! swapping the symlink is a deploy, flipping it back is a rollback, and the
//! last N bundles are kept.
//!
//! This module ships the manifest types + serde + [`verify_bundle`] +
//! [`plan_deploy`]/[`plan_rollback`] (pure). It does NOT touch the real
//! filesystem to install anything — the apply half is not implemented here.

use std::collections::BTreeMap;
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::constants::DEFAULT_STATE_ROOT;
use crate::error::{CerudError, CerudResult};
use crate::hash::{canonical_json_bytes, sha256_file, sha256_hex};

/// The current bundle-manifest format version.
pub const MANIFEST_FORMAT_VERSION: u16 = 1;

/// The role a bundled file plays in the deployed graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BundleFileRole {
    /// A graph YAML file.
    GraphYaml,
    /// A schema `.msg`/`.yaml` file.
    Schema,
    /// A compiled node cdylib.
    Cdylib,
    /// Anything else carried in the bundle.
    Other,
}

/// One file in a bundle, with its content digest.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BundleFile {
    /// Path RELATIVE to the bundle dir (`graphs/perception.yaml`, ...).
    pub path: String,
    /// Lowercase-hex SHA-256 of the file's bytes.
    pub sha256: String,
    /// What the file is.
    pub role: BundleFileRole,
}

/// The content-addressed manifest for a deploy bundle.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BundleManifest {
    /// Manifest format version.
    pub format_version: u16,
    /// The bundle's content address: lowercase-hex SHA-256 over the sorted
    /// file digests + the version/target metadata (see [`compute_bundle_hash`]).
    pub bundle_hash: String,
    /// The Cerulion version this bundle was built against.
    pub cerulion_version: String,
    /// The target glibc version, when the bundle targets a specific one
    /// (`None` = unconstrained / statically linked).
    pub target_glibc: Option<String>,
    /// Per-topic (or per-schema) wire schema hashes, so a deploy can be
    /// rejected against a robot whose schemas drifted.
    pub schema_hashes: BTreeMap<String, u64>,
    /// Every file in the bundle.
    pub files: Vec<BundleFile>,
}

impl BundleManifest {
    /// Reject a manifest whose file paths are unsafe (absolute, or containing a
    /// `..`/root/prefix component) BEFORE any of them is joined against a
    /// bundle dir. The same guard class as `log_tail`'s traversal defense —
    /// a hostile manifest must not be able to read or write outside the bundle.
    pub fn validate_paths(&self) -> CerudResult<()> {
        for file in &self.files {
            validate_bundle_relpath(&file.path)?;
        }
        Ok(())
    }
}

/// Verify a bundle-relative path is safe to join against a bundle dir: it must
/// be a non-empty, relative path whose every component is a plain name (a bare
/// `.` is tolerated). Absolute paths and any `..`/root/prefix component are
/// refused.
///
/// This is a purely LEXICAL guard (no filesystem access). The symlink-escape
/// half — a bundle file whose on-disk type is a symlink, or that canonicalizes
/// outside the bundle dir — is enforced separately in [`verify_file_digest`]
/// (which has the bundle dir on hand). Together they match `log_tail`'s
/// lexical-then-canonical guard class.
pub fn validate_bundle_relpath(path: &str) -> CerudResult<()> {
    let unsafe_path = || {
        CerudError::Manifest(format!(
            "bundle file path '{path}' is unsafe (absolute or escapes the bundle dir)"
        ))
    };
    if path.is_empty() {
        return Err(unsafe_path());
    }
    let p = Path::new(path);
    if p.is_absolute() {
        return Err(unsafe_path());
    }
    for component in p.components() {
        match component {
            Component::Normal(_) | Component::CurDir => {}
            // ParentDir (`..`), RootDir (`/`), Prefix (`C:\`) all escape.
            _ => return Err(unsafe_path()),
        }
    }
    Ok(())
}

/// Compute a manifest's content address: SHA-256 over the sorted list of
/// `(path, sha256, role)` plus `format_version`, `cerulion_version`,
/// `target_glibc`, and the (already sorted) `schema_hashes`.
///
/// The `bundle_hash` field itself is EXCLUDED from the preimage (it is the
/// output), so a manifest can be self-verified with [`verify_bundle_hash`].
pub fn compute_bundle_hash(manifest: &BundleManifest) -> String {
    // Sort files by path for a canonical, order-independent digest.
    let mut files: Vec<&BundleFile> = manifest.files.iter().collect();
    files.sort_by(|a, b| a.path.cmp(&b.path));
    let files_json: Vec<serde_json::Value> = files
        .iter()
        .map(|f| {
            serde_json::json!({
                "path": f.path,
                "sha256": f.sha256.to_lowercase(),
                "role": f.role,
            })
        })
        .collect();

    let preimage = serde_json::json!({
        "format_version": manifest.format_version,
        "cerulion_version": manifest.cerulion_version,
        "target_glibc": manifest.target_glibc,
        "schema_hashes": manifest.schema_hashes,
        "files": files_json,
    });
    sha256_hex(&canonical_json_bytes(&preimage))
}

/// Verify a manifest's declared `bundle_hash` equals its recomputed content
/// address. A mismatch means the manifest metadata was altered.
pub fn verify_bundle_hash(manifest: &BundleManifest) -> CerudResult<()> {
    let recomputed = compute_bundle_hash(manifest);
    if !recomputed.eq_ignore_ascii_case(&manifest.bundle_hash) {
        return Err(CerudError::Manifest(format!(
            "bundle_hash mismatch: manifest declares {}, content addresses to {recomputed}",
            manifest.bundle_hash
        )));
    }
    Ok(())
}

/// Verify one bundle file's on-disk SHA-256 matches its manifest digest.
/// `bundle_dir` is the directory the manifest's relative paths resolve under.
///
/// Two traversal guards run BEFORE the digest read: the lexical
/// [`validate_bundle_relpath`] (rejects `../`/absolute), then a SYMLINK guard —
/// content-addressed bundles must contain no symlinks, so a file whose on-disk
/// type is a symlink is refused, and the canonicalized path must stay within
/// the (canonicalized) bundle dir (defeats an intermediate symlink escaping it).
pub fn verify_file_digest(bundle_dir: &Path, file: &BundleFile) -> CerudResult<()> {
    validate_bundle_relpath(&file.path)?;
    let full = bundle_dir.join(&file.path);

    // Reject a symlink final component outright (content-addressed bundles hold
    // content, not pointers). `symlink_metadata` does NOT follow the final link.
    match std::fs::symlink_metadata(&full) {
        Ok(md) if md.file_type().is_symlink() => {
            return Err(CerudError::Manifest(format!(
                "bundle file '{}' is a symlink; content-addressed bundles must contain no symlinks",
                file.path
            )));
        }
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(CerudError::Manifest(format!(
                "bundle file '{}' is missing from the bundle dir",
                file.path
            )));
        }
        Err(e) => return Err(e.into()),
    }

    // Canonicalize and assert the resolved path stays within the bundle dir —
    // catches an intermediate (parent-component) symlink escaping the bundle.
    let canon_dir = bundle_dir.canonicalize()?;
    let canon_full = full.canonicalize()?;
    if !canon_full.starts_with(&canon_dir) {
        return Err(CerudError::Manifest(format!(
            "bundle file '{}' resolves outside the bundle dir (symlink traversal)",
            file.path
        )));
    }

    let actual = sha256_file(&canon_full)?;
    if !actual.eq_ignore_ascii_case(&file.sha256) {
        return Err(CerudError::DigestMismatch {
            path: file.path.clone(),
            expected: file.sha256.to_lowercase(),
            actual,
        });
    }
    Ok(())
}

/// Verify a whole bundle: safe file paths, the manifest's content address, AND
/// every file's on-disk digest. Returns the first failure.
pub fn verify_bundle(bundle_dir: &Path, manifest: &BundleManifest) -> CerudResult<()> {
    manifest.validate_paths()?;
    verify_bundle_hash(manifest)?;
    for file in &manifest.files {
        verify_file_digest(bundle_dir, file)?;
    }
    Ok(())
}

// ─────────────────────────────── Symlink plan ──────────────────────────────

/// The on-robot bundle directory layout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BundleLayout {
    /// The state root (default `/var/lib/cerulion`).
    pub state_root: PathBuf,
    /// How many bundles to retain (the newest N by install time).
    pub keep_last: usize,
}

impl BundleLayout {
    /// A layout under an explicit state root.
    pub fn new(state_root: impl Into<PathBuf>, keep_last: usize) -> Self {
        BundleLayout {
            state_root: state_root.into(),
            keep_last,
        }
    }

    /// The `bundles/` directory holding one dir per bundle hash.
    pub fn bundles_dir(&self) -> PathBuf {
        self.state_root.join("bundles")
    }

    /// The directory for a specific bundle hash.
    pub fn bundle_dir(&self, hash: &str) -> PathBuf {
        self.bundles_dir().join(hash)
    }

    /// The `current` symlink that points at the live bundle.
    pub fn current_link(&self) -> PathBuf {
        self.state_root.join("current")
    }
}

impl Default for BundleLayout {
    fn default() -> Self {
        BundleLayout::new(DEFAULT_STATE_ROOT, 3)
    }
}

/// An installed bundle on the robot (a dir under `bundles/`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InstalledBundle {
    /// The bundle's content-address hash (its dir name).
    pub hash: String,
    /// When it was installed (ns since epoch), for keep-last ordering.
    pub installed_at_ns: u64,
}

/// A computed, side-effect-free plan for a symlink swap.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SymlinkPlan {
    /// The `current` symlink to (re)point.
    pub current_link: PathBuf,
    /// The bundle dir the symlink will point at.
    pub target_dir: PathBuf,
    /// The hash the symlink will point to (== the deploy/rollback target).
    pub target_hash: String,
    /// The hash `current` pointed at before this plan (the rollback anchor).
    pub previous_hash: Option<String>,
    /// Bundle hashes to prune (delete) to honor `keep_last`. Never contains
    /// the target or the previous hash.
    pub prune: Vec<String>,
}

/// Plan a deploy: repoint `current` at `new`, then prune to `keep_last`.
///
/// Pure. `installed` is the set already on disk (excluding `new`), `current`
/// is the hash `current` points at now (if any). The prune list keeps the
/// newest `keep_last` bundles by install time, but NEVER prunes the new target
/// or the previous `current` (the rollback anchor stays available).
pub fn plan_deploy(
    layout: &BundleLayout,
    installed: &[InstalledBundle],
    current: Option<&str>,
    new: &InstalledBundle,
) -> SymlinkPlan {
    // Dedupe (hash -> newest install time), folding in the new bundle.
    let mut by_hash: BTreeMap<String, u64> = BTreeMap::new();
    for b in installed {
        let e = by_hash.entry(b.hash.clone()).or_insert(b.installed_at_ns);
        if b.installed_at_ns > *e {
            *e = b.installed_at_ns;
        }
    }
    let e = by_hash
        .entry(new.hash.clone())
        .or_insert(new.installed_at_ns);
    if new.installed_at_ns > *e {
        *e = new.installed_at_ns;
    }

    // Newest-first by install time (hash breaks ties for determinism).
    let mut ordered: Vec<(String, u64)> = by_hash.into_iter().collect();
    ordered.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));

    let retained: std::collections::BTreeSet<String> = ordered
        .iter()
        .take(layout.keep_last.max(1))
        .map(|(h, _)| h.clone())
        .collect();

    // Prune everything not retained, but protect the target + rollback anchor.
    let prune: Vec<String> = ordered
        .iter()
        .map(|(h, _)| h.clone())
        .filter(|h| !retained.contains(h) && h != &new.hash && Some(h.as_str()) != current)
        .collect();

    SymlinkPlan {
        current_link: layout.current_link(),
        target_dir: layout.bundle_dir(&new.hash),
        target_hash: new.hash.clone(),
        previous_hash: current.map(|s| s.to_string()),
        prune,
    }
}

/// Plan a rollback: repoint `current` at an already-installed `target`.
///
/// Pure. Errors if `target` is not among `installed` (you cannot roll back to
/// a bundle that is not on disk). A rollback prunes nothing.
pub fn plan_rollback(
    layout: &BundleLayout,
    installed: &[InstalledBundle],
    current: Option<&str>,
    target: &str,
) -> CerudResult<SymlinkPlan> {
    if !installed.iter().any(|b| b.hash == target) {
        return Err(CerudError::Manifest(format!(
            "cannot roll back to '{target}': it is not among the installed bundles"
        )));
    }
    Ok(SymlinkPlan {
        current_link: layout.current_link(),
        target_dir: layout.bundle_dir(target),
        target_hash: target.to_string(),
        previous_hash: current.map(|s| s.to_string()),
        prune: Vec::new(),
    })
}
