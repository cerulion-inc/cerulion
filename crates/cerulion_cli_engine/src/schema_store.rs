// SPDX-License-Identifier: AGPL-3.0-only
//! The workspace `.msg` schema store — the ament-mirror
//! `schemas/<pkg>/msg/<Type>.msg` folder.
//!
//! Every `.msg` file under `schemas/<pkg>/msg/` is parsed to a
//! [`MessageSchema`] via the SAME [`parse_rosmsg`] the built-in registry and
//! codegen use — the package name comes FROM THE PATH (`.msg` text carries no
//! package), so `schemas/sensor_msgs/msg/Imu.msg` parses as `sensor_msgs/Imu`.
//! The nested `<pkg>/msg/` layout is deliberate:
//!
//! - it mirrors an ament install prefix, so a robot-local
//!   `share/<pkg>/msg/*.msg` byte-copy lands verbatim (the harvest
//!   materializes here), and
//! - it can never collide with the flat `schemas/*.yaml` files the YAML
//!   reader handles — the store lives in per-package SUBDIRECTORIES, the YAML
//!   schemas are FILES directly under `schemas/`.
//!
//! Robustness contract (mirrors `schema_cmd::workspace_yaml_files`): a
//! missing `schemas/` dir is quiet, an unreadable dir/entry is a loud
//! `warn!` + skip, and a `.msg` that fails to PARSE is a loud `warn!` + skip
//! — never a silent skip, and never a hard abort (one bad file must not sink
//! the rest). Enumeration is sorted so the store is byte-deterministic
//! (Principle #7).
//!
//! Consumers of the store (the ONE resolution chain): attach's resolvability
//! predicate (`ros_cmd::AttachSchemaChain`), `schema list`/`schema info`, and
//! the bridge codec seed. This module is the shared reader.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use cerulion_core::codegen::{parse_rosmsg, MessageSchema};

/// One parsed `.msg` file from the store: the schema IR plus its provenance
/// path (for `schema list` and acquisition reports).
#[derive(Debug, Clone)]
pub struct StoredSchema {
    /// Parsed schema IR. Its `package` is the `<pkg>` path segment, so
    /// [`MessageSchema::qualified_name`] is `pkg/Type`.
    pub schema: MessageSchema,
    /// Store-relative provenance path `pkg/msg/Type.msg` (forward slashes,
    /// deterministic) — what listings and reports display.
    pub relative_path: String,
}

/// The workspace `.msg` schema store: every `schemas/<pkg>/msg/<Type>.msg`
/// file parsed to [`MessageSchema`], keyed by qualified name `pkg/Type`.
#[derive(Debug, Default)]
pub struct SchemaStore {
    /// Keyed by qualified name `pkg/Type`. A `BTreeMap` keeps enumeration
    /// sorted (and package-grouped — `pkg/` prefixes sort together), which
    /// every listing consumer relies on for determinism.
    by_name: BTreeMap<String, StoredSchema>,
}

impl SchemaStore {
    /// The empty store — the builtins-only baseline (no `.msg` files).
    pub fn empty() -> Self {
        Self::default()
    }

    /// Load every `<schemas_dir>/<pkg>/msg/<Type>.msg` into the store.
    ///
    /// `schemas_dir` is the workspace `schemas/` directory (the store lives
    /// in its per-package subdirs; the flat `*.yaml` files alongside are
    /// ignored here). See the module doc for the missing/unreadable/parse-fail
    /// robustness contract. Never fails: a store that could not read a single
    /// file is simply empty.
    pub fn load(schemas_dir: &Path) -> Self {
        let mut by_name = BTreeMap::new();
        for (pkg, msg_dir) in package_msg_dirs(schemas_dir) {
            for msg_path in msg_files(&msg_dir) {
                let Some(stem) = msg_path.file_stem().and_then(|s| s.to_str()) else {
                    // A non-UTF-8 stem cannot be a valid ROS type name.
                    tracing::warn!(
                        file = %msg_path.display(),
                        "schema store: .msg file has a non-UTF-8 name — skipped"
                    );
                    continue;
                };
                let text = match std::fs::read_to_string(&msg_path) {
                    Ok(text) => text,
                    Err(e) => {
                        tracing::warn!(
                            file = %msg_path.display(),
                            error = %e,
                            "schema store: could not read .msg file — skipped (the rest still load)"
                        );
                        continue;
                    }
                };
                match parse_rosmsg(&text, stem, Some(pkg.as_str())) {
                    Ok(schema) => {
                        let qualified = format!("{pkg}/{stem}");
                        let relative_path = format!("{pkg}/msg/{stem}.msg");
                        // Distinct paths cannot collide on a case-sensitive
                        // FS, so a duplicate key means a case-fold clash on
                        // macOS/Windows — later (sorted) file wins, loudly.
                        if let Some(prev) = by_name.insert(
                            qualified.clone(),
                            StoredSchema {
                                schema,
                                relative_path: relative_path.clone(),
                            },
                        ) {
                            tracing::warn!(
                                schema = %qualified,
                                kept = %relative_path,
                                dropped = %prev.relative_path,
                                "schema store: two files map to one qualified name — later file wins"
                            );
                        }
                    }
                    Err(e) => {
                        tracing::warn!(
                            file = %msg_path.display(),
                            package = %pkg,
                            schema = %stem,
                            error = ?e,
                            "schema store: failed to parse .msg — skipped (the rest still load)"
                        );
                    }
                }
            }
        }
        Self { by_name }
    }

    /// True when a qualified `pkg/Type` name is in the store.
    pub fn resolves(&self, qualified_name: &str) -> bool {
        self.by_name.contains_key(qualified_name)
    }

    /// Look up one store schema by qualified name `pkg/Type`.
    pub fn get(&self, qualified_name: &str) -> Option<&StoredSchema> {
        self.by_name.get(qualified_name)
    }

    /// Every store entry whose BARE type name (the part after `/`) equals
    /// `bare` — one per package that defines it, in sorted qualified-name
    /// order. Callers resolve a unique match and treat >1 as ambiguous
    /// (the loud-ambiguity house rule).
    pub fn matches_bare_name(&self, bare: &str) -> Vec<&StoredSchema> {
        self.by_name
            .iter()
            .filter(|(qualified, _)| {
                qualified
                    .rsplit_once('/')
                    .is_some_and(|(_, type_name)| type_name == bare)
            })
            .map(|(_, stored)| stored)
            .collect()
    }

    /// Enumerate every store entry as `(qualified_name, entry)` in sorted
    /// (package-grouped) order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &StoredSchema)> {
        self.by_name.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// Clone every store schema's IR in sorted qualified-name order — the
    /// ONE enumeration seam for consumers that fold the store into a
    /// combined schema set (the graph-run hash-divergence map, the recorded
    /// wire-size map, the replay field registry). Those folds must not
    /// hand-roll their own `schemas/<pkg>/msg/` walks — the store reader is
    /// this module (see the module doc's robustness contract).
    pub fn schemas(&self) -> Vec<MessageSchema> {
        self.by_name.values().map(|s| s.schema.clone()).collect()
    }

    /// Store entries whose qualified name equals a built-in ROS 2 message.
    /// The store copy WINS in the resolution chain (the built-in shadow
    /// semantics), so each is a loud fact for the caller to surface. Sorted
    /// qualified names.
    pub fn builtin_shadows(&self) -> Vec<String> {
        self.by_name
            .keys()
            .filter(|qualified| builtin_has_qualified(qualified.as_str()))
            .cloned()
            .collect()
    }

    /// Number of store entries.
    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    /// True when the store holds no `.msg` schemas.
    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }
}

/// Does the built-in ROS 2 registry contain the qualified name `pkg/Type`?
/// Shared by [`SchemaStore::builtin_shadows`] and the CLI shadow marking.
pub fn builtin_has_qualified(qualified: &str) -> bool {
    match qualified.split_once('/') {
        Some((pkg, msg)) if !pkg.is_empty() && !msg.is_empty() && !msg.contains('/') => {
            native_ros2_messages::BUILTIN_MSGS
                .iter()
                .any(|&(p, n, _)| p == pkg && n == msg)
        }
        _ => false,
    }
}

/// Enumerate `(package_name, <schemas_dir>/<pkg>/msg)` for every package
/// subdir that carries a `msg/` directory, in sorted package order.
///
/// A missing `schemas_dir` is quiet (`debug!` — legitimately absent, e.g. a
/// pre-store workspace); an unreadable dir or dir-entry is a loud `warn!` +
/// skip. Non-directory entries (the flat `schemas/*.yaml` files) and package
/// dirs with no `msg/` subdir are filtered, not skips.
fn package_msg_dirs(schemas_dir: &Path) -> Vec<(String, PathBuf)> {
    let entries = match std::fs::read_dir(schemas_dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            tracing::debug!(dir = %schemas_dir.display(), "schema store: no schemas/ directory");
            return Vec::new();
        }
        Err(e) => {
            tracing::warn!(
                dir = %schemas_dir.display(),
                error = %e,
                "schema store: schemas/ present but unreadable — skipping the .msg store"
            );
            return Vec::new();
        }
    };
    let mut dirs = Vec::new();
    for dirent in entries {
        let path = match dirent {
            Ok(entry) => entry.path(),
            Err(e) => {
                tracing::warn!(
                    dir = %schemas_dir.display(),
                    error = %e,
                    "schema store: unreadable directory entry — skipped"
                );
                continue;
            }
        };
        // Only per-package subdirectories participate; the flat YAML
        // schemas live directly under `schemas/` and are the YAML
        // reader's business.
        if !path.is_dir() {
            continue;
        }
        let msg_dir = path.join("msg");
        if !msg_dir.is_dir() {
            continue;
        }
        let Some(pkg) = path.file_name().and_then(|n| n.to_str()) else {
            tracing::warn!(dir = %path.display(), "schema store: non-UTF-8 package directory name — skipped");
            continue;
        };
        dirs.push((pkg.to_string(), msg_dir));
    }
    dirs.sort();
    dirs
}

/// Enumerate the `*.msg` files in one `msg/` directory, sorted. An
/// unreadable dir/entry is a loud `warn!` + skip; non-`.msg` files are a
/// filter, not a skip.
fn msg_files(msg_dir: &Path) -> Vec<PathBuf> {
    let entries = match std::fs::read_dir(msg_dir) {
        Ok(entries) => entries,
        Err(e) => {
            tracing::warn!(
                dir = %msg_dir.display(),
                error = %e,
                "schema store: msg/ directory unreadable — skipped"
            );
            return Vec::new();
        }
    };
    let mut files = Vec::new();
    for dirent in entries {
        let path = match dirent {
            Ok(entry) => entry.path(),
            Err(e) => {
                tracing::warn!(
                    dir = %msg_dir.display(),
                    error = %e,
                    "schema store: unreadable msg/ entry — skipped"
                );
                continue;
            }
        };
        if path.extension().and_then(|e| e.to_str()) == Some("msg") {
            files.push(path);
        }
    }
    files.sort();
    files
}
