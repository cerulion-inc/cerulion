// SPDX-License-Identifier: AGPL-3.0-only
//! Materialize a robot-served schema closure into the desk's local schema store.
//!
//! # The reuse seam (reported for the record)
//!
//! `cerulion_cli_engine` materializes acquired `.msg` files into
//! `schemas/<pkg>/msg/<Type>.msg` (the `ros2 attach` store layout that
//! `SchemaStore::load` reads, which `topic echo` / `viz` decode against). Reusing
//! that code directly would pull the heavy `cerulion_cli_engine` (with its DDS /
//! rerun-adjacent deps) into the desk client. So this is the SMALLEST FAITHFUL
//! reuse: it writes the SAME store layout (`.msg` → `schemas/<pkg>/msg/`, YAML →
//! `schemas/<Type>.yaml`) from the shared `cerulion_q` [`SchemaDoc`] the robot
//! serves, so the resulting on-disk store is byte-identical to what a workspace
//! `.msg` acquisition would have produced — and a desk `SchemaStore::load` over
//! `schemas_dir` resolves the type with ZERO further work.
//!
//! It never clobbers a DIFFERING local file (that would silently overwrite the
//! user's own schema): an absent path is written, an identical path is a no-op,
//! and a conflicting path is a loud `warn!` + skip (the local file wins).
//!
//! An absent path is written whole or not at all (`write_whole`): the text goes
//! to a hidden temporary sibling, is synced, and is hard linked into place, so a
//! reader sees no file or the whole file, and a write that fails or is cut short
//! leaves no partial file to be kept later as a local edit. A file another writer
//! created meanwhile makes the link fail and is classified like any existing
//! file. Where the link fails for any other reason the text is renamed into
//! place instead, with a warning, which can replace a file created since the
//! path was read.
//!
//! # Peer text
//!
//! `SchemaDoc::qualified` is ROBOT-CONTROLLED, and this module is one of the crate's
//! producers of operator-visible text from it (the others: `worker::classify_catalog_reply`,
//! `worker::classify_demand_reply`, and every `PairError`
//! construction site in `pair`): it logs the name directly AND interpolates it into
//! `ConnectError::Schema` messages that `worker.rs` renders at WARN. Both classes go
//! through [`sanitize_peer_text`], following the same boundary rule as the classifiers —
//! **sanitize where robot bytes become the thing that gets rendered**: inside the
//! `format!` for an error (so every present and future render site is safe by
//! construction), and at the render itself for a value logged directly (there is no
//! error boundary in between).
//!
//! Note that `validate_qualified` does NOT reject control characters — `\u{1b}[2J` is
//! a perfectly ordinary `Component::Normal` — so a name that PASSES the traversal guard
//! can still carry a CSI escape into a log line. Every render below therefore sanitizes,
//! including the derived `dest` path (which embeds the raw name). The raw value stays
//! intact on `MaterializeOutcome::Rejected { qualified }` for machine consumers, exactly
//! as `ConnectSummary` keeps `catalog.robot` raw.

use std::path::{Component, Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use cerulion_core::{SchemaDoc, SchemaEncoding};
use cerulion_wireclient::epoch::sanitize_peer_text;

use crate::error::{ConnectError, ConnectResult};

/// The outcome of materializing ONE schema doc.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaterializeOutcome {
    /// A fresh file was written at this path.
    Wrote(PathBuf),
    /// The path already held byte-identical content (idempotent no-op).
    AlreadyPresent(PathBuf),
    /// The path held DIFFERENT content; the local file was kept (not clobbered).
    ConflictKeptLocal(PathBuf),
    /// The doc's robot-controlled `qualified` name was UNSAFE or invalid (path
    /// traversal / absolute / no package) — REFUSED, nothing written (the batch
    /// continues). Carries the offending name + the reason (counted + logged).
    Rejected {
        /// The offending qualified name.
        qualified: String,
        /// Why it was refused (path-traversal guard / no package).
        reason: String,
    },
}

/// Materialize every doc in a served closure into `schemas_dir`, returning the
/// per-doc outcome. Creates parent directories as needed. A `.msg` doc lands at
/// `schemas/<pkg>/msg/<Type>.msg`; a YAML doc at `schemas/<Type>.yaml`.
pub fn materialize_docs(
    schemas_dir: &Path,
    docs: &[SchemaDoc],
) -> ConnectResult<Vec<MaterializeOutcome>> {
    docs.iter()
        .map(|doc| materialize_one(schemas_dir, doc))
        .collect()
}

/// Materialize ONE doc; see [`materialize_docs`].
fn materialize_one(schemas_dir: &Path, doc: &SchemaDoc) -> ConnectResult<MaterializeOutcome> {
    // Build the dest path (validating the ROBOT-CONTROLLED `qualified` name FIRST
    // — path-traversal guard, f0). An unsafe / invalid name is REFUSED per-doc
    // (loud warn + counted `Rejected`), NEVER a silent skip and NEVER an aborted
    // batch — the legit docs in the same closure still materialize.
    let dest = match dest_path(schemas_dir, doc) {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!(
                qualified = %sanitize_peer_text(&doc.qualified),
                error = %e,
                "cerulion connect: REFUSING a robot-served schema with an unsafe/invalid name \
                 (path-traversal guard) — not writing it"
            );
            return Ok(MaterializeOutcome::Rejected {
                qualified: doc.qualified.clone(),
                reason: e.to_string(),
            });
        }
    };
    // Every render below is peer-derived: `doc.qualified` directly, and `dest` because
    // it EMBEDS that name (the traversal guard bounds where the path can point, not what
    // characters it contains). Both go through the sanitizer.
    let shown_path = sanitize_peer_text(&dest.display().to_string());
    let shown_schema = sanitize_peer_text(&doc.qualified);
    if let Some(parent) = dest.parent() {
        std::fs::create_dir_all(parent).map_err(|e| {
            ConnectError::Schema(format!(
                "{}: {e}",
                sanitize_peer_text(&parent.display().to_string())
            ))
        })?;
    }
    // An existing file is kept: identical is a no-op, differing is a conflict.
    let existing_file = |existing: Vec<u8>| {
        if existing == doc.text.as_bytes() {
            return MaterializeOutcome::AlreadyPresent(dest.clone());
        }
        tracing::warn!(
            path = %shown_path,
            schema = %shown_schema,
            "cerulion connect: a DIFFERENT local schema already exists at this path; keeping \
             the local file (not overwriting). Remove it to adopt the robot's served version."
        );
        MaterializeOutcome::ConflictKeptLocal(dest.clone())
    };
    let io_failed = |e: std::io::Error| ConnectError::Schema(format!("{shown_path}: {e}"));
    match std::fs::read(&dest) {
        Ok(existing) => Ok(existing_file(existing)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            match write_whole(&dest, doc.text.as_bytes()).map_err(io_failed)? {
                Published::Wrote => {
                    tracing::info!(
                        path = %shown_path,
                        schema = %shown_schema,
                        "cerulion connect: materialized robot-served schema into the desk store"
                    );
                    Ok(MaterializeOutcome::Wrote(dest.clone()))
                }
                // Something appeared at `dest` after the read above, or is an
                // entry the read could not follow, such as a link to a missing file.
                Published::Exists => match std::fs::read(&dest) {
                    Ok(existing) => Ok(existing_file(existing)),
                    Err(e) => Err(ConnectError::Schema(format!(
                        "{shown_path}: something at this path is kept and cannot be read: {e}"
                    ))),
                },
            }
        }
        Err(e) => Err(io_failed(e)),
    }
}

/// What [`publish`] found at the schema's path.
#[derive(Debug, PartialEq, Eq)]
enum Published {
    /// The path now holds the staged text.
    Wrote,
    /// Something already existed at the path; it was left as it was.
    Exists,
}

/// Counts the staging names this process makes; with the process id it gives
/// each call of [`staging_path`] a different name.
static STAGING_SEQ: AtomicU64 = AtomicU64::new(0);

/// How many fresh staging names [`stage`] tries before it gives up on a
/// directory where each one already exists.
const STAGING_ATTEMPTS: u32 = 8;

/// The longest file name, in bytes, a staging name is held to: the per-name
/// limit of Linux filesystems such as ext4 and tmpfs. macOS and Windows count
/// their 255 in UTF-16 units, and a name within 255 bytes never has more units
/// than bytes.
const NAME_MAX_BYTES: usize = 255;

/// A hidden sibling of `dest` for [`stage`] to create: the same directory, so
/// publishing stays on one filesystem; a dot, then as much of `dest`'s file name
/// as fits, cut at a character boundary, so the whole staging name stays within
/// [`NAME_MAX_BYTES`]; then the process id, the next [`STAGING_SEQ`] and a
/// `.tmp` extension, which no schema store reader loads. A name left by an
/// earlier process with the same id can repeat, which [`stage`] meets by
/// creating new and trying the next.
fn staging_path(dest: &Path) -> std::io::Result<PathBuf> {
    let name = dest.file_name().ok_or_else(|| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "a schema path has no file name",
        )
    })?;
    let name = name.to_string_lossy();
    let seq = STAGING_SEQ.fetch_add(1, Ordering::Relaxed);
    let suffix = format!(".{}.{seq}.tmp", std::process::id());
    // The leading dot and the suffix take their exact bytes; the name gets the rest.
    let mut end = name.len().min(NAME_MAX_BYTES - 1 - suffix.len());
    while !name.is_char_boundary(end) {
        end -= 1;
    }
    Ok(dest.with_file_name(format!(".{}{suffix}", &name[..end])))
}

/// Write `bytes` at `dest` whole or not at all: [`stage`] it, then
/// [`publish`] it.
fn write_whole(dest: &Path, bytes: &[u8]) -> std::io::Result<Published> {
    let staging = stage(dest, bytes)?;
    publish(&staging, dest)
}

/// Write `bytes` into a staging file of its own next to `dest` (created new,
/// never opened over an existing file), sync it, and return its path. A write
/// or sync that fails discards the staging file.
fn stage(dest: &Path, bytes: &[u8]) -> std::io::Result<PathBuf> {
    use std::io::Write;
    let mut attempt = 1;
    let (staging, mut file) = loop {
        let staging = staging_path(dest)?;
        match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&staging)
        {
            Ok(file) => break (staging, file),
            Err(e)
                if e.kind() == std::io::ErrorKind::AlreadyExists && attempt < STAGING_ATTEMPTS =>
            {
                attempt += 1;
            }
            Err(e) => return Err(e),
        }
    };
    let synced = file.write_all(bytes).and_then(|()| file.sync_all());
    drop(file);
    match synced {
        Ok(()) => Ok(staging),
        Err(e) => {
            discard(&staging);
            Err(e)
        }
    }
}

/// Put a synced staging file at `dest` by hard link, which fails when `dest`
/// exists: that is [`Published::Exists`], and `dest` is left as it was. A link
/// that fails for any other reason (a filesystem without hard links among them)
/// is a warning, and the staging file is renamed onto `dest` instead, which
/// replaces a file created at `dest` since it was read. The staging file is gone
/// afterwards: discarded after a link, renamed, or discarded when the rename
/// fails too.
fn publish(staging: &Path, dest: &Path) -> std::io::Result<Published> {
    publish_by(staging, dest, |from, to| std::fs::hard_link(from, to))
}

/// [`publish`] with the hard link made by `link`.
fn publish_by(
    staging: &Path,
    dest: &Path,
    link: impl FnOnce(&Path, &Path) -> std::io::Result<()>,
) -> std::io::Result<Published> {
    let published = match link(staging, dest) {
        Ok(()) => Ok(Published::Wrote),
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => Ok(Published::Exists),
        Err(link_error) => {
            tracing::warn!(
                path = %sanitize_peer_text(&dest.display().to_string()),
                error = %link_error,
                "cerulion connect: hard linking a schema file failed, so it is renamed into \
                 place, which can replace a file created at that path since it was read"
            );
            return std::fs::rename(staging, dest)
                .map(|()| Published::Wrote)
                .inspect_err(|_| discard(staging));
        }
    };
    discard(staging);
    published
}

/// Remove a staging file. A removal that fails is logged at debug, and the
/// caller's own result stands.
fn discard(staging: &Path) {
    if let Err(e) = std::fs::remove_file(staging) {
        tracing::debug!(
            path = %sanitize_peer_text(&staging.display().to_string()),
            error = %e,
            "cerulion connect: could not remove a schema staging file"
        );
    }
}

/// The store path for a doc, built from path COMPONENTS (never string-join) so the
/// layout is correct on every platform. `.msg` ⇒ `schemas/<pkg>/msg/<Type>.msg`;
/// YAML ⇒ `schemas/<Type>.yaml`.
///
/// The robot-controlled `qualified` is VALIDATED first ([`validate_qualified`] —
/// path-traversal guard, f0) so a hostile `../../../tmp/pwn/Evil` or an absolute
/// name can never make the write escape `schemas_dir`.
fn dest_path(schemas_dir: &Path, doc: &SchemaDoc) -> ConnectResult<PathBuf> {
    validate_qualified(&doc.qualified)?;
    let (pkg, ty) = split_qualified(&doc.qualified);
    match doc.encoding {
        SchemaEncoding::Msg => {
            let pkg = pkg.ok_or_else(|| {
                ConnectError::Schema(format!(
                    "served .msg schema '{}' has no package (expected 'pkg/Type')",
                    sanitize_peer_text(&doc.qualified)
                ))
            })?;
            Ok(schemas_dir.join(pkg).join("msg").join(format!("{ty}.msg")))
        }
        SchemaEncoding::Yaml => Ok(schemas_dir.join(format!("{ty}.yaml"))),
    }
}

/// f0 path-traversal guard: reject a robot-controlled `qualified` name that could
/// escape the schema store. Refuses an empty name, any `\` (a Windows separator —
/// rejected on every platform so `pkg\..\evil` can't smuggle traversal on a Unix
/// build), and any `..` / absolute-root / drive-prefix path component (via
/// `Path::components`, which resolves the platform separator). Every accepted name
/// contains ONLY `Normal`/`CurDir` components, so the built `schemas_dir.join(...)`
/// path is structurally contained — no `..`-resolution or filesystem canonicalize
/// needed (keeping it deterministic + oracle-testable over tempdirs). PURE.
///
/// Both refusal messages embed the ROBOT-CONTROLLED name and reach an operator through
/// `worker.rs`'s `error = %e` WARN line, so the name is neutered + bounded HERE — the
/// same boundary rule the reply classifiers follow (see the module docs). The guard
/// itself is unaffected: it inspects the RAW `qualified`, never the display form.
fn validate_qualified(qualified: &str) -> ConnectResult<()> {
    if qualified.is_empty() {
        return Err(ConnectError::Schema(
            "served schema has an empty qualified name".to_string(),
        ));
    }
    if qualified.contains('\\') {
        return Err(ConnectError::Schema(format!(
            "served schema name '{}' contains a backslash — refusing (path-traversal guard)",
            sanitize_peer_text(qualified)
        )));
    }
    for comp in Path::new(qualified).components() {
        match comp {
            Component::Normal(_) | Component::CurDir => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(ConnectError::Schema(format!(
                    "served schema name '{}' has a '..' / absolute / drive path component \
                     — refusing to write outside the schema store (path-traversal guard)",
                    sanitize_peer_text(qualified)
                )));
            }
        }
    }
    Ok(())
}

/// Split a qualified `pkg/Type` name. A bare `Type` (no slash) yields
/// `(None, "Type")`.
fn split_qualified(qualified: &str) -> (Option<&str>, &str) {
    match qualified.rsplit_once('/') {
        Some((pkg, ty)) => (Some(pkg), ty),
        None => (None, qualified),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn msg_doc(qualified: &str, text: &str) -> SchemaDoc {
        SchemaDoc {
            qualified: qualified.to_string(),
            encoding: SchemaEncoding::Msg,
            text: text.to_string(),
            deps: vec![],
        }
    }

    /// A `.msg` doc materializes to `schemas/<pkg>/msg/<Type>.msg` with BYTE-EXACT
    /// content — the desk store layout `SchemaStore::load` reads. Hand oracle.
    #[test]
    fn materializes_msg_to_store_layout_byte_exact() {
        let dir = tempfile::tempdir().unwrap();
        let text = "acme/Sub sub\nuint32 seq\n"; // verbatim, trailing newline preserved
        let doc = msg_doc("acme/State", text);
        let outcomes = materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap();

        let dest = dir.path().join("acme").join("msg").join("State.msg");
        assert_eq!(outcomes, vec![MaterializeOutcome::Wrote(dest.clone())]);
        // Byte-exact content (never a self-compare — the oracle is the literal text).
        assert_eq!(std::fs::read_to_string(&dest).unwrap(), text);
    }

    /// A closure of several docs (root + nested dep) all land; a YAML doc lands at
    /// `schemas/<Type>.yaml`. Hand oracle over the exact paths + contents.
    #[test]
    fn materializes_a_closure_and_yaml() {
        let dir = tempfile::tempdir().unwrap();
        let docs = vec![
            msg_doc("acme/State", "acme/Sub sub\n"),
            msg_doc("acme/Sub", "float64 x\n"),
            SchemaDoc {
                qualified: "vendor/Thing".to_string(),
                encoding: SchemaEncoding::Yaml,
                text: "name: Thing\nfields:\n  - x: float64\n".to_string(),
                deps: vec![],
            },
        ];
        let outcomes = materialize_docs(dir.path(), &docs).unwrap();
        assert!(outcomes
            .iter()
            .all(|o| matches!(o, MaterializeOutcome::Wrote(_))));

        assert_eq!(
            std::fs::read_to_string(dir.path().join("acme").join("msg").join("State.msg")).unwrap(),
            "acme/Sub sub\n"
        );
        assert_eq!(
            std::fs::read_to_string(dir.path().join("acme").join("msg").join("Sub.msg")).unwrap(),
            "float64 x\n"
        );
        assert_eq!(
            std::fs::read_to_string(dir.path().join("Thing.yaml")).unwrap(),
            "name: Thing\nfields:\n  - x: float64\n"
        );
    }

    /// Re-materializing IDENTICAL content is an idempotent no-op; a DIFFERING
    /// local file is KEPT (never clobbered). Hand oracle.
    #[test]
    fn idempotent_and_no_clobber() {
        let dir = tempfile::tempdir().unwrap();
        let doc = msg_doc("acme/State", "uint32 seq\n");
        // First write.
        assert!(matches!(
            materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap()[0],
            MaterializeOutcome::Wrote(_)
        ));
        // Second identical write is a no-op.
        assert!(matches!(
            materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap()[0],
            MaterializeOutcome::AlreadyPresent(_)
        ));
        // A conflicting local edit is KEPT (the served version does not clobber it).
        let dest = dir.path().join("acme").join("msg").join("State.msg");
        std::fs::write(&dest, "uint64 seq  # local edit\n").unwrap();
        assert!(matches!(
            materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap()[0],
            MaterializeOutcome::ConflictKeptLocal(_)
        ));
        assert_eq!(
            std::fs::read_to_string(&dest).unwrap(),
            "uint64 seq  # local edit\n",
            "the local file wins a conflict"
        );
    }

    /// The qualified names a workspace's schema store loads, and its warnings.
    fn loaded(workspace: &Path) -> (Vec<String>, Vec<String>) {
        let (set, warnings) =
            cerulion_core::dynamic::SchemaSet::from_workspace_dir(workspace).unwrap();
        let names = set
            .schemas()
            .iter()
            .map(|schema| schema.qualified_name())
            .collect();
        (names, warnings)
    }

    /// A staged schema is invisible until it is published: its path stays
    /// absent, the store loads nothing and warns about nothing, and a connect
    /// cut short at that point leaves the next materialization free to write
    /// the whole schema. The staging file sits beside its schema, hidden, with
    /// a `.tmp` extension, for a `.msg` and a YAML schema alike.
    #[test]
    fn a_staged_schema_is_invisible_until_it_is_published() {
        let workspace = tempfile::tempdir().unwrap();
        let schemas = workspace.path().join("schemas");
        let msg = schemas.join("acme").join("msg").join("Sub.msg");
        let yaml = schemas.join("Thing.yaml");
        std::fs::create_dir_all(msg.parent().unwrap()).unwrap();
        let msg_staging = stage(&msg, b"float64 x\n").unwrap();
        let yaml_staging = stage(&yaml, b"name: Thing\n").unwrap();
        for (dest, staging, text) in [
            (&msg, &msg_staging, "float64 x\n"),
            (&yaml, &yaml_staging, "name: Thing\n"),
        ] {
            assert!(!dest.exists(), "{dest:?} exists before it is published");
            assert_eq!(std::fs::read_to_string(staging).unwrap(), text);
            assert_eq!(staging.parent(), dest.parent());
            let staged_name = staging.file_name().unwrap().to_str().unwrap();
            let dest_name = dest.file_name().unwrap().to_str().unwrap();
            assert!(
                staged_name.starts_with(&format!(".{dest_name}.")) && staged_name.ends_with(".tmp"),
                "{staged_name}"
            );
        }
        assert_eq!(loaded(workspace.path()), (vec![], vec![]));

        // The connect that staged these was cut short: the next one writes.
        assert_eq!(
            materialize_docs(&schemas, &[msg_doc("acme/Sub", "float64 x\n")]).unwrap(),
            vec![MaterializeOutcome::Wrote(msg.clone())]
        );
        assert_eq!(std::fs::read_to_string(&msg).unwrap(), "float64 x\n");
        assert_eq!(loaded(workspace.path()).0, vec!["acme/Sub".to_string()]);

        assert_eq!(publish(&yaml_staging, &yaml).unwrap(), Published::Wrote);
        assert_eq!(std::fs::read_to_string(&yaml).unwrap(), "name: Thing\n");
        assert!(
            !yaml_staging.exists(),
            "publishing discards the staging file"
        );
    }

    /// Two writes of one schema stage into two files, so neither can remove or
    /// publish the other's half-written text.
    #[test]
    fn two_writes_of_one_schema_stage_apart() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("Sub.msg");
        let first = stage(&dest, b"float64 x\n").unwrap();
        let second = stage(&dest, b"float64 y\n").unwrap();
        assert_ne!(first, second);
        assert_eq!(std::fs::read_to_string(&first).unwrap(), "float64 x\n");
        assert_eq!(std::fs::read_to_string(&second).unwrap(), "float64 y\n");
    }

    /// A file created at the schema's path while the schema was staged is kept:
    /// publishing reports it and leaves its text, and discards the staging file.
    #[test]
    fn a_file_created_during_the_write_is_kept() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("Sub.msg");
        let staging = stage(&dest, b"float64 x\n").unwrap();
        std::fs::write(&dest, "float64 local\n").unwrap();
        assert_eq!(publish(&staging, &dest).unwrap(), Published::Exists);
        assert_eq!(std::fs::read_to_string(&dest).unwrap(), "float64 local\n");
        assert!(!staging.exists());
    }

    /// A write that cannot stage leaves nothing behind, and a schema path held
    /// by a directory keeps the directory and what is in it.
    #[test]
    fn a_write_that_cannot_stage_leaves_nothing_and_a_held_path_is_kept() {
        let dir = tempfile::tempdir().unwrap();
        let missing = dir.path().join("missing").join("Sub.msg");
        assert_eq!(
            write_whole(&missing, b"float64 x\n").unwrap_err().kind(),
            std::io::ErrorKind::NotFound
        );
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 0);

        let held = dir.path().join("Sub.msg");
        std::fs::create_dir(&held).unwrap();
        std::fs::write(held.join("kept"), "kept").unwrap();
        assert_eq!(
            write_whole(&held, b"float64 x\n").unwrap(),
            Published::Exists
        );
        assert_eq!(std::fs::read_to_string(held.join("kept")).unwrap(), "kept");
        let left: Vec<_> = std::fs::read_dir(dir.path())
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        assert_eq!(left, vec![std::ffi::OsString::from("Sub.msg")]);
    }

    /// A publish whose hard link and fallback rename both fail returns the
    /// rename's error and discards its staging file.
    #[test]
    fn a_publish_whose_link_and_rename_both_fail_discards_its_staging_file() {
        let dir = tempfile::tempdir().unwrap();
        let staging = stage(&dir.path().join("Sub.msg"), b"float64 x\n").unwrap();
        let unreachable = dir.path().join("missing").join("Sub.msg");
        assert_eq!(
            publish(&staging, &unreachable).unwrap_err().kind(),
            std::io::ErrorKind::NotFound
        );
        assert!(!staging.exists());
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 0);
    }

    /// A hard link that fails for a reason other than an existing path is a
    /// warning naming the path, and the staging file is renamed into place,
    /// replacing what was created at that path meanwhile.
    #[test]
    #[tracing_test::traced_test]
    fn a_link_that_fails_otherwise_renames_into_place_with_a_warning() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("Sub.msg");
        let staging = stage(&dest, b"float64 x\n").unwrap();
        std::fs::write(&dest, "float64 local\n").unwrap();
        let no_links =
            |_: &Path, _: &Path| Err(std::io::Error::from(std::io::ErrorKind::Unsupported));
        assert_eq!(
            publish_by(&staging, &dest, no_links).unwrap(),
            Published::Wrote
        );
        assert_eq!(std::fs::read_to_string(&dest).unwrap(), "float64 x\n");
        assert!(!staging.exists());
        assert!(logs_contain(
            "cerulion connect: hard linking a schema file failed, so it is renamed into place, \
             which can replace a file created at that path since it was read"
        ));
        assert!(logs_contain("Sub.msg"));
        assert!(logs_contain("WARN"));
    }

    /// A staging name embeds a short schema name whole and a long one cut at a
    /// character boundary, and stays within 255 bytes: exactly 255 for a
    /// 254-byte ASCII name, and at least 252 for a 252-byte name of 62 four-byte
    /// characters and `.msg`; a path with no file name has no staging name.
    #[test]
    fn a_staging_name_fits_the_name_limit_and_needs_a_file_name() {
        let pid = std::process::id();
        let short = staging_path(Path::new("store/Sub.msg")).unwrap();
        let short = short.file_name().unwrap().to_str().unwrap().to_owned();
        assert!(
            short.starts_with(&format!(".Sub.msg.{pid}.")) && short.ends_with(".tmp"),
            "{short}"
        );
        for (long, shortest) in [
            (format!("{}.msg", "a".repeat(250)), 255),
            (format!("{}.msg", "\u{1d538}".repeat(62)), 252),
        ] {
            let staging = staging_path(&Path::new("store").join(&long)).unwrap();
            assert_eq!(staging.parent(), Some(Path::new("store")));
            let name = staging.file_name().unwrap().to_str().unwrap();
            let suffix_at = name.rfind(&format!(".{pid}.")).unwrap();
            let embedded = &name[1..suffix_at];
            assert!(name.starts_with('.') && name.ends_with(".tmp"), "{name}");
            assert!(name.len() <= 255, "{} bytes: {name}", name.len());
            assert!(long.starts_with(embedded), "{embedded}");
            // Cut at most one character short of the limit.
            assert!(name.len() >= shortest, "{} bytes: {name}", name.len());
        }
        assert_eq!(
            staging_path(Path::new("/")).unwrap_err().kind(),
            std::io::ErrorKind::InvalidInput
        );
    }

    /// A type name of 62 four-byte characters makes a 252-byte `.msg` file name,
    /// inside every platform's limit: it is staged and written whole under its
    /// own name. A type name of 128 such characters, 516 bytes and 260 UTF-16
    /// units, is past the limit everywhere: it is refused when its path is read,
    /// before anything is staged, so the package's msg directory holds no file.
    #[test]
    fn a_multibyte_name_within_the_limit_is_written_and_one_past_it_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let at_limit = "\u{1d538}".repeat(62);
        let dest = dir
            .path()
            .join("acme")
            .join("msg")
            .join(format!("{at_limit}.msg"));
        assert_eq!(dest.file_name().unwrap().len(), 252);
        assert_eq!(
            materialize_docs(
                dir.path(),
                &[msg_doc(&format!("acme/{at_limit}"), "float64 x\n")]
            )
            .unwrap(),
            vec![MaterializeOutcome::Wrote(dest.clone())]
        );
        assert_eq!(std::fs::read_to_string(&dest).unwrap(), "float64 x\n");
        let entries = |path: &Path| -> Vec<std::ffi::OsString> {
            std::fs::read_dir(path)
                .unwrap()
                .map(|e| e.unwrap().file_name())
                .collect()
        };
        assert_eq!(
            entries(dest.parent().unwrap()),
            vec![dest.file_name().unwrap().to_owned()]
        );

        let past = tempfile::tempdir().unwrap();
        let err = materialize_docs(
            past.path(),
            &[msg_doc(
                &format!("acme/{}", "\u{1d538}".repeat(128)),
                "float64 x\n",
            )],
        )
        .unwrap_err();
        assert!(err.to_string().contains("File name too long"), "{err}");
        assert!(entries(&past.path().join("acme").join("msg")).is_empty());
    }

    /// A link to a missing file at a schema's path is left as it is: nothing is
    /// written through it and it is not replaced, and materializing fails with
    /// an error that names the schema's path and says it is kept.
    #[cfg(unix)]
    #[test]
    fn a_dangling_link_at_a_schema_path_is_left_as_it_is() {
        let dir = tempfile::tempdir().unwrap();
        let dest = dir.path().join("acme").join("msg").join("Sub.msg");
        std::fs::create_dir_all(dest.parent().unwrap()).unwrap();
        let target = dir.path().join("elsewhere.msg");
        std::os::unix::fs::symlink(&target, &dest).unwrap();
        let err = materialize_docs(dir.path(), &[msg_doc("acme/Sub", "float64 x\n")]).unwrap_err();
        assert!(
            err.to_string()
                .contains("Sub.msg: something at this path is kept and cannot be read: "),
            "{err}"
        );
        assert!(std::fs::symlink_metadata(&dest)
            .unwrap()
            .file_type()
            .is_symlink());
        assert!(!target.exists(), "nothing was written through the link");
        let left: Vec<_> = std::fs::read_dir(dest.parent().unwrap())
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        assert_eq!(left, vec![std::ffi::OsString::from("Sub.msg")]);
    }

    /// A `.msg` doc with no package is REFUSED per-doc (a `Rejected` outcome —
    /// never a mis-placed file, never an aborted batch).
    #[test]
    fn msg_without_package_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let doc = msg_doc("BareType", "uint32 x\n");
        let outcomes = materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap();
        match &outcomes[0] {
            MaterializeOutcome::Rejected { qualified, reason } => {
                assert_eq!(qualified, "BareType");
                assert!(reason.contains("no package"), "reason: {reason}");
            }
            other => panic!("expected Rejected, got {other:?}"),
        }
    }

    /// f0 HEADLINE (path-traversal): a robot-served doc whose `qualified` uses `..`
    /// to escape `schemas_dir` is REFUSED — NOTHING is written outside the store —
    /// while a legit sibling in the SAME batch still materializes. Mutation-
    /// meaningful AND SAFE: the hostile target is a RELATIVE traversal that (only if
    /// the guard were reverted) would land inside the enclosing tempdir but OUTSIDE
    /// `schemas_dir` — so a regression is CAUGHT by the escape walk without ever
    /// attempting a real system-path write. (Absolute / backslash / drive escapes
    /// are covered by `validate_qualified_oracle`, which refuses them with no write
    /// attempted at all.)
    #[test]
    fn path_traversal_qualified_is_refused_no_escape() {
        let root = tempfile::tempdir().unwrap();
        // The store is a SUBDIR of `root`, so a `../escaped/...` traversal would
        // land at `<root>/escaped/...` — INSIDE the tempdir (safe to probe), yet
        // OUTSIDE `schemas_dir` (a genuine escape of the store).
        let schemas_dir = root.path().join("schemas");
        std::fs::create_dir_all(&schemas_dir).unwrap();

        let hostile = "../escaped/Pwn"; // → <root>/escaped/msg/Pwn.msg if unguarded
        let docs = vec![
            msg_doc("acme/Good", "uint32 ok\n"),
            msg_doc(hostile, "ROBOT PAYLOAD — must never escape the store\n"),
        ];

        let outcomes = materialize_docs(&schemas_dir, &docs).unwrap();
        // The legit doc materialized.
        assert!(
            matches!(outcomes[0], MaterializeOutcome::Wrote(_)),
            "the legit sibling still materializes"
        );
        assert_eq!(
            std::fs::read_to_string(schemas_dir.join("acme").join("msg").join("Good.msg")).unwrap(),
            "uint32 ok\n"
        );
        // The hostile doc was REFUSED.
        match &outcomes[1] {
            MaterializeOutcome::Rejected { qualified, reason } => {
                assert_eq!(qualified, hostile);
                assert!(
                    reason.contains("traversal"),
                    "reason names the guard: {reason}"
                );
            }
            other => panic!("expected Rejected for {hostile:?}, got {other:?}"),
        }
        // THE ESCAPE PROOF: nothing escaped `schemas_dir` — `<root>/escaped` never
        // came into being. Reverting the guard writes `<root>/escaped/msg/Pwn.msg`
        // → this assertion flips.
        assert!(
            !root.path().join("escaped").exists(),
            "a robot payload escaped the schema store into <root>/escaped"
        );
        // And the ONLY thing under `schemas_dir` is the legit file.
        let mut files = Vec::new();
        collect_files(&schemas_dir, &mut files);
        assert_eq!(
            files.len(),
            1,
            "only the legit doc landed in the store: {files:?}"
        );
    }

    /// Recursively collect every file path under `dir` (for the escape proof).
    fn collect_files(dir: &Path, out: &mut Vec<PathBuf>) {
        let Ok(entries) = std::fs::read_dir(dir) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_files(&path, out);
            } else {
                out.push(path);
            }
        }
    }

    /// `validate_qualified` oracle: legit names pass; traversal / absolute /
    /// backslash / empty are refused.
    #[test]
    fn validate_qualified_oracle() {
        // Legit.
        assert!(validate_qualified("acme/State").is_ok());
        assert!(validate_qualified("std_msgs/Header").is_ok());
        assert!(validate_qualified("BareType").is_ok()); // no-package caught later, not here
                                                         // Refused.
        assert!(validate_qualified("../evil").is_err());
        assert!(validate_qualified("a/../../evil").is_err());
        assert!(validate_qualified("/abs/Evil").is_err());
        assert!(validate_qualified("pkg\\evil").is_err());
        assert!(validate_qualified("").is_err());
    }

    #[test]
    fn split_qualified_oracle() {
        assert_eq!(split_qualified("acme/State"), (Some("acme"), "State"));
        assert_eq!(split_qualified("Bare"), (None, "Bare"));
        assert_eq!(
            split_qualified("a/b/Deep"),
            (Some("a/b"), "Deep"),
            "rsplit keeps the deepest type segment"
        );
    }

    /// This module is a THIRD producer of operator-visible text
    /// from the ROBOT-CONTROLLED `qualified` name, and every message it builds must be
    /// neutered + bounded.
    ///
    /// Unsanitized, `validate_qualified`'s two refusals interpolate the raw name into a
    /// `ConnectError::Schema` that `worker.rs` renders as `error = %e` at WARN (ON by
    /// default — `cerulion-connectd`'s filter is `info`), so a robot serving a schema
    /// closure whose `qualified` was `"\u{1b}[2J/../evil"` would trip the traversal guard
    /// and inject a raw CSI screen-clear into the operator's terminal. The name is a
    /// path COMPONENT string, so the traversal guard does not (and should not) reject
    /// control characters — sanitizing at the message boundary is the guard.
    ///
    /// HAND ORACLES: the escape is `U+FFFD` in the message, no raw control byte
    /// survives, a megabyte name is bounded; and the GUARD's verdict is unchanged
    /// (it still refuses, still on the raw name). Dropping either
    /// `sanitize_peer_text` in `validate_qualified` fails the control-byte asserts.
    #[test]
    fn qualified_name_is_neutered_and_bounded_in_every_refusal_message() {
        // (a) The traversal arm — the exact stimulus this test pins.
        let hostile = "\u{1b}[2J/../evil";
        let err = validate_qualified(hostile).expect_err("'..' must be refused");
        let ConnectError::Schema(msg) = &err else {
            panic!("expected a Schema error, got {err:?}");
        };
        assert!(
            msg.contains("\u{fffd}[2J"),
            "the escape is neutered in the message: {msg:?}"
        );
        assert!(
            !msg.chars().any(|c| c.is_control()),
            "no raw control character reaches the operator: {msg:?}"
        );

        // (b) The backslash arm, with a CR overwrite + a newline that would forge a
        //     second log record.
        let err = validate_qualified("pkg\\a\rb\nc").expect_err("a backslash must be refused");
        let ConnectError::Schema(msg) = &err else {
            panic!("expected a Schema error, got {err:?}");
        };
        assert!(
            msg.contains("pkg\\a\u{fffd}b\u{fffd}c"),
            "neutered: {msg:?}"
        );
        assert!(!msg.chars().any(|c| c.is_control()));

        // (c) A megabyte name cannot flood the log pipeline.
        let flood = format!("{}/../evil", "z".repeat(1_000_000));
        let err = validate_qualified(&flood).expect_err("'..' must be refused");
        let ConnectError::Schema(msg) = &err else {
            panic!("expected a Schema error, got {err:?}");
        };
        assert!(
            msg.chars().count() < 700,
            "bounded, got {} chars",
            msg.chars().count()
        );
        assert!(msg.contains("…(truncated)"));

        // (d) The no-package arm (built in `dest_path`) carries the same treatment.
        let err = dest_path(Path::new("/tmp"), &msg_doc("Bare\u{1b}[2J", "x"))
            .expect_err("a bare .msg name has no package");
        let ConnectError::Schema(msg) = &err else {
            panic!("expected a Schema error, got {err:?}");
        };
        assert!(msg.contains("Bare\u{fffd}[2J"), "neutered: {msg:?}");
        assert!(!msg.chars().any(|c| c.is_control()));

        // (e) ANTI-TAUTOLOGY: the guard's VERDICT is unchanged — it still reads the raw
        //     name, so an ordinary name passes and the refusals above are real refusals
        //     (a sanitizer applied to the guard's INPUT would, e.g., let a bounded-away
        //     `..` through).
        assert!(validate_qualified("acme/State").is_ok());
        assert!(validate_qualified(&format!("{}/../evil", "z".repeat(600))).is_err());
    }

    /// The peer-derived DEST PATH is neutered before it reaches a log line too: the
    /// path embeds the robot's `qualified` name verbatim (the traversal guard bounds
    /// where the path may POINT, not which characters it may contain), so a name that
    /// legitimately passes validation can still carry a CSI escape into the
    /// materialize/conflict lines. Hand oracle over the returned `MaterializeOutcome`
    /// path (raw — machine consumers keep the real path) plus the rendered form.
    #[test]
    fn a_validation_passing_name_with_control_chars_is_neutered_at_the_render() {
        let dir = tempfile::tempdir().unwrap();
        // `\u{1b}[2J` is an ordinary `Component::Normal` — validation ACCEPTS it.
        let sneaky = "acme/St\u{1b}[2Jate";
        assert!(
            validate_qualified(sneaky).is_ok(),
            "the traversal guard does not (and need not) reject control characters"
        );
        let doc = msg_doc(sneaky, "float64 x\n");
        let outcomes = materialize_docs(dir.path(), std::slice::from_ref(&doc)).unwrap();

        // The OUTCOME carries the real (raw) path — machine consumers are unaffected.
        let MaterializeOutcome::Wrote(dest) = &outcomes[0] else {
            panic!("expected a Wrote, got {:?}", outcomes[0]);
        };
        assert!(dest.to_string_lossy().contains('\u{1b}'));

        // What the RENDER path produces carries no raw control character.
        let shown = sanitize_peer_text(&dest.display().to_string());
        assert!(!shown.chars().any(|c| c.is_control()), "{shown:?}");
        assert!(shown.contains("St\u{fffd}[2Jate"));
    }
}
