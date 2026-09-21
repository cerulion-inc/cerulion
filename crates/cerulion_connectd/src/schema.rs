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
    match std::fs::read(&dest) {
        Ok(existing) if existing == doc.text.as_bytes() => {
            Ok(MaterializeOutcome::AlreadyPresent(dest))
        }
        Ok(_) => {
            tracing::warn!(
                path = %shown_path,
                schema = %shown_schema,
                "cerulion connect: a DIFFERENT local schema already exists at this path — keeping \
                 the local file (not overwriting). Remove it to adopt the robot's served version."
            );
            Ok(MaterializeOutcome::ConflictKeptLocal(dest))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            std::fs::write(&dest, &doc.text)
                .map_err(|e| ConnectError::Schema(format!("{shown_path}: {e}")))?;
            tracing::info!(
                path = %shown_path,
                schema = %shown_schema,
                "cerulion connect: materialized robot-served schema into the desk store"
            );
            Ok(MaterializeOutcome::Wrote(dest))
        }
        Err(e) => Err(ConnectError::Schema(format!("{shown_path}: {e}"))),
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
