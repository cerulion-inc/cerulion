// SPDX-License-Identifier: AGPL-3.0-only
//! Build a [`FrameWalker`] over every built-in ROS 2 schema, plus
//! the bridge config's workspace `.msg` store.
//!
//! `native_ros2_messages::BUILTIN_MSGS` is the exact `.msg` text the
//! generated types compiled against, so a walker built from it
//! decodes any built-in topic byte-identically to the generated readers.
//! This is the generic-path substrate: a schema-driven sink resolves an
//! incoming frame's `WireHeader.schema_hash` → schema → typed fields with
//! zero per-message code.
//!
//! The BUILTINS-ONLY walker cannot decode the acquired / store-only types the
//! `ros2 attach` viz node wires (their `schema_hash` is unknown → the sink's
//! warn-once unknown-hash drop — precisely the acquisition headline
//! rendering NOTHING). [`walker_from_bridge_config_env`] closes that gap: it
//! reads the SAME bridge config the `dds_bridge` loads (via
//! `$DDS_BRIDGE_CONFIG`, absolute at the attach auto-run handoff), extracts
//! its `msg_dirs` (relative entries resolved against the CONFIG FILE's
//! directory — the bridge's own contract), and seeds the walker with the
//! store schemas, so the two halves of the pipeline agree on the schema
//! universe.

use std::path::{Path, PathBuf};

use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::{SchemaDoc, SchemaEncoding};

/// The env var naming the bridge's mapping-config file. A LITERAL mirror of
/// `dds_bridge`'s `config::CONFIG_ENV` (this support lib cannot depend on the
/// node crate); the two must stay in sync.
pub const BRIDGE_CONFIG_ENV: &str = "DDS_BRIDGE_CONFIG";

/// Parse every `BUILTIN_MSGS` entry into a [`MessageSchema`] — the shared base
/// every viz walker starts from. Any `.msg` that fails to parse is skipped with a
/// loud warning (a corrupt built-in is a codegen bug, not a runtime condition) so
/// the base still covers the rest. Pure — no I/O beyond the embedded corpus.
fn builtin_schemas() -> Vec<MessageSchema> {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (package, name, text) in native_ros2_messages::BUILTIN_MSGS {
        match parse_rosmsg(text, name, Some(package)) {
            Ok(schema) => schemas.push(schema),
            Err(e) => {
                tracing::warn!(
                    package,
                    name,
                    error = ?e,
                    "cerulion_viz: failed to parse a built-in .msg; skipping it in the walker",
                );
            }
        }
    }
    schemas
}

/// Finish a walker over `schemas`, surfacing any resolution warnings. The single
/// `FrameWalker::new` seam every constructor in this module funnels through.
fn finish_walker(schemas: Vec<MessageSchema>) -> FrameWalker {
    let (walker, warnings) = FrameWalker::new(schemas);
    for w in &warnings {
        tracing::warn!(warning = %w, "cerulion_viz: frame-walker schema resolution warning");
    }
    walker
}

/// Build a [`FrameWalker`] over every built-in ROS 2 schema. Resolution warnings
/// from the walker are surfaced.
pub fn builtin_walker() -> FrameWalker {
    finish_walker(builtin_schemas())
}

/// Whether `qualified` (`pkg/Type`) names a BUILT-IN ROS 2 type — one every
/// Cerulion binary already compiles in.
///
/// The daemon's `schemas` side-load verb refuses a definition for one.
/// Docs are folded into the walker LAST and the layout resolver is
/// last-insert-wins (see [`walker_with_store_and_docs`]), so accepting a
/// caller-supplied `sensor_msgs/Image` would redefine that type DAEMON-WIDE,
/// including for live robot taps the caller knows nothing about. The refusal is
/// unconditional because the BLAST RADIUS is unconditional: one caller's doc
/// changing every consumer's decoding is never an acceptable trade, whatever the
/// caller's intent.
///
/// **Such a doc CAN legitimately arrive, and the refusal is still right.**
/// The claim that "the bag recorder never ships built-in text" is FALSE, and
/// is enforced by no code path. A workspace that SHADOWS a built-in
/// (`schemas/sensor_msgs/msg/Image.msg`) is a
/// first-class, documented configuration whose store copy WINS at resolution, so
/// a robot recording under that workspace publishes frames stamped with the
/// SHADOW's hash and `cerulion bag record` ships the shadow's text — correctly,
/// since it is the only definition that explains the recorded hash. That doc is
/// still refused here, and the actual consequence (the topic decodes for
/// `topic echo` / `bag info` but renders nothing in Studio) is reported to the
/// operator by `cerulion bag info` / `bag play` rather than promised away — see
/// `cerulion_cli_engine::bag_cmd::bag_doc_is_viewable`, which mirrors this
/// predicate for exactly that reason.
///
/// A NAME lookup over the embedded registry, not a walker build — cheap enough
/// to run per offered doc.
pub fn is_builtin_schema_name(qualified: &str) -> bool {
    let Some((pkg, ty)) = qualified.split_once('/') else {
        return false;
    };
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .any(|(p, t, _)| *p == pkg && *t == ty)
}

/// Every built-in schema (as [`builtin_walker`]) PLUS the workspace `.msg`
/// store schemas under `msg_dirs`. Store schemas are
/// parsed with the SAME parser and appended LAST, so a store definition wins
/// over a colliding built-in (the layout resolver's by-qualified map is
/// last-insert-wins — matching the bridge codec's shadow semantics).
/// An empty slice is byte-identical to [`builtin_walker`].
pub fn walker_with_store(msg_dirs: &[PathBuf]) -> FrameWalker {
    walker_with_store_and_docs(msg_dirs, &[])
}

/// [`walker_with_store`] PLUS robot-served schema-closure docs (the remote
/// arm) — builtins ∪ store(`msg_dirs`) ∪ `docs`, so a CUSTOM type a robot serves
/// over the network (fetched via the `schema` verb) becomes decodable and
/// renderable by the daemon's walker. Docs are appended LAST, so a robot-served
/// definition wins a rare qualified-name collision with a store/built-in entry
/// (the by-qualified layout resolver is last-insert-wins, matching the bridge
/// codec's shadow ladder). `docs.is_empty()` is byte-identical to [`walker_with_store`].
///
/// Each `msg`-encoded doc is parsed via `parse_rosmsg`; a `yaml`-encoded doc is
/// warn-SKIPPED — the daemon carries no workspace-YAML schema parser (that lives
/// in the CLI engine, which the daemon must not depend on), and robot-served ROS
/// types are `.msg`. A doc that fails to parse is skipped with a `warn!` (the rest
/// still seed) — one bad doc never takes down the walker.
pub fn walker_with_store_and_docs(msg_dirs: &[PathBuf], docs: &[SchemaDoc]) -> FrameWalker {
    let mut schemas: Vec<MessageSchema> = builtin_schemas();
    let mut store_count = 0usize;
    for dir in msg_dirs {
        for schema in read_store_schemas(dir) {
            store_count += 1;
            schemas.push(schema);
        }
    }
    if store_count > 0 {
        tracing::info!(
            count = store_count,
            dirs = ?msg_dirs,
            "cerulion_viz: workspace .msg store schemas joined the frame walker (bridge \
             msg_dirs) — store-only topics are decodable"
        );
    }
    let mut doc_count = 0usize;
    for doc in docs {
        match doc.encoding {
            SchemaEncoding::Msg => {
                let (pkg, ty) = match doc.qualified.split_once('/') {
                    Some((p, t)) => (Some(p), t),
                    None => (None, doc.qualified.as_str()),
                };
                match parse_rosmsg(&doc.text, ty, pkg) {
                    Ok(schema) => {
                        doc_count += 1;
                        schemas.push(schema);
                    }
                    Err(e) => tracing::warn!(
                        qualified = %doc.qualified, error = ?e,
                        "cerulion_viz: could not parse a robot-served .msg doc — skipped (the \
                         type stays undecodable, the rest still seed)"
                    ),
                }
            }
            SchemaEncoding::Yaml => tracing::warn!(
                qualified = %doc.qualified,
                "cerulion_viz: a robot-served YAML schema doc cannot be seeded by the viz \
                 daemon (no workspace-YAML parser) — skipped; the type stays undecodable"
            ),
        }
    }
    if doc_count > 0 {
        tracing::info!(
            count = doc_count,
            "cerulion_viz: robot-served schema-closure docs joined the frame walker \
             — custom remote types are decodable"
        );
    }
    finish_walker(schemas)
}

/// Read every `<store_dir>/<pkg>/msg/<Type>.msg` into a parsed
/// [`MessageSchema`] — the viz-side mirror of the bridge's store reader (this
/// lib cannot depend on the `dds_bridge` node crate). Same robustness
/// contract: every degraded shape is a `warn!` + skip, never a hard failure
/// (one bad file must not take down the rest of the walker's schema set).
fn read_store_schemas(store_dir: &Path) -> Vec<MessageSchema> {
    let mut out: Vec<MessageSchema> = Vec::new();
    let entries = match std::fs::read_dir(store_dir) {
        Ok(e) => e,
        Err(e) => {
            tracing::warn!(
                dir = %store_dir.display(),
                error = %e,
                "cerulion_viz store: msg_dirs entry unreadable — skipped"
            );
            return out;
        }
    };
    let mut pkg_dirs: Vec<(String, PathBuf)> = Vec::new();
    for dirent in entries {
        let entry = match dirent {
            Ok(e) => e,
            Err(e) => {
                tracing::warn!(
                    dir = %store_dir.display(),
                    error = %e,
                    "cerulion_viz store: unreadable directory entry — skipped"
                );
                continue;
            }
        };
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let msg_dir = path.join("msg");
        if !msg_dir.is_dir() {
            continue;
        }
        let Some(pkg) = path.file_name().and_then(|n| n.to_str()) else {
            tracing::warn!(
                path = %path.display(),
                "cerulion_viz store: non-UTF-8 package directory name — skipped"
            );
            continue;
        };
        pkg_dirs.push((pkg.to_string(), msg_dir));
    }
    pkg_dirs.sort();
    for (pkg, msg_dir) in pkg_dirs {
        let mut files: Vec<PathBuf> = Vec::new();
        match std::fs::read_dir(&msg_dir) {
            Ok(entries) => {
                for dirent in entries {
                    match dirent {
                        Ok(d) => {
                            let p = d.path();
                            if p.extension().and_then(|x| x.to_str()) == Some("msg") {
                                files.push(p);
                            }
                        }
                        Err(e) => tracing::warn!(
                            dir = %msg_dir.display(),
                            error = %e,
                            "cerulion_viz store: unreadable directory entry — skipped"
                        ),
                    }
                }
            }
            Err(e) => {
                tracing::warn!(
                    dir = %msg_dir.display(),
                    error = %e,
                    "cerulion_viz store: msg/ directory unreadable — skipped"
                );
                continue;
            }
        }
        files.sort();
        for f in files {
            let Some(stem) = f.file_stem().and_then(|s| s.to_str()) else {
                tracing::warn!(
                    file = %f.display(),
                    "cerulion_viz store: .msg file has a non-UTF-8 name — skipped"
                );
                continue;
            };
            let text = match std::fs::read_to_string(&f) {
                Ok(t) => t,
                Err(e) => {
                    tracing::warn!(
                        file = %f.display(),
                        error = %e,
                        "cerulion_viz store: could not read .msg — skipped"
                    );
                    continue;
                }
            };
            match parse_rosmsg(&text, stem, Some(pkg.as_str())) {
                Ok(s) => out.push(s),
                Err(e) => tracing::warn!(
                    file = %f.display(),
                    error = ?e,
                    "cerulion_viz store: failed to parse .msg — skipped"
                ),
            }
        }
    }
    out
}

/// Extract a bridge config's `msg_dirs` list, RELATIVE entries resolved
/// against the CONFIG FILE's directory — the exact resolution contract the
/// bridge itself applies at load. Any failure (file
/// unreadable, YAML malformed, `msg_dirs` not a string list) is a loud `warn!`
/// + empty result — the walker then degrades to builtins-only, never a crash.
fn bridge_config_msg_dirs(config_path: &Path) -> Vec<PathBuf> {
    let text = match std::fs::read_to_string(config_path) {
        Ok(t) => t,
        Err(e) => {
            tracing::warn!(
                path = %config_path.display(),
                error = %e,
                "cerulion_viz: could not read the bridge config — walker stays builtins-only"
            );
            return Vec::new();
        }
    };
    let value: serde_yaml::Value = match serde_yaml::from_str(&text) {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!(
                path = %config_path.display(),
                error = %e,
                "cerulion_viz: bridge config is not valid YAML — walker stays builtins-only"
            );
            return Vec::new();
        }
    };
    let Some(dirs) = value.get("msg_dirs") else {
        return Vec::new(); // no store — today's builtins-only behavior.
    };
    let Some(seq) = dirs.as_sequence() else {
        tracing::warn!(
            path = %config_path.display(),
            "cerulion_viz: bridge config msg_dirs is not a list — walker stays builtins-only"
        );
        return Vec::new();
    };
    let base = config_path.parent().unwrap_or(Path::new(""));
    let mut out = Vec::new();
    for d in seq {
        let Some(s) = d.as_str() else {
            tracing::warn!(
                path = %config_path.display(),
                "cerulion_viz: bridge config msg_dirs entry is not a string — skipped"
            );
            continue;
        };
        let p = PathBuf::from(s);
        if p.is_relative() && !base.as_os_str().is_empty() {
            out.push(base.join(p));
        } else {
            out.push(p);
        }
    }
    out
}

/// [`walker_with_store`] seeded from the bridge config named by `config_path`
/// (the path half of [`walker_from_bridge_config_env`], split out so tests
/// need no process-global env mutation).
pub fn walker_from_bridge_config_path(config_path: &Path) -> FrameWalker {
    walker_with_store(&bridge_config_msg_dirs(config_path))
}

/// The sink's production walker constructor: when
/// [`BRIDGE_CONFIG_ENV`] is set (the attach auto-run exports it, absolute),
/// seed the walker from that config's `msg_dirs` store so the viz node can
/// decode the SAME store-only types the bridge transcodes; unset = the
/// builtins-only walker (today's behavior — a graph run without the bridge
/// config env has no store to point at).
pub fn walker_from_bridge_config_env() -> FrameWalker {
    walker_from_bridge_config_env_with_docs(&[])
}

/// [`walker_from_bridge_config_env`] PLUS robot-served schema-closure `docs`
/// (the remote arm): the daemon rebuilds its walker through THIS the moment a
/// schema-less remote attach fetches a custom type's `.msg` closure over the
/// network, so the fetched type becomes decodable/renderable. The base is
/// re-derived from [`BRIDGE_CONFIG_ENV`] EXACTLY as [`walker_from_bridge_config_env`]
/// derives it (the env is fixed for a process lifetime, so the rebuilt base is
/// byte-identical to the daemon's initial walker), then `docs` are folded in.
/// `docs.is_empty()` is byte-identical to [`walker_from_bridge_config_env`].
pub fn walker_from_bridge_config_env_with_docs(docs: &[SchemaDoc]) -> FrameWalker {
    let dirs = match std::env::var(BRIDGE_CONFIG_ENV) {
        Ok(path) => bridge_config_msg_dirs(Path::new(&path)),
        Err(_) => Vec::new(),
    };
    walker_with_store_and_docs(&dirs, docs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtin_walker_knows_the_go2_demo_schemas() {
        let walker = builtin_walker();
        assert!(walker.knows("sensor_msgs/PointCloud2"));
        assert!(walker.knows("sensor_msgs/CompressedImage"));
        assert!(walker.knows("geometry_msgs/Twist"));
        assert!(walker.knows("std_msgs/Header"));
    }

    /// The predicate the daemon's side-load refusal is built on had
    /// no test of its own — it existed as a definition plus exactly one call
    /// site, and the e2e arm that named it could not see it (see
    /// `vizd_e2e_test::accepted_counts_what_the_walker_took_and_builtins_are_refused`).
    ///
    /// Hand oracle over the embedded registry, and the NEGATIVE arms carry the
    /// weight: neutering the predicate to `|_| true` would refuse every custom
    /// type a bag ships (the whole feature), and to `|_| false` would re-open the
    /// daemon-wide redefinition it exists to stop. Both are killed here.
    #[test]
    fn is_builtin_schema_name_matches_the_embedded_registry_and_nothing_else() {
        // Built-ins from three different packages, incl. the one the daemon's
        // doc comment names and the one the e2e impostor names.
        for q in [
            "sensor_msgs/Image",
            "geometry_msgs/Vector3",
            "std_msgs/Header",
            "nav_msgs/Path",
        ] {
            assert!(is_builtin_schema_name(q), "{q} IS a vendored built-in");
        }
        // A CUSTOM type — the class a bag legitimately ships — must pass through.
        for q in [
            "go/LowState",
            "acme/Widget",
            "offer_probe/Good",
            // A custom type in a package that ALSO holds built-ins: the match is
            // per `(pkg, Type)`, never per package, or one vendored `sensor_msgs`
            // type would blanket-refuse a robot's whole `sensor_msgs` namespace.
            "sensor_msgs/NotAVendoredType",
        ] {
            assert!(!is_builtin_schema_name(q), "{q} is NOT a built-in");
        }
        // Malformed names answer `false` rather than panicking or half-matching:
        // the input is caller-supplied over the control socket.
        for q in ["", "Image", "/Image", "sensor_msgs/", "a/b/c"] {
            assert!(
                !is_builtin_schema_name(q),
                "{q:?} is not a resolvable `pkg/Type`"
            );
        }
    }

    /// Write `<store_dir>/<pkg>/msg/<Type>.msg` = `text` — a fake `.msg` store.
    fn write_store_msg(store_dir: &Path, pkg: &str, ty: &str, text: &str) {
        let dir = store_dir.join(pkg).join("msg");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(format!("{ty}.msg")), text).unwrap();
    }

    #[test]
    fn store_only_type_becomes_walkable() {
        // Supplement B: the store-seeded walker knows a type NEITHER built-in
        // nor UNITREE — the exact acquired-type class the builtins-only walker
        // silently dropped (unknown schema_hash → warn-once → nothing renders).
        let store = tempfile::tempdir().unwrap();
        write_store_msg(store.path(), "acme", "Widget", "int32 id\nfloat64 value\n");
        let dirs = vec![store.path().to_path_buf()];

        let walker = walker_with_store(&dirs);
        assert!(walker.knows("acme/Widget"), "store type must be walkable");
        // The builtins-only walker does NOT (proves the store is load-bearing).
        assert!(!builtin_walker().knows("acme/Widget"));
        // Builtins still covered (the store ADDS, never replaces).
        assert!(walker.knows("sensor_msgs/PointCloud2"));
    }

    #[test]
    fn walker_seeds_from_bridge_config_with_config_relative_msg_dirs() {
        // Supplement B end-to-end at the path seam: a bridge config in
        // <ws>/graphs/ naming `../schemas` (the attach-emitted value) seeds the
        // walker with the <ws>/schemas store — resolved against the CONFIG
        // FILE's directory exactly as the bridge does (supplement A), so it
        // works from any CWD.
        let ws = tempfile::tempdir().unwrap();
        write_store_msg(&ws.path().join("schemas"), "acme", "Widget", "int32 id\n");
        let graphs = ws.path().join("graphs");
        std::fs::create_dir_all(&graphs).unwrap();
        let config_path = graphs.join("attach.bridge.yaml");
        std::fs::write(
            &config_path,
            "domain_id: 0\nonly_networks: []\nmsg_dirs:\n  - ../schemas\nmappings:\n  - \
             dds_topic: /widget\n    ros_type: acme/Widget\n    cerulion_topic: /go2/widget\n    \
             qos: best_effort\n",
        )
        .unwrap();

        let walker = walker_from_bridge_config_path(&config_path);
        assert!(
            walker.knows("acme/Widget"),
            "the config's ../schemas store must seed the walker"
        );

        // No-msg_dirs config = builtins-only (today's behavior, no store).
        let plain = graphs.join("plain.bridge.yaml");
        std::fs::write(&plain, "domain_id: 0\nmappings: []\n").unwrap();
        assert!(!walker_from_bridge_config_path(&plain).knows("acme/Widget"));

        // A missing/unreadable config degrades to builtins-only (loud warn),
        // never a crash.
        let gone = graphs.join("nope.bridge.yaml");
        let degraded = walker_from_bridge_config_path(&gone);
        assert!(degraded.knows("sensor_msgs/PointCloud2"));
        assert!(!degraded.knows("acme/Widget"));
    }

    /// Remote arm: a robot-served `.msg` closure doc seeds the walker so a
    /// CUSTOM type (nesting a built-in) becomes walkable AND resolvable to its wire
    /// `schema_hash` (the value the daemon feeds `register_ingress_topic`). The
    /// builtins-only walker does NOT know it (proves the doc is load-bearing).
    #[test]
    fn served_msg_doc_becomes_walkable_and_hash_resolvable() {
        let doc = SchemaDoc {
            qualified: "acme/RemoteWidget".to_string(),
            encoding: SchemaEncoding::Msg,
            // Nests a built-in (std_msgs/Header) — the base must resolve it during
            // layout computation for the custom type to become walkable.
            text: "std_msgs/Header header\nint32 id\nfloat64 value\n".to_string(),
            deps: vec![],
        };
        let walker = walker_with_store_and_docs(&[], std::slice::from_ref(&doc));
        assert!(
            walker.knows("acme/RemoteWidget"),
            "a served .msg doc makes the custom type walkable"
        );
        assert!(
            walker.schema_hash_for("acme/RemoteWidget").is_some(),
            "the seeded type resolves to the wire hash register_ingress_topic needs"
        );
        // Builtins still covered (docs ADD, never replace).
        assert!(walker.knows("sensor_msgs/PointCloud2"));
        assert!(walker.knows("std_msgs/Header"));
        // The builtins-only walker does NOT know it (the doc is load-bearing).
        assert!(!builtin_walker().knows("acme/RemoteWidget"));
        // Empty docs == the plain store walker (byte-identical behavior).
        assert!(!walker_with_store_and_docs(&[], &[]).knows("acme/RemoteWidget"));
    }

    /// A YAML-encoded served doc is warn-SKIPPED (the daemon has no workspace-YAML
    /// parser): the type stays undecodable, never a silent mis-seed.
    #[test]
    fn served_yaml_doc_is_warn_skipped() {
        let doc = SchemaDoc {
            qualified: "acme/YamlType".to_string(),
            encoding: SchemaEncoding::Yaml,
            text: "name: YamlType\nfields: []\n".to_string(),
            deps: vec![],
        };
        let walker = walker_with_store_and_docs(&[], &[doc]);
        assert!(
            !walker.knows("acme/YamlType"),
            "a YAML doc is not seeded by the viz daemon (no YAML parser)"
        );
        // Builtins unaffected.
        assert!(walker.knows("sensor_msgs/PointCloud2"));
    }

    /// The env-based rebuild path folds docs over the builtins base when no bridge
    /// config env is set (the daemon's schema-less-remote-attach rebuild seam).
    #[test]
    fn env_with_docs_folds_docs_over_builtins() {
        // No DDS_BRIDGE_CONFIG in this test's env → base is builtins-only.
        std::env::remove_var(BRIDGE_CONFIG_ENV);
        let doc = SchemaDoc {
            qualified: "acme/EnvWidget".to_string(),
            encoding: SchemaEncoding::Msg,
            text: "int32 id\n".to_string(),
            deps: vec![],
        };
        let walker = walker_from_bridge_config_env_with_docs(&[doc]);
        assert!(walker.knows("acme/EnvWidget"), "env rebuild folds the doc");
        assert!(walker.knows("geometry_msgs/Twist"), "builtins base intact");
        // Empty docs == the plain env walker.
        assert!(!walker_from_bridge_config_env_with_docs(&[]).knows("acme/EnvWidget"));
    }
}
