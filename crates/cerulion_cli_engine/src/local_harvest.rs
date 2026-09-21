// SPDX-License-Identifier: AGPL-3.0-only
//! The LOCAL half of schema acquisition: the local-ament rung.
//!
//! [`LocalAmentAcquirer`] is a filesystem-only [`SchemaAcquirer`] that harvests
//! a requested `pkg/Type`'s verbatim `.msg` — and its FULL nested closure —
//! from the host's own ROS 2 install, discovered via `$AMENT_PREFIX_PATH`. It
//! is the "Shape-B" rung: it covers a robot when `cerulion ros2 attach` runs ON
//! the robot, and any dev machine with a ROS 2 install ("local
//! `$AMENT_PREFIX_PATH` scan"). No ssh, no network — pure `share/` reads.
//!
//! # How a type is found in one prefix `P`
//!
//! 1. Prefer the ament index marker
//!    `P/share/ament_index/resource_index/rosidl_interfaces/<pkg>` — a newline
//!    list of the package's interface files (`msg/<Type>.msg`, `srv/…`, …). If
//!    it lists `msg/<Type>.msg`, read `P/share/<pkg>/msg/<Type>.msg` VERBATIM.
//! 2. If the marker is absent (some overlays install without an index) OR
//!    present-but-under-listing the type, probe `P/share/<pkg>/msg/<Type>.msg`
//!    directly (a `debug!` breadcrumb records the fallback).
//!
//! The FILE is the source of truth; the marker only steers whether to look.
//! Prefixes are walked in order, so the FIRST prefix that yields the file wins
//! (`$AMENT_PREFIX_PATH` order = overlay precedence).
//!
//! # Full nested closure
//!
//! The harvested text is parsed with the SAME [`parse_rosmsg`] the store reader
//! and codegen use; every non-built-in referenced type (walked with the SAME
//! [`ros_cmd::nested_dependencies`](crate::ros_cmd) reference logic the
//! acquisition-completeness check uses) is harvested RECURSIVELY, cycle-safe
//! (a visited set). A type served by the built-in corpus
//! ([`native_ros2_messages::BUILTIN_MSGS`]) is NEVER harvested into the bundle
//! — the driver serves it from the corpus (and the completeness walk
//! accepts it). A type is acquired only when its WHOLE closure harvested and
//! parsed; any gap (missing/unparseable member, missing nested dep) SKIPS the
//! requested type with a precise, loud reason naming the dependency chain —
//! never a half-resolved bundle.
//!
//! # Store interaction (the never-clobber floor)
//!
//! This acquirer has NO knowledge of the workspace `.msg` store — it just
//! harvests what the ROS install carries. Store files are never clobbered by a
//! re-acquired twin because the DRIVER already omits, at staging time, any
//! closure member already resolvable via the store or built-ins
//! (`ros_cmd::process_acquired`: `if chain.resolves(&q) { continue; }`), so a
//! harvested twin of a committed store type is never materialized. That is why
//! including every non-built-in dep here is safe.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use cerulion_core::codegen::parse_rosmsg;
use cerulion_dds::{
    AcquiredMsg, AcquiredSchema, AcquisitionOutcome, AcquisitionRung, DiscoveryResult, RungSkip,
    SchemaAcquirer, TypeAcquisition,
};

use crate::ros_cmd::nested_dependencies;
use crate::schema_store::builtin_has_qualified;

/// A filesystem-only [`SchemaAcquirer`] over the host's `$AMENT_PREFIX_PATH`
/// ROS 2 install. See the module docs for the harvest algorithm.
#[derive(Debug, Clone)]
pub struct LocalAmentAcquirer {
    /// Ament install prefixes, in precedence order (first wins). Each is an
    /// install root whose `share/<pkg>/msg/*.msg` and
    /// `share/ament_index/resource_index/rosidl_interfaces/<pkg>` are read.
    prefixes: Vec<PathBuf>,
}

impl LocalAmentAcquirer {
    /// Construct over an explicit, ordered prefix list (test-injectable — no
    /// env read). The first prefix that carries a type's `.msg` wins.
    pub fn new(prefixes: Vec<PathBuf>) -> Self {
        Self { prefixes }
    }

    /// Construct from the process `$AMENT_PREFIX_PATH` (colon-split, empty
    /// segments dropped, order preserved = overlay precedence). An unset/empty
    /// variable yields an acquirer with no prefixes, which SKIPS every type
    /// with the "no local ROS install" reason.
    pub fn from_env() -> Self {
        let raw = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
        Self::new(split_ament_prefix_path(&raw))
    }

    /// Acquire ONE requested `pkg/Type`: harvest its full nested closure or
    /// SKIP with a precise reason.
    fn acquire_one(&self, requested: &str) -> AcquisitionOutcome {
        if self.prefixes.is_empty() {
            return skipped("no AMENT_PREFIX_PATH — no local ROS install to harvest from");
        }
        let mut closure: Vec<AcquiredMsg> = Vec::new();
        let mut visited: BTreeSet<String> = BTreeSet::new();
        match self.harvest_closure(requested, &mut closure, &mut visited) {
            Ok(()) => AcquisitionOutcome::Acquired(AcquiredSchema {
                rung: AcquisitionRung::LocalAment,
                closure,
            }),
            Err(reason) => skipped(reason),
        }
    }

    /// Harvest `qualified` and (recursively) every non-built-in nested
    /// dependency into `closure`, cycle-safe via `visited`. Pushes each member
    /// exactly once, requested-type-first (DFS pre-order). `Err(reason)` on any
    /// gap — the reason names the dependency chain for a nested failure.
    fn harvest_closure(
        &self,
        qualified: &str,
        closure: &mut Vec<AcquiredMsg>,
        visited: &mut BTreeSet<String>,
    ) -> Result<(), String> {
        // Cycle-safe: a type already harvested (or in-flight) is a no-op.
        if !visited.insert(qualified.to_string()) {
            return Ok(());
        }
        let (pkg, ty) = split_qualified(qualified)
            .ok_or_else(|| format!("{qualified:?} is not a valid pkg/Type name"))?;

        let text = self.read_msg_text(pkg, ty)?;
        let schema = parse_rosmsg(&text, ty, Some(pkg)).map_err(|e| {
            format!(
                "harvested {pkg}/{ty}.msg from the local ROS install but it failed to parse: {e}"
            )
        })?;
        closure.push(AcquiredMsg {
            package: pkg.to_string(),
            type_name: ty.to_string(),
            msg_text: text,
        });

        for r in nested_dependencies(&schema) {
            // Resolve the reference to a filesystem harvest target with the
            // SAME ladder semantics the CDR decoder uses:
            // a bare `Header` resolves to `std_msgs/Header` (a built-in,
            // served from the corpus), NOT to a non-existent `<pkg>/Header` a
            // harvest would chase and fail on.
            let dep = r.harvest_target();
            // A built-in dep is served from the corpus, never harvested into
            // the bundle (the completeness walk accepts built-ins).
            if builtin_has_qualified(&dep) {
                continue;
            }
            self.harvest_closure(&dep, closure, visited)
                .map_err(|why| {
                    format!("nested dependency {dep} of {qualified} could not be harvested: {why}")
                })?;
        }
        Ok(())
    }

    /// Read `<pkg>/msg/<ty>.msg`'s verbatim text from the first prefix that
    /// carries it (marker-listed OR direct probe). `Err(reason)` names the most
    /// precise aggregate failure when no prefix yields the file.
    fn read_msg_text(&self, pkg: &str, ty: &str) -> Result<String, String> {
        // Some prefix's marker LISTED the type (so the file was promised).
        let mut saw_marker_listing = false;
        // Some prefix has a marker for the pkg that does NOT list the type.
        let mut saw_marker_without_type = false;

        for prefix in &self.prefixes {
            let marker = prefix
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("rosidl_interfaces")
                .join(pkg);
            let msg_path = prefix
                .join("share")
                .join(pkg)
                .join("msg")
                .join(format!("{ty}.msg"));

            if marker.is_file() {
                match std::fs::read_to_string(&marker) {
                    Ok(text) if ament_marker_lists_msg(&text, ty) => {
                        saw_marker_listing = true;
                        if msg_path.is_file() {
                            return read_verbatim(&msg_path);
                        }
                        // Marker promised the file but it is absent here —
                        // keep looking in later prefixes.
                    }
                    Ok(_) => {
                        saw_marker_without_type = true;
                        // Some overlays under-list; still probe the file.
                        if msg_path.is_file() {
                            tracing::debug!(
                                prefix = %prefix.display(),
                                pkg,
                                ty,
                                "local ament harvest: ament index present but does not list \
                                 this msg — probing share/<pkg>/msg/<Type>.msg directly"
                            );
                            return read_verbatim(&msg_path);
                        }
                    }
                    Err(e) => {
                        tracing::debug!(
                            prefix = %prefix.display(),
                            pkg,
                            marker = %marker.display(),
                            error = %e,
                            "local ament harvest: ament index marker unreadable — probing the \
                             file directly"
                        );
                        if msg_path.is_file() {
                            return read_verbatim(&msg_path);
                        }
                    }
                }
            } else if msg_path.is_file() {
                // Marker-less install: the file is the source of truth.
                tracing::debug!(
                    prefix = %prefix.display(),
                    pkg,
                    ty,
                    "local ament harvest: no ament index marker for pkg — probing \
                     share/<pkg>/msg/<Type>.msg directly"
                );
                return read_verbatim(&msg_path);
            }
        }

        // Exhausted every prefix — choose the most precise reason.
        if saw_marker_listing {
            Err(format!(
                "the local ROS install's ament index lists msg/{ty}.msg for {pkg}, but no such \
                 file exists in any ament prefix (broken install)"
            ))
        } else if saw_marker_without_type {
            Err(format!(
                "{pkg} is installed in the local ROS install (ament index present) but does not \
                 list msg/{ty}.msg, and no {pkg}/msg/{ty}.msg file was found in any ament prefix"
            ))
        } else {
            Err(format!(
                "{pkg} not found in any ament prefix (no rosidl_interfaces marker and no \
                 share/{pkg}/msg/{ty}.msg file)"
            ))
        }
    }
}

impl SchemaAcquirer for LocalAmentAcquirer {
    // The local ament rung reads the FILESYSTEM (`$AMENT_PREFIX_PATH`), so it
    // ignores the threaded discovery context — only the wire rung consumes it.
    fn acquire(&self, types: &[String], _discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        // Every requested type is ATTEMPTED (harvested or skipped with a
        // precise reason) — the local rung never silently omits a type. Skip
        // reasons are surfaced by the driver's report, so no `warn!` here
        // (avoids double-warning); the probe breadcrumbs are `debug!`.
        types
            .iter()
            .map(|t| TypeAcquisition {
                requested: t.clone(),
                outcome: self.acquire_one(t),
            })
            .collect()
    }
}

/// Read a `.msg` file's exact bytes as UTF-8 text (the byte-verbatim contract:
/// the store materialization must be byte-identical to the install file).
fn read_verbatim(msg_path: &Path) -> Result<String, String> {
    std::fs::read_to_string(msg_path)
        .map_err(|e| format!("reading {} failed: {e}", msg_path.display()))
}

/// Wrap a reason in a single-rung [`AcquisitionOutcome::Skipped`] tagged
/// [`AcquisitionRung::LocalAment`].
fn skipped(reason: impl Into<String>) -> AcquisitionOutcome {
    AcquisitionOutcome::Skipped(vec![RungSkip {
        rung: AcquisitionRung::LocalAment,
        reason: reason.into(),
    }])
}

/// Split `$AMENT_PREFIX_PATH` into prefixes: colon-separated, empty segments
/// dropped, order preserved (= overlay precedence). Pure — unit-testable on an
/// injected string with no env mutation.
pub(crate) fn split_ament_prefix_path(value: &str) -> Vec<PathBuf> {
    value
        .split(':')
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .collect()
}

/// Does an ament `rosidl_interfaces/<pkg>` marker list `msg/<ty>.msg`? The
/// marker is a newline list of the package's interface files; a line matches
/// when it trims exactly to `msg/<ty>.msg`. Pure — unit-testable.
pub(crate) fn ament_marker_lists_msg(marker_text: &str, ty: &str) -> bool {
    let needle = format!("msg/{ty}.msg");
    marker_text.lines().any(|line| line.trim() == needle)
}

/// Split a qualified `pkg/Type` into `(pkg, Type)`, requiring exactly one `/`
/// with non-empty halves. `None` for a bare/invalid name.
fn split_qualified(qualified: &str) -> Option<(&str, &str)> {
    match qualified.split_once('/') {
        Some((pkg, ty)) if !pkg.is_empty() && !ty.is_empty() && !ty.contains('/') => {
            Some((pkg, ty))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─────────────────────────── Pure helper oracles ───────────────────────

    #[test]
    fn test_split_ament_prefix_path_drops_empties_and_preserves_order() {
        assert_eq!(
            split_ament_prefix_path("/opt/ros/jazzy:/home/u/ws/install"),
            vec![
                PathBuf::from("/opt/ros/jazzy"),
                PathBuf::from("/home/u/ws/install"),
            ]
        );
        // Empty segments (leading/trailing/interior double-colon) dropped.
        assert_eq!(
            split_ament_prefix_path(":/a::/b:"),
            vec![PathBuf::from("/a"), PathBuf::from("/b")]
        );
        // Empty / whitespace-only inputs → no prefixes.
        assert!(split_ament_prefix_path("").is_empty());
        assert_eq!(split_ament_prefix_path("solo"), vec![PathBuf::from("solo")]);
    }

    #[test]
    fn test_ament_marker_lists_msg_matches_exact_line() {
        let marker = "msg/PointCloud2.msg\nmsg/Image.msg\nsrv/GetPlan.srv\n";
        assert!(ament_marker_lists_msg(marker, "PointCloud2"));
        assert!(ament_marker_lists_msg(marker, "Image"));
        // Not listed / partial matches must NOT trip.
        assert!(!ament_marker_lists_msg(marker, "GetPlan")); // it's a srv
        assert!(!ament_marker_lists_msg(marker, "Point")); // substring, not a line
        assert!(!ament_marker_lists_msg(marker, "Image2"));
        assert!(!ament_marker_lists_msg("", "Image"));
    }

    #[test]
    fn test_split_qualified_requires_one_slash() {
        assert_eq!(
            split_qualified("acme_msgs/Widget"),
            Some(("acme_msgs", "Widget"))
        );
        assert_eq!(split_qualified("Widget"), None);
        assert_eq!(split_qualified("acme/msg/Widget"), None);
        assert_eq!(split_qualified("/Widget"), None);
        assert_eq!(split_qualified("acme/"), None);
    }

    // ─────────────────────── Fake ament-tree harness ───────────────────────

    /// Write `<prefix>/share/<pkg>/msg/<ty>.msg` = `text`. When `in_index`,
    /// ALSO append `msg/<ty>.msg` to the pkg's ament index marker.
    fn write_ament_msg(prefix: &Path, pkg: &str, ty: &str, text: &str, in_index: bool) {
        let msg_dir = prefix.join("share").join(pkg).join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join(format!("{ty}.msg")), text).unwrap();
        if in_index {
            let idx_dir = prefix
                .join("share")
                .join("ament_index")
                .join("resource_index")
                .join("rosidl_interfaces");
            std::fs::create_dir_all(&idx_dir).unwrap();
            let marker = idx_dir.join(pkg);
            let mut content = std::fs::read_to_string(&marker).unwrap_or_default();
            content.push_str(&format!("msg/{ty}.msg\n"));
            std::fs::write(&marker, content).unwrap();
        }
    }

    /// Extract the single-rung skip reason (panics if not a single-rung skip).
    fn skip_reason(outcome: &AcquisitionOutcome) -> &str {
        match outcome {
            AcquisitionOutcome::Skipped(reasons) => {
                assert_eq!(reasons.len(), 1, "local rung emits ONE reason");
                assert_eq!(reasons[0].rung, AcquisitionRung::LocalAment);
                &reasons[0].reason
            }
            AcquisitionOutcome::Acquired(_) => panic!("expected Skipped, got Acquired"),
        }
    }

    fn acquired(outcome: &AcquisitionOutcome) -> &AcquiredSchema {
        match outcome {
            AcquisitionOutcome::Acquired(s) => s,
            AcquisitionOutcome::Skipped(r) => panic!("expected Acquired, got Skipped: {r:?}"),
        }
    }

    /// Closure member `pkg/Type` names, in bundle (harvest) order.
    fn closure_names(schema: &AcquiredSchema) -> Vec<String> {
        schema.closure.iter().map(|m| m.qualified_name()).collect()
    }

    // ─────────────────────────────── TEST 1 ────────────────────────────────

    /// Happy path: a marker + msg-file install harvests the requested type AND
    /// a cross-package nested dep; rung label + byte-verbatim text (hand oracle).
    #[test]
    fn test_happy_path_harvests_type_and_cross_package_dep() {
        let tmp = tempfile::tempdir().unwrap();
        let p = tmp.path();
        const WIDGET: &str = "# a widget\nint32 id\nacme_geometry/Vec2 where\nfloat64 value\n";
        const VEC2: &str = "float64 x\nfloat64 y\n";
        write_ament_msg(p, "acme_msgs", "Widget", WIDGET, true);
        write_ament_msg(p, "acme_geometry", "Vec2", VEC2, true);

        let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].requested, "acme_msgs/Widget");
        let schema = acquired(&out[0].outcome);
        assert_eq!(schema.rung, AcquisitionRung::LocalAment);
        assert_eq!(schema.rung.label(), "local ROS install via ament index");
        // Full closure: requested type first, then the cross-package dep.
        assert_eq!(
            closure_names(schema),
            vec![
                "acme_msgs/Widget".to_string(),
                "acme_geometry/Vec2".to_string()
            ]
        );
        // BYTE-VERBATIM (hand oracle) for both members.
        assert_eq!(schema.closure[0].msg_text, WIDGET);
        assert_eq!(schema.closure[1].msg_text, VEC2);
    }

    // ─────────────────────────────── TEST 2 ────────────────────────────────

    /// Precedence: two prefixes both carry the type with DIFFERENT bytes → the
    /// FIRST prefix wins.
    #[test]
    fn test_precedence_first_prefix_wins() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        write_ament_msg(a.path(), "acme_msgs", "Widget", "int32 first\n", true);
        write_ament_msg(b.path(), "acme_msgs", "Widget", "int32 second\n", true);

        let acq = LocalAmentAcquirer::new(vec![a.path().to_path_buf(), b.path().to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let schema = acquired(&out[0].outcome);
        assert_eq!(
            schema.closure[0].msg_text, "int32 first\n",
            "first prefix wins"
        );
    }

    // ─────────────────────────────── TEST 3 ────────────────────────────────

    /// Recursion depth >= 2 (A -> B -> C) AND cycle safety (A -> B -> A
    /// terminates, each member appears once).
    #[test]
    fn test_recursion_depth_and_cycle_safety() {
        // Depth chain A -> B -> C (C is a leaf).
        {
            let tmp = tempfile::tempdir().unwrap();
            let p = tmp.path();
            write_ament_msg(p, "acme", "A", "acme/B b\nint32 n\n", true);
            write_ament_msg(p, "acme", "B", "acme/C c\n", true);
            write_ament_msg(p, "acme", "C", "float64 v\n", true);
            let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
            let out = acq.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
            let schema = acquired(&out[0].outcome);
            assert_eq!(
                closure_names(schema),
                vec![
                    "acme/A".to_string(),
                    "acme/B".to_string(),
                    "acme/C".to_string()
                ]
            );
        }
        // Cycle A -> B -> A: must terminate, each member exactly once.
        {
            let tmp = tempfile::tempdir().unwrap();
            let p = tmp.path();
            write_ament_msg(p, "acme", "A", "acme/B b\n", true);
            write_ament_msg(p, "acme", "B", "acme/A a\n", true);
            let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
            let out = acq.acquire(&["acme/A".to_string()], &DiscoveryResult::empty());
            let schema = acquired(&out[0].outcome);
            let mut names = closure_names(schema);
            names.sort();
            assert_eq!(names, vec!["acme/A".to_string(), "acme/B".to_string()]);
            assert_eq!(schema.closure.len(), 2, "each cycle member once");
        }
    }

    // ─────────────────────────────── TEST 4 ────────────────────────────────

    /// A built-in dep (`std_msgs/Header`, `geometry_msgs/Point`) is NOT
    /// harvested into the bundle — served from the corpus, never duplicated.
    #[test]
    fn test_builtin_dep_not_harvested_into_bundle() {
        let tmp = tempfile::tempdir().unwrap();
        let p = tmp.path();
        // Widget references two built-ins and one custom type.
        write_ament_msg(
            p,
            "acme_msgs",
            "Widget",
            "std_msgs/Header header\ngeometry_msgs/Point position\nacme_msgs/Tag tag\n",
            true,
        );
        write_ament_msg(p, "acme_msgs", "Tag", "int32 id\n", true);

        let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let schema = acquired(&out[0].outcome);
        let names = closure_names(schema);
        // Only the non-built-in closure: Widget + Tag. NO Header/Point.
        assert_eq!(
            names,
            vec!["acme_msgs/Widget".to_string(), "acme_msgs/Tag".to_string()]
        );
        assert!(!names.iter().any(|n| n == "std_msgs/Header"));
        assert!(!names.iter().any(|n| n == "geometry_msgs/Point"));
    }

    // ─────────────────────────────── TEST 5 ────────────────────────────────

    /// Distinct skip reasons: empty prefixes, pkg nowhere, marker-without-type
    /// + no file, and a missing nested dep (reason names the dep).
    #[test]
    fn test_distinct_skip_reasons() {
        // (a) No prefixes configured.
        {
            let acq = LocalAmentAcquirer::new(vec![]);
            let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
            let reason = skip_reason(&out[0].outcome);
            assert!(reason.contains("no AMENT_PREFIX_PATH"), "{reason}");
            assert!(reason.contains("no local ROS install"), "{reason}");
        }
        // (b) Pkg absent from every prefix (no marker, no file).
        {
            let tmp = tempfile::tempdir().unwrap();
            // A different pkg exists so the prefix is a real install.
            write_ament_msg(tmp.path(), "other_msgs", "Thing", "int32 x\n", true);
            let acq = LocalAmentAcquirer::new(vec![tmp.path().to_path_buf()]);
            let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
            let reason = skip_reason(&out[0].outcome);
            assert!(
                reason.contains("acme_msgs not found in any ament prefix"),
                "{reason}"
            );
        }
        // (c) Marker present for the pkg, but it does NOT list the type AND no
        //     file for the type exists.
        {
            let tmp = tempfile::tempdir().unwrap();
            // acme_msgs is installed (marker + file for a DIFFERENT type).
            write_ament_msg(tmp.path(), "acme_msgs", "Other", "int32 x\n", true);
            let acq = LocalAmentAcquirer::new(vec![tmp.path().to_path_buf()]);
            let out = acq.acquire(
                &["acme_msgs/Missing".to_string()],
                &DiscoveryResult::empty(),
            );
            let reason = skip_reason(&out[0].outcome);
            assert!(reason.contains("ament index present"), "{reason}");
            assert!(reason.contains("does not list msg/Missing.msg"), "{reason}");
        }
        // (d) A missing NESTED dep — the reason names the dep chain, and the
        //     requested type stays UNACQUIRED.
        {
            let tmp = tempfile::tempdir().unwrap();
            let p = tmp.path();
            // Widget references acme_geometry/Vec2, which is NOT installed.
            write_ament_msg(p, "acme_msgs", "Widget", "acme_geometry/Vec2 where\n", true);
            let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
            let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
            let reason = skip_reason(&out[0].outcome);
            assert!(
                reason.contains("nested dependency acme_geometry/Vec2"),
                "{reason}"
            );
            assert!(reason.contains("acme_msgs/Widget"), "{reason}");
            assert!(
                reason.contains("acme_geometry not found in any ament prefix"),
                "{reason}"
            );
        }
    }

    // ─────────────────────────────── TEST 6 ────────────────────────────────

    /// A malformed harvested `.msg` (a single-token line — the known hard parse
    /// error) SKIPS the whole type, surfacing the parse error.
    #[test]
    fn test_malformed_harvested_msg_skips_with_parse_error() {
        let tmp = tempfile::tempdir().unwrap();
        // "int32" alone is a type token with no field name — a hard parse err.
        write_ament_msg(tmp.path(), "acme_msgs", "Widget", "int32\n", true);
        let acq = LocalAmentAcquirer::new(vec![tmp.path().to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let reason = skip_reason(&out[0].outcome);
        assert!(reason.contains("failed to parse"), "{reason}");
        assert!(reason.contains("acme_msgs/Widget.msg"), "{reason}");
    }

    // ─────────────────────────────── TEST 7 ────────────────────────────────

    /// A marker-LESS install (no ament index at all) still harvests via the
    /// direct `share/<pkg>/msg/<Type>.msg` probe.
    #[test]
    fn test_markerless_install_harvests_via_direct_probe() {
        let tmp = tempfile::tempdir().unwrap();
        write_ament_msg(tmp.path(), "acme_msgs", "Widget", "int32 id\n", false); // no index
                                                                                 // Sanity: no ament_index dir exists.
        assert!(!tmp.path().join("share").join("ament_index").exists());
        let acq = LocalAmentAcquirer::new(vec![tmp.path().to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let schema = acquired(&out[0].outcome);
        assert_eq!(closure_names(schema), vec!["acme_msgs/Widget".to_string()]);
        assert_eq!(schema.closure[0].msg_text, "int32 id\n");
    }

    // ─────────────────────────────── TEST 8 ────────────────────────────────

    /// Determinism: two runs over the same install yield byte-identical
    /// outcomes.
    #[test]
    fn test_determinism_two_runs_byte_identical() {
        let tmp = tempfile::tempdir().unwrap();
        let p = tmp.path();
        write_ament_msg(
            p,
            "acme_msgs",
            "Widget",
            "acme_msgs/Tag tag\nint32 id\n",
            true,
        );
        write_ament_msg(p, "acme_msgs", "Tag", "string name\n", true);
        write_ament_msg(p, "other_msgs", "Gadget", "int32 x\n", true);
        let acq = LocalAmentAcquirer::new(vec![p.to_path_buf()]);
        let wanted = vec![
            "acme_msgs/Widget".to_string(),
            "other_msgs/Gadget".to_string(),
            "acme_msgs/Nope".to_string(), // a skip, for good measure
        ];
        let disc = DiscoveryResult::empty();
        let run1 = acq.acquire(&wanted, &disc);
        let run2 = acq.acquire(&wanted, &disc);
        assert_eq!(run1, run2, "two runs must be byte-identical");
    }

    /// Marker-listed-but-file-absent in prefix A, present in prefix B → B wins
    /// (the "first prefix that yields the FILE wins" contract, marker aside).
    #[test]
    fn test_marker_promises_but_file_only_in_later_prefix() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        // Prefix A: marker lists Widget but the .msg file is removed.
        write_ament_msg(a.path(), "acme_msgs", "Widget", "int32 stale\n", true);
        std::fs::remove_file(
            a.path()
                .join("share")
                .join("acme_msgs")
                .join("msg")
                .join("Widget.msg"),
        )
        .unwrap();
        // Prefix B: the real file.
        write_ament_msg(b.path(), "acme_msgs", "Widget", "int32 real\n", true);

        let acq = LocalAmentAcquirer::new(vec![a.path().to_path_buf(), b.path().to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let schema = acquired(&out[0].outcome);
        assert_eq!(schema.closure[0].msg_text, "int32 real\n");
    }

    // ─────────────────────────────── TEST 10 ───────────────────────────────

    /// A BARE `Header` reference (ROS 2's legal
    /// shorthand for `std_msgs/Header`) resolves via the codec's Header rung to
    /// the BUILT-IN `std_msgs/Header` — served from the corpus, NOT chased as a
    /// non-existent `acme_msgs/Header` (which an eager `<pkg>/Name`
    /// resolution would do, SKIPPING the whole type with a spurious missing-dep).
    #[test]
    fn test_bare_header_ref_resolves_to_builtin_not_same_package() {
        let tmp = tempfile::tempdir().unwrap();
        write_ament_msg(
            tmp.path(),
            "acme_msgs",
            "Widget",
            "Header header\nfloat64 x\n",
            true,
        );
        let acq = LocalAmentAcquirer::new(vec![tmp.path().to_path_buf()]);
        let out = acq.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
        let schema = acquired(&out[0].outcome);
        // Only Widget is harvested; the bare Header is a built-in, never chased
        // as acme_msgs/Header.
        assert_eq!(closure_names(schema), vec!["acme_msgs/Widget".to_string()]);
    }
}
