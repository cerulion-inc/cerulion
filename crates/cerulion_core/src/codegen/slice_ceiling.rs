// SPDX-License-Identifier: AGPL-3.0-only
//! The RUNTIME per-type slice-ceiling lookup for bridged variable
//! topics, plus the `CERULION_RMW_SLICE_CEILING` override grammar.
//!
//! # Per-type ceiling policy
//!
//! The rmw bridge (`rmw_cerulion`) consults the EXISTING native five-tier
//! per-type table instead of the blanket 128 MiB for variable bridged topics,
//! plus one env escape hatch (`CERULION_RMW_SLICE_CEILING=<pkg/Type>:<bytes>`),
//! because a bridged topic has no graph YAML to write `max_slice_len:` in.
//!
//! This module is the CORE half: the name-keyed lookup over the one shared
//! table match (`variable_schema_listed_tier` — the same match codegen's
//! [`crate::codegen::variable_schema_max_slice_len`] resolves through, never a
//! copy) and the pure override parser/resolver. The bridge half — the
//! `rmw_cerulion` call sites that read the env and consult these functions —
//! lives in that crate's `api/pubsub.rs`.
//!
//! # Spelling: `pkg/Type`, never `pkg::Type`
//!
//! Every name this module accepts is the Cerulion QUALIFIED schema name —
//! `sensor_msgs/Image` — which is exactly the spelling the rmw type bridge
//! derives from rosidl introspection and the spelling the tier table is keyed
//! by. `pkg::Type` is REJECTED loudly, for two reasons stated once here:
//!
//! 1. In the env grammar, `:` is the name/bytes separator, so a `::`-spelled
//!    name is structurally unparseable — accepting it anywhere else would
//!    split the module's surface into two spellings.
//! 2. A `::`-spelled name can never be a table key, so accepting it in the
//!    lookup could only ever mean a silent fall-through to the 128 MiB
//!    catch-all — the exact silent-default class this table exists to kill.
//!
//! The warn-coverage story is deliberately ASYMMETRIC between the two
//! halves: the PARSER rejects ANY name that is not exactly
//! two non-empty `/`-separated components — a leading/trailing slash, a third
//! segment (the rosidl `pkg/msg/Type` spelling gets its own did-you-mean
//! hint), no slash at all, whitespace left in the name once the entry's
//! edges are trimmed — because an override stored under a name that can
//! never match a bridge topic would be silently INERT, the exact class this
//! module's strict-parse discipline exists to kill. The LOOKUP stays TOTAL
//! and degrades to the catch-all, warning only on the provably-wrong
//! spellings (`pkg::Type`, rosidl `pkg/msg/Type`) — each SHORT-CIRCUITS to
//! the catch-all without consulting the table, so one mistake gets ONE
//! diagnostic (the fall-through used to reach the in-repo guard,
//! which stacked a misleading missing-arm error on the same call); a
//! 2-segment unknown or a package-less workspace-schema name stays SILENT —
//! silence there is the documented tier, not a missed warning.
//!
//! # Fixed-layout types are not this module's business
//!
//! A recursively-fixed type's wire frame is EXACTLY `WireHeader::SIZE +
//! fixed section`, and the bridge already sizes those exactly (see
//! [`crate::codegen::route_budget`] for the same rule on the `ros2 attach`
//! plane). The lookup here is for VARIABLE-layout types only; handing it a
//! fixed in-repo type's name is answered with a loud `error!` plus the safe
//! catch-all — never a panic (the panic on that shape is the CODEGEN wrapper's
//! build-time gate, deliberately not replicated at run time).

use crate::wire::MaxSliceLen;

use super::generator::{
    in_repo_package, variable_schema_listed_tier, UNLISTED_SCHEMA_FALLBACK_BYTES,
};

/// The environment variable [`resolve_rmw_slice_ceiling`] layers over the
/// table: a comma-separated `<pkg/Type>:<bytes>` list, e.g.
/// `sensor_msgs/Image:33554432,my_pkg/Big:268435456`.
///
/// Read by the CALLER (the rmw bridge) —
/// the functions here take the string so they stay pure and oracle-testable.
pub const RMW_SLICE_CEILING_ENV: &str = "CERULION_RMW_SLICE_CEILING";

/// The per-type slice ceiling for a VARIABLE-layout schema, resolved through
/// the SAME five-tier table codegen embeds into every generated
/// `MAX_SLICE_LEN` (one source of truth, see
/// [`crate::codegen::variable_schema_max_slice_len`]).
///
/// * `pkg_slash_type` is the Cerulion qualified name (`"sensor_msgs/Image"`).
///   A `pkg::Type` spelling is rejected LOUDLY (one `warn!` naming the
///   accepted spelling) and treated as unknown, and so is the rosidl
///   `pkg/msg/Type` spelling (with a did-you-mean hint). Both SHORT-CIRCUIT
///   straight to the catch-all without consulting the table — one mistake,
///   ONE diagnostic (the in-repo guard cannot tell a misspelling from a
///   missing arm). See the module docs for why normalizing is not on the
///   table and for the parser/lookup warn-coverage split.
/// * An unknown type takes the table's own catch-all (128 MiB — safe because
///   iceoryx2 `Static` pools are lazy/demand-paged; see the tier table's
///   launch-bump note).
/// * A FIXED-layout type is NOT this function's business — the bridge sizes
///   those exactly at `WireHeader::SIZE + fixed section`. Handing one in
///   anyway is a loud `error!` plus the catch-all, never a panic (that panic
///   is the codegen wrapper's build-time gate).
///
/// Cold path: meant for route/publisher CREATION, never per-message.
pub fn slice_ceiling_for_type(pkg_slash_type: &str) -> MaxSliceLen {
    // A KNOWN-WRONG spelling SHORT-CIRCUITS to the
    // catch-all WITHOUT consulting the table path — one user mistake, ONE
    // diagnostic. Falling through would hand e.g. `tf2_msgs/msg/TFMessage`
    // to the in-repo guard below, which cannot tell a misspelling from a
    // missing table arm and would stack a second, MISLEADING error on the
    // same call.
    if pkg_slash_type.contains("::") {
        tracing::warn!(
            type_name = %pkg_slash_type,
            "slice-ceiling lookup received a 'pkg::Type' spelling; the accepted \
             spelling is 'pkg/Type' (the Cerulion qualified name) — treating the \
             type as UNKNOWN, which takes the 128 MiB catch-all tier"
        );
        return typed_ceiling(UNLISTED_SCHEMA_FALLBACK_BYTES);
    }
    if rosidl_three_segment(pkg_slash_type) {
        tracing::warn!(
            type_name = %pkg_slash_type,
            "slice-ceiling lookup received the rosidl 'pkg/msg/Type' spelling — \
             did you mean 'pkg/Type' (drop the 'msg' segment)? treating the type \
             as UNKNOWN, which takes the 128 MiB catch-all tier"
        );
        return typed_ceiling(UNLISTED_SCHEMA_FALLBACK_BYTES);
    }
    let bytes = match variable_schema_listed_tier(pkg_slash_type) {
        Some(bytes) => bytes,
        None => {
            if in_repo_package(pkg_slash_type).is_some() {
                tracing::error!(
                    type_name = %pkg_slash_type,
                    "slice-ceiling lookup for an in-repo type the tier table does \
                     not list — either a FIXED-layout type (not this lookup's \
                     business: the bridge sizes those exactly at wire header + \
                     fixed section) or a variable schema missing its arm (the \
                     codegen build-time gate catches that); serving the 128 MiB \
                     catch-all"
                );
            }
            UNLISTED_SCHEMA_FALLBACK_BYTES
        }
    };
    typed_ceiling(bytes)
}

/// The checked `usize -> MaxSliceLen` conversion shared by the short-circuit
/// arms and the table path. Failure is unreachable for the shipping table
/// (every tier and the catch-all sit inside `MaxSliceLen`'s bounds); the
/// panic mirrors codegen's own const-eval guarantee rather than inventing a
/// silent fallback.
fn typed_ceiling(bytes: usize) -> MaxSliceLen {
    u32::try_from(bytes)
        .ok()
        .and_then(MaxSliceLen::try_new)
        .expect(
            "the tier table returned a budget outside MaxSliceLen's bounds (>= 32 B, \
             <= u32::MAX) — the table and the wire bounds have drifted; fix the table",
        )
}

/// Is `name` EXACTLY the `pkg/Type` shape — two non-empty components split on
/// one `/`? Anything else (a leading/trailing slash, a third segment, no
/// slash at all) can never match a bridge topic's qualified name, so an
/// override stored under it would be silently INERT — the parser refuses to
/// store one.
fn is_pkg_slash_type(name: &str) -> bool {
    match name.split_once('/') {
        Some((pkg, ty)) => !pkg.is_empty() && !ty.is_empty() && !ty.contains('/'),
        None => false,
    }
}

/// The rosidl `pkg/msg/Type` spelling — exactly three non-empty segments with
/// `msg` in the middle. Never a wire name, but the misspelling a real ROS
/// user actually types, so both the parser and the lookup answer it with a
/// specific "did you mean 'pkg/Type'" hint instead of the generic message.
fn rosidl_three_segment(name: &str) -> bool {
    let mut segments = name.split('/');
    matches!(
        (
            segments.next(),
            segments.next(),
            segments.next(),
            segments.next()
        ),
        (Some(pkg), Some("msg"), Some(ty), None) if !pkg.is_empty() && !ty.is_empty()
    )
}

/// The parsed `CERULION_RMW_SLICE_CEILING` override set.
///
/// Built by [`SliceCeilingOverrides::parse`]; consulted by
/// [`SliceCeilingOverrides::get`]. A caller creating many routes should parse
/// ONCE and layer `get` over [`slice_ceiling_for_type`] rather than calling
/// [`resolve_rmw_slice_ceiling`] per route (which re-parses, re-warning any
/// malformed entry each time).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SliceCeilingOverrides {
    /// `(qualified name, ceiling)` in first-appearance order; at most one
    /// entry per name (a duplicate replaces in place — LAST wins, loudly).
    entries: Vec<(String, MaxSliceLen)>,
}

impl SliceCeilingOverrides {
    /// Parse an env value of comma-separated `<pkg/Type>:<bytes>` entries.
    ///
    /// STRICT and total: a malformed entry is a loud `warn!` naming the
    /// offending entry and is SKIPPED — never a silent zero, never a panic —
    /// and the well-formed entries around it still apply. The malformed
    /// shapes, each with its own message:
    ///
    /// * no `:` separator, or a type name that is not EXACTLY `pkg/Type` —
    ///   two non-empty components: empty/missing halves, a leading or
    ///   trailing slash, a third segment (the rosidl `pkg/msg/Type` spelling
    ///   gets its own did-you-mean hint), `::`-spelled (see the module docs),
    ///   or carrying whitespace anywhere in the name (each entry is trimmed
    ///   at its EDGES first, so a space after the list comma stays
    ///   fine). A stored name that can never match a bridge topic would
    ///   be a silently-inert override, so none is stored;
    /// * a bytes value that is not an unsigned integer, exceeds `u32::MAX`
    ///   (`WireHeader::total_size` is `u32` — the same bound
    ///   [`MaxSliceLen`] enforces), or sits below the 32-byte wire-header
    ///   floor (zero included).
    ///
    /// Empty segments (a trailing comma, a blank value) are skipped silently —
    /// they name nothing to warn about. Duplicate entries for one type warn
    /// and the LAST wins, so appending to an inherited value overrides it.
    pub fn parse(env_value: &str) -> Self {
        let mut entries: Vec<(String, MaxSliceLen)> = Vec::new();
        for raw in env_value.split(',') {
            let entry = raw.trim();
            if entry.is_empty() {
                continue;
            }
            let Some((name, value)) = entry.rsplit_once(':') else {
                tracing::warn!(
                    entry = %entry,
                    "malformed {RMW_SLICE_CEILING_ENV} entry: no ':' separator \
                     (expected '<pkg/Type>:<bytes>'); entry SKIPPED"
                );
                continue;
            };
            if name.contains(':') {
                tracing::warn!(
                    entry = %entry,
                    "malformed {RMW_SLICE_CEILING_ENV} entry: the type name contains \
                     ':' — the accepted spelling is 'pkg/Type', never 'pkg::Type'; \
                     entry SKIPPED"
                );
                continue;
            }
            // Whitespace REMAINING in
            // the name after the per-entry trim would be stored verbatim and
            // could then never match a bridge topic's exact name — the same
            // silently-inert class the shape check closed, one notch
            // over. Edge whitespace around the ENTRY (a space after the list
            // comma) was already trimmed above and stays friendly; whatever
            // is left is inside the name, and that is a real malformation.
            if name.chars().any(char::is_whitespace) {
                tracing::warn!(
                    entry = %entry,
                    "malformed {RMW_SLICE_CEILING_ENV} entry: the type name contains \
                     whitespace — a stored name can only ever match a bridge \
                     topic's exact 'pkg/Type'; entry SKIPPED"
                );
                continue;
            }
            if !is_pkg_slash_type(name) {
                if rosidl_three_segment(name) {
                    tracing::warn!(
                        entry = %entry,
                        "malformed {RMW_SLICE_CEILING_ENV} entry: 'pkg/msg/Type' is \
                         the rosidl spelling, not the wire name — did you mean \
                         'pkg/Type' (drop the 'msg' segment)? entry SKIPPED"
                    );
                } else {
                    tracing::warn!(
                        entry = %entry,
                        "malformed {RMW_SLICE_CEILING_ENV} entry: the type name is not \
                         'pkg/Type'-qualified (exactly two non-empty components split \
                         on '/'); entry SKIPPED"
                    );
                }
                continue;
            }
            let bytes_u64: u64 = match value.parse() {
                Ok(b) => b,
                Err(_) => {
                    tracing::warn!(
                        entry = %entry,
                        "malformed {RMW_SLICE_CEILING_ENV} entry: the bytes value is \
                         not an unsigned integer; entry SKIPPED"
                    );
                    continue;
                }
            };
            let Ok(bytes) = u32::try_from(bytes_u64) else {
                tracing::warn!(
                    entry = %entry,
                    "malformed {RMW_SLICE_CEILING_ENV} entry: the bytes value exceeds \
                     u32::MAX — WireHeader::total_size is u32, so a larger slot \
                     cannot be represented on the wire; entry SKIPPED"
                );
                continue;
            };
            let Some(ceiling) = MaxSliceLen::try_new(bytes) else {
                tracing::warn!(
                    entry = %entry,
                    "malformed {RMW_SLICE_CEILING_ENV} entry: the bytes value is \
                     below the 32-byte wire-header floor (a slot must hold at least \
                     the WireHeader; zero included); entry SKIPPED"
                );
                continue;
            };
            if let Some(slot) = entries.iter_mut().find(|(n, _)| n.as_str() == name) {
                tracing::warn!(
                    type_name = %name,
                    earlier_bytes = slot.1.get(),
                    later_bytes = ceiling.get(),
                    "duplicate {RMW_SLICE_CEILING_ENV} entries for one type; the \
                     LAST entry wins"
                );
                slot.1 = ceiling;
            } else {
                entries.push((name.to_string(), ceiling));
            }
        }
        Self { entries }
    }

    /// The override for `pkg_slash_type`, if one parsed. Exact-name match —
    /// the parser already rejected every non-`pkg/Type` spelling.
    pub fn get(&self, pkg_slash_type: &str) -> Option<MaxSliceLen> {
        self.entries
            .iter()
            .find(|(n, _)| n.as_str() == pkg_slash_type)
            .map(|(_, ceiling)| *ceiling)
    }

    /// Number of distinct types with a parsed override.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// `true` when nothing parsed (empty/absent env, or every entry malformed).
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// The composed rule: the env override WINS OUTRIGHT (it may widen
/// or narrow — unlike [`crate::codegen::route_budget`]'s `min`-compose, the
/// operator's explicit word is adopted verbatim); an absent env, an empty
/// env, or an env that does not name this type falls to the table
/// ([`slice_ceiling_for_type`]); an unknown type takes the table's catch-all.
///
/// Total by construction — the layered fallbacks admit no "no answer", which
/// is why this returns a bare [`MaxSliceLen`]: an `Option` would force every
/// caller to invent a fallback the table already provides (re-creating the
/// blanket this decision replaces).
///
/// `env_value` is the raw `CERULION_RMW_SLICE_CEILING` string, read by the
/// CALLER (purity — see [`RMW_SLICE_CEILING_ENV`]); it is re-parsed on each
/// call, so a caller resolving many types should parse once via
/// [`SliceCeilingOverrides`] instead.
pub fn resolve_rmw_slice_ceiling(pkg_slash_type: &str, env_value: Option<&str>) -> MaxSliceLen {
    if let Some(env_value) = env_value {
        if let Some(overridden) = SliceCeilingOverrides::parse(env_value).get(pkg_slash_type) {
            return overridden;
        }
    }
    slice_ceiling_for_type(pkg_slash_type)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracing_test::traced_test;

    const HUGE: u32 = 128 * 1024 * 1024;
    const LARGE: u32 = 16 * 1024 * 1024;
    const MEDIUM: u32 = 4 * 1024 * 1024;
    const SMALL: u32 = 256 * 1024;
    const TINY: u32 = 16 * 1024;

    /// Count captured lines carrying `level` as a whole whitespace token AND
    /// `needle` anywhere — a bare substring level match would also hit a
    /// needle that happens to contain the level's letters (the span-name
    /// lesson).
    fn count_level(lines: &[&str], level: &str, needle: &str) -> usize {
        lines
            .iter()
            .filter(|l| l.split_whitespace().any(|t| t == level) && l.contains(needle))
            .count()
    }

    // ── the lookup ────────────────────────────────────────────────────────

    /// One hand-oracle pin per tier, so a lookup bypassed to any constant
    /// fails at least four of the five.
    #[test]
    fn every_tier_resolves_through_the_shared_table_as_a_typed_ceiling() {
        assert_eq!(slice_ceiling_for_type("sensor_msgs/Image").get(), HUGE);
        assert_eq!(
            slice_ceiling_for_type("visualization_msgs/MarkerArray").get(),
            LARGE
        );
        assert_eq!(slice_ceiling_for_type("nav_msgs/Path").get(), MEDIUM);
        assert_eq!(slice_ceiling_for_type("tf2_msgs/TFMessage").get(), SMALL);
        assert_eq!(slice_ceiling_for_type("nav_msgs/Odometry").get(), TINY);
    }

    /// Unknown types — a packaged user type AND the package-less workspace
    /// shape — take the catch-all SILENTLY (that is their documented tier,
    /// not a failure).
    #[traced_test]
    #[test]
    fn an_unknown_type_takes_the_catch_all_silently() {
        assert_eq!(slice_ceiling_for_type("my_pkg/Big").get(), HUGE);
        assert_eq!(slice_ceiling_for_type("MyWorkspaceSchema").get(), HUGE);
        logs_assert(|lines: &[&str]| {
            match lines
                .iter()
                .filter(|l| l.split_whitespace().any(|t| t == "WARN" || t == "ERROR"))
                .count()
            {
                0 => Ok(()),
                n => Err(format!(
                    "unknown non-in-repo types must be silent, got {n} loud lines"
                )),
            }
        });
    }

    /// The spelling decision: `pkg::Type` is rejected LOUDLY and treated as
    /// unknown. The subject is a SMALL-tier type so the catch-all outcome
    /// (128 MiB) DISCRIMINATES from its real tier (256 KiB) — the accepted
    /// twin resolving to the real tier in the same body is the anti-tautology
    /// half.
    ///
    /// The `::` arm short-circuits like the rosidl arm, so even a
    /// slash-carrying `::` name whose first segment is an in-repo package
    /// (`tf2_msgs/msg::Bad`) gets ONE diagnostic — the short-circuit returns
    /// before the table path's in-repo guard can stack a misleading
    /// missing-arm ERROR on the same call.
    #[traced_test]
    #[test]
    fn a_double_colon_spelling_is_rejected_loudly_to_the_catch_all() {
        assert_eq!(slice_ceiling_for_type("tf2_msgs::TFMessage").get(), HUGE);
        assert_eq!(slice_ceiling_for_type("tf2_msgs/msg::Bad").get(), HUGE);
        assert_eq!(slice_ceiling_for_type("tf2_msgs/TFMessage").get(), SMALL);
        logs_assert(|lines: &[&str]| {
            let spelling = count_level(lines, "WARN", "the accepted spelling is 'pkg/Type'");
            let errors = lines
                .iter()
                .filter(|l| l.split_whitespace().any(|t| t == "ERROR"))
                .count();
            if spelling == 2 && errors == 0 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly 2 spelling WARNs and 0 ERRORs (one diagnostic \
                     per mistake), got {spelling} WARNs / {errors} ERRORs"
                ))
            }
        });
    }

    /// A fixed-layout in-repo type's name is a loud `error!` plus the safe
    /// catch-all — NEVER a panic. (The lookup cannot see layouts, only names;
    /// a fixed type has no tier arm, and pre-split the shared catch-all
    /// PANICKED for any unlisted in-repo name.)
    #[traced_test]
    #[test]
    fn an_in_repo_fixed_type_name_is_a_loud_error_not_a_panic() {
        assert_eq!(slice_ceiling_for_type("geometry_msgs/Vector3").get(), HUGE);
        logs_assert(|lines: &[&str]| {
            match count_level(lines, "ERROR", "the tier table does not list") {
                1 => Ok(()),
                n => Err(format!(
                    "expected exactly one ERROR for the in-repo miss, got {n}"
                )),
            }
        });
    }

    /// The codegen wrapper's build-time gate SURVIVED the split: an
    /// unlisted in-repo name still panics there (and only there).
    #[test]
    #[should_panic(expected = "no explicit MAX_SLICE_LEN tier arm")]
    fn the_codegen_wrapper_still_panics_for_an_unlisted_in_repo_name() {
        let _ = crate::codegen::variable_schema_max_slice_len("geometry_msgs/Vector3");
    }

    // ── the parser ────────────────────────────────────────────────────────

    #[test]
    fn a_single_entry_parses_to_its_typed_ceiling() {
        let overrides = SliceCeilingOverrides::parse("sensor_msgs/Image:33554432");
        assert_eq!(overrides.len(), 1);
        assert!(!overrides.is_empty());
        assert_eq!(
            overrides.get("sensor_msgs/Image").map(MaxSliceLen::get),
            Some(33_554_432)
        );
        assert_eq!(overrides.get("sensor_msgs/CompressedImage"), None);
    }

    #[test]
    fn multiple_entries_parse_independently_and_entry_whitespace_is_trimmed() {
        let overrides =
            SliceCeilingOverrides::parse(" sensor_msgs/Image:33554432 , my_pkg/Big:268435456 ");
        assert_eq!(overrides.len(), 2);
        assert_eq!(
            overrides.get("sensor_msgs/Image").map(MaxSliceLen::get),
            Some(33_554_432)
        );
        assert_eq!(
            overrides.get("my_pkg/Big").map(MaxSliceLen::get),
            Some(268_435_456)
        );
    }

    /// The headline strictness pin: a malformed entry is skipped with a WARN
    /// naming it, and its well-formed neighbors still apply.
    #[traced_test]
    #[test]
    fn a_malformed_entry_is_skipped_loudly_and_its_neighbors_survive() {
        let overrides = SliceCeilingOverrides::parse("nonsense,tf2_msgs/TFMessage:65536");
        assert_eq!(overrides.len(), 1);
        assert_eq!(
            overrides.get("tf2_msgs/TFMessage").map(MaxSliceLen::get),
            Some(65_536)
        );
        logs_assert(|lines: &[&str]| {
            let with_entry = lines
                .iter()
                .filter(|l| {
                    l.split_whitespace().any(|t| t == "WARN")
                        && l.contains("entry SKIPPED")
                        && l.contains("nonsense")
                })
                .count();
            match with_entry {
                1 => Ok(()),
                n => Err(format!(
                    "expected exactly one WARN naming the offending entry, got {n}"
                )),
            }
        });
    }

    /// Every malformed NAME/SHAPE class warns with its own message and is
    /// skipped; nothing parses.
    #[traced_test]
    #[test]
    fn each_malformed_shape_warns_distinctly_and_is_skipped() {
        let overrides =
            SliceCeilingOverrides::parse("abc,pkg::Type:64,Image:1024,:64,a/B:12x,a/B:");
        assert!(overrides.is_empty());
        logs_assert(|lines: &[&str]| {
            let pin = |needle: &str, want: usize| -> Result<(), String> {
                let got = count_level(lines, "WARN", needle);
                if got == want {
                    Ok(())
                } else {
                    Err(format!(
                        "needle {needle:?}: expected {want} WARNs, got {got}"
                    ))
                }
            };
            pin("no ':' separator", 1)?;
            pin("never 'pkg::Type'", 1)?;
            pin("not 'pkg/Type'-qualified", 2)?; // `Image:1024` and `:64`
            pin("not an unsigned integer", 2) // `a/B:12x` and the empty `a/B:`
        });
    }

    /// The wire-header floor, pinned on BOTH sides: 31 (and 0) rejected
    /// loudly, 32 accepted exactly.
    #[traced_test]
    #[test]
    fn the_wire_header_floor_is_pinned_on_both_sides() {
        let overrides = SliceCeilingOverrides::parse("a/B:0,c/D:31,e/F:32");
        assert_eq!(overrides.len(), 1);
        assert_eq!(overrides.get("a/B"), None);
        assert_eq!(overrides.get("c/D"), None);
        assert_eq!(overrides.get("e/F").map(MaxSliceLen::get), Some(32));
        logs_assert(|lines: &[&str]| {
            match count_level(lines, "WARN", "below the 32-byte wire-header floor") {
                2 => Ok(()),
                n => Err(format!("expected 2 floor WARNs (0 and 31), got {n}")),
            }
        });
    }

    /// The u32 wire bound, pinned on BOTH sides: `u32::MAX + 1` rejected
    /// loudly, `u32::MAX` accepted exactly.
    #[traced_test]
    #[test]
    fn the_u32_wire_bound_is_pinned_on_both_sides() {
        let overrides = SliceCeilingOverrides::parse("a/B:4294967296,c/D:4294967295");
        assert_eq!(overrides.len(), 1);
        assert_eq!(overrides.get("a/B"), None);
        assert_eq!(overrides.get("c/D").map(MaxSliceLen::get), Some(u32::MAX));
        logs_assert(
            |lines: &[&str]| match count_level(lines, "WARN", "exceeds u32::MAX") {
                1 => Ok(()),
                n => Err(format!("expected 1 u32-bound WARN, got {n}")),
            },
        );
    }

    /// Empty inputs are the quiet shapes: an empty env, a whitespace-only
    /// env, and a trailing comma parse without a single loud line.
    #[traced_test]
    #[test]
    fn empty_env_shapes_parse_to_no_overrides_with_no_warns() {
        assert!(SliceCeilingOverrides::parse("").is_empty());
        assert!(SliceCeilingOverrides::parse("   ").is_empty());
        let trailing = SliceCeilingOverrides::parse("a/B:64,");
        assert_eq!(trailing.len(), 1);
        logs_assert(|lines: &[&str]| {
            match lines
                .iter()
                .filter(|l| l.split_whitespace().any(|t| t == "WARN" || t == "ERROR"))
                .count()
            {
                0 => Ok(()),
                n => Err(format!("empty shapes must be silent, got {n} loud lines")),
            }
        });
    }

    /// Duplicates: the LAST entry wins (so appending to an inherited env
    /// value overrides it), and the replacement is loud, carrying both values
    /// as fields.
    #[traced_test]
    #[test]
    fn a_duplicate_type_warns_and_the_last_entry_wins() {
        let overrides = SliceCeilingOverrides::parse("a/B:64,a/B:128");
        assert_eq!(overrides.len(), 1);
        assert_eq!(overrides.get("a/B").map(MaxSliceLen::get), Some(128));
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&&str> = lines
                .iter()
                .filter(|l| l.contains("the LAST entry wins"))
                .collect();
            match hits.as_slice() {
                [line]
                    if line.split_whitespace().any(|t| t == "WARN")
                        && l_has_field(line, "earlier_bytes=64")
                        && l_has_field(line, "later_bytes=128") =>
                {
                    Ok(())
                }
                other => Err(format!(
                    "expected ONE WARN carrying earlier_bytes=64 later_bytes=128, got {other:?}"
                )),
            }
        });
    }

    /// `key=value` as a whole whitespace token — `earlier_bytes=64` must not
    /// be satisfied by `earlier_bytes=640` (the `has_field` lesson).
    fn l_has_field(line: &str, field: &str) -> bool {
        line.split_whitespace().any(|t| t == field)
    }

    // ── the resolver ──────────────────────────────────────────────────────

    /// Env wins OUTRIGHT, in BOTH directions: it can widen a SMALL type past
    /// its tier and narrow a HUGE one below it (unlike route_budget's
    /// min-compose — the operator's explicit word is adopted verbatim).
    #[test]
    fn an_env_override_wins_over_the_table_in_both_directions() {
        assert_eq!(
            resolve_rmw_slice_ceiling("tf2_msgs/TFMessage", Some("tf2_msgs/TFMessage:1048576"))
                .get(),
            1_048_576 // widened: table says 256 KiB
        );
        assert_eq!(
            resolve_rmw_slice_ceiling("sensor_msgs/Image", Some("sensor_msgs/Image:33554432"))
                .get(),
            33_554_432 // narrowed: table says 128 MiB
        );
    }

    #[test]
    fn an_absent_or_empty_env_falls_to_the_table() {
        assert_eq!(
            resolve_rmw_slice_ceiling("tf2_msgs/TFMessage", None).get(),
            SMALL
        );
        assert_eq!(
            resolve_rmw_slice_ceiling("tf2_msgs/TFMessage", Some("")).get(),
            SMALL
        );
    }

    #[test]
    fn an_env_naming_other_types_leaves_this_type_on_the_table() {
        assert_eq!(
            resolve_rmw_slice_ceiling("tf2_msgs/TFMessage", Some("sensor_msgs/Image:33554432"))
                .get(),
            SMALL
        );
    }

    /// The unknown-type arms: catch-all without an override, the override
    /// verbatim with one — the env exists precisely because bridged topics
    /// have no YAML, and an unknown USER type is its first customer.
    #[test]
    fn an_unknown_type_takes_the_catch_all_unless_overridden() {
        assert_eq!(resolve_rmw_slice_ceiling("my_pkg/Big", None).get(), HUGE);
        assert_eq!(
            resolve_rmw_slice_ceiling("my_pkg/Big", Some("my_pkg/Big:268435456")).get(),
            268_435_456
        );
    }

    /// A hostile env value never panics and never displaces the table answer
    /// — the malformed-entry discipline holds through the resolver.
    #[traced_test]
    #[test]
    fn a_malformed_env_never_panics_and_falls_to_the_table() {
        assert_eq!(
            resolve_rmw_slice_ceiling(
                "tf2_msgs/TFMessage",
                Some("garbage,,pkg::X:1,a/B:0,tf2_msgs/TFMessage:notanumber")
            )
            .get(),
            SMALL
        );
    }

    /// A name that is not EXACTLY two non-empty
    /// components can never match a bridge topic's `pkg/Type`, so storing it
    /// would be a silently-inert override — the operator sets a ceiling, gets
    /// no warning, and the topic keeps the table value. Every such shape is
    /// the same loud warn-and-skip as any other malformed entry, with the
    /// rosidl `pkg/msg/Type` spelling getting its own actionable hint (it is
    /// the mistake real ROS users actually make); the valid control parses.
    #[traced_test]
    #[test]
    fn every_unqualified_name_shape_is_skipped_loudly_and_the_valid_control_parses() {
        let overrides = SliceCeilingOverrides::parse(
            "/Type:64,pkg/:64,pkg/Type/Extra:64,pkg/msg/Image:64,:64,a/B:64",
        );
        assert_eq!(overrides.len(), 1);
        assert_eq!(overrides.get("a/B").map(MaxSliceLen::get), Some(64));
        for never_stored in ["/Type", "pkg/", "pkg/Type/Extra", "pkg/msg/Image", ""] {
            assert_eq!(
                overrides.get(never_stored),
                None,
                "{never_stored:?} must not be stored"
            );
        }
        logs_assert(|lines: &[&str]| {
            let pin = |needle: &str, want: usize| -> Result<(), String> {
                let got = count_level(lines, "WARN", needle);
                if got == want {
                    Ok(())
                } else {
                    Err(format!(
                        "needle {needle:?}: expected {want} WARNs, got {got}"
                    ))
                }
            };
            pin("not 'pkg/Type'-qualified", 4)?; // /Type, pkg/, pkg/Type/Extra, :64
            pin("did you mean 'pkg/Type'", 1) // pkg/msg/Image — the rosidl hint
        });
    }

    /// Whitespace INSIDE an
    /// entry's name — a trailing space before the `:` or a space inside a
    /// component — must be a loud skip, never stored: the shape check alone
    /// accepted them, and a stored `"sensor_msgs/Image "` can never equal the
    /// bridge's exact `sensor_msgs/Image` — the same silently-inert class
    /// one notch over. Surrounding whitespace per entry stays FRIENDLY (a
    /// space after the list comma is how humans write lists — trimmed from
    /// the start; the control half below must stay warn-free).
    #[traced_test]
    #[test]
    fn whitespace_inside_a_name_is_skipped_loudly_while_entry_edges_stay_trimmed() {
        let overrides = SliceCeilingOverrides::parse(
            "sensor_msgs/Image :33554432,sensor_msgs/ Image:33554432,sensor msgs/Image:64,a/B:64",
        );
        assert_eq!(overrides.len(), 1);
        assert_eq!(overrides.get("a/B").map(MaxSliceLen::get), Some(64));
        assert_eq!(overrides.get("sensor_msgs/Image"), None);
        for never_stored in [
            "sensor_msgs/Image ",
            "sensor_msgs/ Image",
            "sensor msgs/Image",
        ] {
            assert_eq!(
                overrides.get(never_stored),
                None,
                "{never_stored:?} must not be stored"
            );
        }
        // The FRIENDLY half: entry-EDGE whitespace (comma-space lists, tabs)
        // still trims and parses — with no warns at all.
        let friendly = SliceCeilingOverrides::parse("\tc/D:128 , e/F:256");
        assert_eq!(friendly.len(), 2);
        assert_eq!(friendly.get("c/D").map(MaxSliceLen::get), Some(128));
        assert_eq!(friendly.get("e/F").map(MaxSliceLen::get), Some(256));
        logs_assert(
            |lines: &[&str]| match count_level(lines, "WARN", "contains whitespace") {
                3 => Ok(()),
                n => Err(format!("expected 3 whitespace WARNs, got {n}")),
            },
        );
    }

    /// The LOOKUP warns on the rosidl `pkg/msg/Type` spelling
    /// too — a provably-wrong spelling must not silently take the catch-all —
    /// while the correctly spelled twin resolves to its REAL tier (SMALL, not
    /// the catch-all: the discriminator).
    ///
    /// ONE mistake gets EXACTLY ONE
    /// diagnostic. Falling through, the rosidl spelling would emit the did-you-mean WARN
    /// and then reach the table path, where the in-repo guard
    /// (recognizing `tf2_msgs`) stacks a second, MISLEADING error —
    /// "fixed-layout type or missing table arm", neither of which is true of
    /// a misspelling. The known-wrong spelling must short-circuit to the
    /// catch-all without consulting the table, so the oracle counts TOTAL
    /// loud lines, not just hint WARNs.
    #[traced_test]
    #[test]
    fn the_lookup_warns_on_the_rosidl_three_segment_spelling() {
        assert_eq!(slice_ceiling_for_type("tf2_msgs/msg/TFMessage").get(), HUGE);
        assert_eq!(slice_ceiling_for_type("tf2_msgs/TFMessage").get(), SMALL);
        logs_assert(|lines: &[&str]| {
            let loud = lines
                .iter()
                .filter(|l| l.split_whitespace().any(|t| t == "WARN" || t == "ERROR"))
                .count();
            let hint = count_level(lines, "WARN", "did you mean 'pkg/Type'");
            if hint == 1 && loud == 1 {
                Ok(())
            } else {
                Err(format!(
                    "one mistake must get exactly ONE diagnostic — the did-you-mean \
                     WARN (hint WARNs: {hint}, total loud lines: {loud}); a second \
                     line means the known-wrong spelling fell through to the table \
                     path's in-repo guard"
                ))
            }
        });
    }
}
