// SPDX-License-Identifier: AGPL-3.0-only
//! The `--tolerance` YAML schema, its strict validation, and
//! the per-topic/per-field metric-precedence resolver.
//!
//! `cerulion bag play --resim --verify` is BYTE-EXACT by default (Principle
//! #7). A tolerance
//! document relaxes that on nominated topics/fields — e.g. a cross-arch replay
//! whose transcendental-heavy node output differs in float ULPs (a real, not a
//! regression, divergence) is compared under `max_abs`/`rmse` instead of
//! byte-equality; a detector's bounding boxes under `bbox_iou`; an unordered
//! set of tracks under `set_equal`.
//!
//! # This module (the tolerance DOCUMENT)
//!
//! This module owns the tolerance DOCUMENT: its [`ToleranceSpec`] serde shape,
//! the strict parse (deny-unknown at EVERY level so a misspelled key is a hard
//! exit-4 error, never silently ignored), the numeric range checks, the
//! [`ToleranceSpec::resolve`] precedence lookup (`fields[path]` >
//! `topic.metric` > `default_metric`), and the
//! [`FieldResolver::check_field_metric`] validation that refuses a
//! non-`bit_exact` metric on a publisher-opaque field (exit 4). A
//! non-`bit_exact` topic-wide `metric:` or document `default_metric` is
//! FUNCTIONAL (it expands over the topic's schema fields at the diff seam), so
//! [`ToleranceSpec::validate`] gates it against EVERY covered field via
//! [`FieldResolver::topic_field_names`] — an opaque/non-decodable field under
//! such a metric is exit-4 naming the field. It is a pure DESCRIPTOR module —
//! [`MetricKind`] carries the metric parameters; the comparison MATH lives in
//! [`crate::tolerance_metrics`] and is driven by the diff loop in
//! [`crate::replay_engine`], which relaxes each nominated per-field override by
//! its metric while the remainder of the frame stays byte-exact.
//!
//! # The exit-4 gate
//!
//! [`crate::replay_cmd::run_replay`] parses + validates the document as a
//! PRE-FLIGHT gate (after the bag's graph attachment loads, BEFORE the replay
//! engine constructs the runtime), so a bad tolerance YAML fails FAST (exit 4)
//! without ever loading a node cdylib or touching the transport. Every
//! validation failure is a [`ReplayError::ToleranceInvalid`](crate::replay_cmd::ReplayError::ToleranceInvalid).
//!
//! Validation resolves each `topics:` key against the replay topic set and each
//! `fields:` path against that topic's schema (via the
//! [`FieldResolver`] the caller supplies — the real one is
//! [`crate::replay_field_registry`]), and an unresolvable reference is refused
//! with the offending name AND ranked Levenshtein "did you mean …?" suggestions.

use indexmap::IndexMap;
use serde::Deserialize;

/// A per-topic comparison metric descriptor (the parameters; the comparison
/// MATH lives in [`crate::tolerance_metrics`]).
///
/// Internally tagged on `kind` (`{ kind: max_abs, threshold: 0.01 }`). Every
/// variant — including the parameterless ones, which are declared as EMPTY
/// STRUCT variants (`{}`) for exactly this reason — carries
/// `#[serde(deny_unknown_fields)]` via the enum-level attribute, so a stray or
/// misspelled key (`{ kind: max_abs, theshold: 0.01 }` or `{ kind: bit_exact,
/// threshold: 0.01 }`) is a HARD error, never silently dropped. Empirically
/// verified: serde ignores extra fields on a plain unit variant, so the `{}`
/// spelling is load-bearing.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum MetricKind {
    /// Full-frame byte equality — the default. Equivalent to no tolerance at
    /// all for the covered scope.
    BitExact {},
    /// Per-element absolute-difference bound: every numeric element must be
    /// within `threshold` of its recorded twin.
    MaxAbs {
        /// Absolute tolerance (`>= 0`; `< 0` is exit-4-invalid).
        threshold: f64,
    },
    /// Per-element relative-difference bound.
    MaxRel {
        /// Relative tolerance (`>= 0`).
        threshold: f64,
    },
    /// Root-mean-square-error bound over the field's elements.
    Rmse {
        /// RMSE tolerance (`>= 0`).
        threshold: f64,
    },
    /// Bounding-box intersection-over-union floor (object-detection outputs).
    BboxIou {
        /// Minimum acceptable IoU, in `[0.0, 1.0]` (outside is exit-4-invalid).
        min_iou: f64,
    },
    /// Order-insensitive set equality (e.g. an unordered track set).
    SetEqual {},
    /// The recorded set must be a subset of the replayed set.
    SetSubset {},
    /// Order-SENSITIVE list equality (element-wise, position-fixed).
    OrderedListEqual {},
}

impl MetricKind {
    /// serde default for [`ToleranceSpec::default_metric`] — an omitted
    /// `default_metric:` means byte-exact (replay's standing behavior).
    fn default_bit_exact() -> Self {
        MetricKind::BitExact {}
    }

    /// `true` for the byte-equality metric — the metric the diff engine folds
    /// into the byte-exact remainder (never a per-field numeric comparison).
    pub fn is_bit_exact(&self) -> bool {
        matches!(self, MetricKind::BitExact {})
    }

    /// A short, stable, snake_case label for this metric (the `metric` field of
    /// a [`crate::replay_engine::ViolationClass::ToleranceExceeded`] +
    /// verdict line). Matches the YAML `kind:` spelling.
    pub fn label(&self) -> &'static str {
        match self {
            MetricKind::BitExact {} => "bit_exact",
            MetricKind::MaxAbs { .. } => "max_abs",
            MetricKind::MaxRel { .. } => "max_rel",
            MetricKind::Rmse { .. } => "rmse",
            MetricKind::BboxIou { .. } => "bbox_iou",
            MetricKind::SetEqual {} => "set_equal",
            MetricKind::SetSubset {} => "set_subset",
            MetricKind::OrderedListEqual {} => "ordered_list_equal",
        }
    }

    /// The numeric threshold to REPORT alongside a violation: the `threshold`
    /// for the bounded metrics, `min_iou` for `bbox_iou`, and the implicit
    /// `0.0` for the discrete (equality/set/list/bit_exact) metrics whose
    /// "any mismatch" is reported as `worst_value 1.0` vs `threshold 0.0`.
    pub fn report_threshold(&self) -> f64 {
        match self {
            MetricKind::MaxAbs { threshold }
            | MetricKind::MaxRel { threshold }
            | MetricKind::Rmse { threshold } => *threshold,
            MetricKind::BboxIou { min_iou } => *min_iou,
            MetricKind::BitExact {}
            | MetricKind::SetEqual {}
            | MetricKind::SetSubset {}
            | MetricKind::OrderedListEqual {} => 0.0,
        }
    }

    /// Validate this metric's numeric parameters, returning a human reason on
    /// failure (`threshold >= 0`; `min_iou` in `[0, 1]`; no NaN). The scope
    /// label (`"topic '/x' field 'y'"` etc.) is prepended by the caller.
    fn validate_ranges(&self) -> Result<(), String> {
        match self {
            MetricKind::MaxAbs { threshold }
            | MetricKind::MaxRel { threshold }
            | MetricKind::Rmse { threshold } => {
                if threshold.is_nan() {
                    return Err("threshold is NaN".to_string());
                }
                if *threshold < 0.0 {
                    return Err(format!("threshold {threshold} is negative (must be >= 0)"));
                }
                Ok(())
            }
            MetricKind::BboxIou { min_iou } => {
                if min_iou.is_nan() {
                    return Err("min_iou is NaN".to_string());
                }
                if *min_iou < 0.0 || *min_iou > 1.0 {
                    return Err(format!(
                        "min_iou {min_iou} is outside the valid range [0.0, 1.0]"
                    ));
                }
                Ok(())
            }
            MetricKind::BitExact {}
            | MetricKind::SetEqual {}
            | MetricKind::SetSubset {}
            | MetricKind::OrderedListEqual {} => Ok(()),
        }
    }
}

/// Per-topic tolerance: an optional topic-wide `metric:` plus per-field
/// overrides. An empty entry (`{}`) is legal and means "use `default_metric`
/// for this topic" — the same as not listing the topic at all, but explicit.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopicTolerance {
    /// Per-field metric overrides, keyed by dotted field path (`detections.bbox`).
    /// Highest precedence. Order-preserving (deterministic validation output).
    #[serde(default)]
    pub fields: IndexMap<String, MetricKind>,
    /// The topic-wide metric (applies to every field with no `fields:`
    /// override). `None` falls through to `default_metric`.
    #[serde(default)]
    pub metric: Option<MetricKind>,
}

/// A parsed `--tolerance` document.
///
/// ```yaml
/// default_metric:            # optional; omitted == bit_exact
///   kind: bit_exact
/// topics:
///   /perception/detections:
///     metric:                # topic-wide default for this topic
///       kind: bbox_iou
///       min_iou: 0.7
///     fields:
///       score:               # per-field override (highest precedence)
///         kind: max_abs
///         threshold: 0.01
/// ```
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToleranceSpec {
    /// The document-wide default metric — applies to any topic/field with no
    /// more-specific entry. Defaults to [`MetricKind::BitExact`].
    #[serde(default = "MetricKind::default_bit_exact")]
    pub default_metric: MetricKind,
    /// Per-topic entries, keyed by RESOLVED topic name. Order-preserving.
    #[serde(default)]
    pub topics: IndexMap<String, TopicTolerance>,
}

impl ToleranceSpec {
    /// Parse a tolerance YAML string with strict (deny-unknown) semantics.
    ///
    /// A parse error (syntax, unknown key at any level, wrong type, missing
    /// required metric parameter, unknown metric `kind`) is returned as a
    /// human-readable reason — the caller wraps it in
    /// [`ReplayError::ToleranceInvalid`](crate::replay_cmd::ReplayError::ToleranceInvalid)
    /// (exit 4). Does NOT do topic/field-name resolution or range checks — see
    /// [`Self::validate`].
    pub fn parse(yaml: &str) -> Result<Self, String> {
        serde_yaml::from_str(yaml).map_err(|e| e.to_string())
    }

    /// The metric that applies to `topic` / `field_path` under the precedence
    /// `fields[path]` > `topic.metric` > `default_metric`. Pure lookup, no
    /// validation (call [`Self::validate`] first).
    pub fn resolve(&self, topic: &str, field_path: &str) -> &MetricKind {
        if let Some(t) = self.topics.get(topic) {
            if let Some(m) = t.fields.get(field_path) {
                return m;
            }
            if let Some(m) = &t.metric {
                return m;
            }
        }
        &self.default_metric
    }

    /// Validate the document against a [`FieldResolver`] (the replay topic +
    /// schema registry): numeric ranges on every metric, every `topics:` key
    /// resolves to a real replay topic, every `fields:` path resolves against
    /// that topic's schema. The first failure is returned as a human reason
    /// (exit 4), with ranked "did you mean …?" suggestions on an unresolvable
    /// name. Deterministic: topics + fields are walked in document order.
    pub fn validate(&self, resolver: &dyn FieldResolver) -> Result<(), String> {
        // Range checks first (cheapest, name-independent). default_metric, then
        // each topic metric + field metric in document order.
        self.default_metric
            .validate_ranges()
            .map_err(|r| format!("default_metric: {r}"))?;
        for (topic, tol) in &self.topics {
            if let Some(m) = &tol.metric {
                m.validate_ranges()
                    .map_err(|r| format!("topic '{topic}' metric: {r}"))?;
            }
            for (path, m) in &tol.fields {
                m.validate_ranges()
                    .map_err(|r| format!("topic '{topic}' field '{path}': {r}"))?;
            }
        }

        // Name resolution: topics first, then fields per topic.
        let known_topics = resolver.topics();
        for (topic, tol) in &self.topics {
            if !known_topics.iter().any(|t| t == topic) {
                // Topic typo. Build topic suggestions; on a strong best match,
                // ALSO validate this entry's field paths against the SUGGESTED
                // topic so a composed topic+field typo surfaces BOTH corrections
                // in one error (the category-A load-bearing pin).
                let topic_sugg = suggest(topic, &known_topics);
                let mut reason = format!(
                    "tolerance names topic '{topic}', which is not a topic in this recording"
                );
                if !topic_sugg.is_empty() {
                    reason.push_str(&format!(" ({})", did_you_mean(&topic_sugg)));
                }
                // Composed-typo trap: check the fields against the best topic
                // candidate (if any) and append the first field suggestion.
                if let Some(best) = topic_sugg.first() {
                    for (path, _) in &tol.fields {
                        if let Err(fe) = resolver.resolve_field(best, path) {
                            let field_sugg = suggest(&fe.segment, &fe.candidates);
                            reason.push_str(&format!(
                                "; and under the intended topic '{best}', field path '{path}' \
                                 is also unresolvable at segment '{}'",
                                fe.segment
                            ));
                            if !field_sugg.is_empty() {
                                reason.push_str(&format!(" ({})", did_you_mean(&field_sugg)));
                            }
                            break;
                        }
                    }
                }
                return Err(reason);
            }
            // Topic resolves — validate every field path against its schema.
            for (path, metric) in &tol.fields {
                if let Err(fe) = resolver.resolve_field(topic, path) {
                    let mut reason = if fe.schema_unavailable {
                        format!(
                            "tolerance sets a per-field metric on topic '{topic}' field path \
                             '{path}', but this topic's schema is not resolvable for field-level \
                             validation in this workspace"
                        )
                    } else {
                        format!(
                            "tolerance names field path '{path}' on topic '{topic}', which does \
                             not resolve at segment '{}'",
                            fe.segment
                        )
                    };
                    let field_sugg = suggest(&fe.segment, &fe.candidates);
                    if !field_sugg.is_empty() {
                        reason.push_str(&format!(" ({})", did_you_mean(&field_sugg)));
                    }
                    return Err(reason);
                }
                // The field resolves — now gate the METRIC against the
                // field's decodability (a non-bit_exact metric on a
                // publisher-opaque / non-numeric field is exit-4, LOUDLY).
                resolver
                    .check_field_metric(topic, path, metric)
                    .map_err(|reason| {
                        format!("tolerance on topic '{topic}' field '{path}': {reason}")
                    })?;
            }
        }

        // Topic-wide `metric:` and document `default_metric` COVERAGE. A
        // non-`bit_exact` topic-wide/default metric is applied per-field at the
        // diff seam (see `build_capture_tolerance`), so validate it against EVERY
        // schema field it would cover — an opaque/non-decodable field (or a
        // 64-bit-int field under a lossy metric) is exit-4, naming the field. This
        // is the strictest reading: the default `default_metric` is `bit_exact`,
        // so this bites only a DELIBERATE topic-wide or global relaxation.
        // Precedence: a field with an explicit top-level `fields:` override was
        // already gated above and is skipped here.
        let default_non_bit = !self.default_metric.is_bit_exact();
        for topic in &known_topics {
            let topic_tol = self.topics.get(topic);
            let topic_metric = topic_tol.and_then(|t| t.metric.as_ref());
            let topic_metric_non_bit = topic_metric.is_some_and(|m| !m.is_bit_exact());
            // Nothing to expand unless a topic-wide OR the global default metric
            // is non-`bit_exact` (a `bit_exact` topic-wide metric SHADOWS a
            // non-bit default for this topic — the topic stays byte-exact).
            let default_applies_here = default_non_bit && topic_metric.is_none();
            if !topic_metric_non_bit && !default_applies_here {
                continue;
            }
            let fields = match resolver.topic_field_names(topic) {
                Some(f) => f,
                None => {
                    // Schema unavailable. An EXPLICIT non-`bit_exact` topic-wide
                    // metric on such a topic is exit-4 (the user nominated a topic
                    // that resists field-wise decoding); a global `default_metric`
                    // simply does not relax an unresolvable-schema topic (it stays
                    // byte-exact — stricter, never a false pass), so skip.
                    if topic_metric_non_bit {
                        return Err(format!(
                            "tolerance sets a topic-wide '{}' metric on topic '{topic}', but this \
                             topic's schema is not resolvable for field-level validation in this \
                             workspace; remove the topic-wide metric or use a resolvable schema",
                            topic_metric
                                .expect("topic_metric_non_bit implies Some")
                                .label()
                        ));
                    }
                    continue;
                }
            };
            let (resolved, scope) = if let Some(m) = topic_metric {
                (m, "topic-wide metric")
            } else {
                (&self.default_metric, "default_metric")
            };
            if resolved.is_bit_exact() {
                continue;
            }
            for field in &fields {
                // An explicit per-field override wins (precedence) and was already
                // gated in the loop above — skip it here.
                if topic_tol.is_some_and(|t| t.fields.contains_key(field)) {
                    continue;
                }
                resolver
                    .check_field_metric(topic, field, resolved)
                    .map_err(|reason| {
                        format!("tolerance {scope} on topic '{topic}' field '{field}': {reason}")
                    })?;
            }
        }
        Ok(())
    }
}

/// The schema/topic knowledge the tolerance validator needs — implemented by
/// [`crate::replay_field_registry::FieldRegistry`] over the real graph +
/// workspace schemas, and by test doubles for hermetic unit tests.
pub trait FieldResolver {
    /// Every valid replay topic name (graph-produced ∪ external), in a stable
    /// order. A tolerance `topics:` key must appear here.
    fn topics(&self) -> Vec<String>;

    /// Resolve a dotted field `path` against `topic`'s schema. `Ok(())` when
    /// the whole path resolves; [`FieldResolveError`] names the first failing
    /// segment + the sibling candidate names at that level (for suggestions).
    fn resolve_field(&self, topic: &str, path: &str) -> Result<(), FieldResolveError>;

    /// Decodability classification: given a field `path` that
    /// already RESOLVES (call after [`Self::resolve_field`]), decide whether
    /// `metric` can actually be APPLIED to it. `bit_exact` is legal on any
    /// resolvable field (byte compare needs no decode). A non-`bit_exact`
    /// metric is legal only on a metric-DECODABLE field (a top-level numeric
    /// scalar or a `float64[]`-class sequence); a field that resolves but is
    /// publisher-OPAQUE (a nested / `DynamicArray<Nested>` / dotted per-element
    /// path, or a non-numeric leaf) is refused with the decided exit-4 message.
    /// The default impl accepts everything (test doubles that do not model wire
    /// layout); [`crate::replay_field_registry::FieldRegistry`] overrides it.
    fn check_field_metric(
        &self,
        _topic: &str,
        _path: &str,
        _metric: &MetricKind,
    ) -> Result<(), String> {
        Ok(())
    }

    /// Topic-wide + default_metric coverage: the TOP-LEVEL field names of
    /// `topic`'s root schema (fixed ∪ variable, declaration order) — the domain a
    /// topic-wide `metric:` or the document `default_metric` expands over. Used by
    /// [`ToleranceSpec::validate`] to gate every schema field the resolved metric
    /// would actually be applied to (via [`Self::check_field_metric`]).
    ///
    /// `Some(vec![])` — the DEFAULT impl — means "this resolver does not model the
    /// wire layout" (test doubles), so a topic-wide/default metric has no fields
    /// to gate and its coverage validation is a permissive no-op (mirrors the
    /// permissive [`Self::check_field_metric`] default). The real
    /// [`crate::replay_field_registry::FieldRegistry`] returns the actual field
    /// set, or `None` when the topic's schema is UNAVAILABLE — a non-`bit_exact`
    /// topic-wide metric on such a topic is then exit-4 (a global `default_metric`
    /// simply does not relax an unresolvable-schema topic; it stays byte-exact).
    fn topic_field_names(&self, _topic: &str) -> Option<Vec<String>> {
        Some(Vec::new())
    }
}

/// Why a field path did not resolve (drives the exit-4 message + suggestions).
#[derive(Debug, Clone)]
pub struct FieldResolveError {
    /// The first path segment that failed to resolve — the subject of the
    /// "did you mean …?" suggestion.
    pub segment: String,
    /// The field names available at the level `segment` was looked up in
    /// (suggestion candidates). Empty when the level has no named children
    /// (e.g. indexing into a primitive) or when the schema is unavailable.
    pub candidates: Vec<String>,
    /// The topic is known but its schema could not be resolved at all, so
    /// field-level validation was impossible (distinct from a genuine typo).
    pub schema_unavailable: bool,
}

/// Rank `candidates` as corrections for `offending`: keep those within the edit
/// budget (`<= 2` OR `<= ceil(len/3)` edits, whichever is larger), sorted by
/// `(distance, name)` for a deterministic order.
fn suggest(offending: &str, candidates: &[String]) -> Vec<String> {
    let budget = edit_budget(offending.len());
    let mut ranked: Vec<(usize, &String)> = candidates
        .iter()
        .filter_map(|c| {
            let d = levenshtein(offending, c);
            (d <= budget).then_some((d, c))
        })
        .collect();
    // Sort by (distance, name) — total + deterministic. `sort_by` is stable but
    // the name tiebreak makes the order independent of the input order anyway.
    ranked.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.cmp(b.1)));
    ranked.into_iter().map(|(_, c)| c.clone()).collect()
}

/// The Levenshtein edit budget for a name of length `len`: `max(2, ceil(len/3))`.
fn edit_budget(len: usize) -> usize {
    2.max(len.div_ceil(3))
}

/// Render a suggestion list as `did you mean \`a\`, \`b\`?`. Never called with
/// an empty list (callers guard).
fn did_you_mean(suggestions: &[String]) -> String {
    let quoted: Vec<String> = suggestions.iter().map(|s| format!("`{s}`")).collect();
    format!("did you mean {}?", quoted.join(", "))
}

/// Classic Levenshtein edit distance (two-row DP, no allocation beyond one row).
/// Operates on Unicode scalar values (chars), so a multibyte name compares
/// sensibly.
fn levenshtein(a: &str, b: &str) -> usize {
    let b_chars: Vec<char> = b.chars().collect();
    let n = b_chars.len();
    // prev[j] = distance from a[..i] to b[..j].
    let mut prev: Vec<usize> = (0..=n).collect();
    let mut curr: Vec<usize> = vec![0; n + 1];
    for (i, ca) in a.chars().enumerate() {
        curr[0] = i + 1;
        for (j, &cb) in b_chars.iter().enumerate() {
            let cost = if ca == cb { 0 } else { 1 };
            curr[j + 1] = (prev[j + 1] + 1).min(curr[j] + 1).min(prev[j] + cost);
        }
        std::mem::swap(&mut prev, &mut curr);
    }
    prev[n]
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Parse + deny_unknown_fields at every level ────────────────────────

    #[test]
    fn empty_document_defaults_to_bit_exact_no_topics() {
        let spec = ToleranceSpec::parse("{}").unwrap();
        assert_eq!(spec.default_metric, MetricKind::BitExact {});
        assert!(spec.topics.is_empty());
        // A truly empty file parses to the same defaults.
        let spec2 = ToleranceSpec::parse("").unwrap();
        assert_eq!(spec2.default_metric, MetricKind::BitExact {});
        assert!(spec2.topics.is_empty());
    }

    #[test]
    fn full_document_parses_every_metric_shape() {
        let yaml = "\
default_metric:
  kind: max_rel
  threshold: 0.001
topics:
  /a:
    metric:
      kind: bbox_iou
      min_iou: 0.7
    fields:
      score:
        kind: max_abs
        threshold: 0.01
      tracks:
        kind: set_equal
  /b: {}
";
        let spec = ToleranceSpec::parse(yaml).unwrap();
        assert_eq!(spec.default_metric, MetricKind::MaxRel { threshold: 0.001 });
        let a = &spec.topics["/a"];
        assert_eq!(a.metric, Some(MetricKind::BboxIou { min_iou: 0.7 }));
        assert_eq!(a.fields["score"], MetricKind::MaxAbs { threshold: 0.01 });
        assert_eq!(a.fields["tracks"], MetricKind::SetEqual {});
        // Empty topic entry is legal.
        let b = &spec.topics["/b"];
        assert!(b.fields.is_empty() && b.metric.is_none());
    }

    #[test]
    fn unknown_key_at_spec_level_is_rejected() {
        let err = ToleranceSpec::parse("defualt_metric:\n  kind: bit_exact").unwrap_err();
        assert!(err.contains("defualt_metric"), "got: {err}");
    }

    #[test]
    fn unknown_key_at_topic_level_is_rejected() {
        let err = ToleranceSpec::parse("topics:\n  /a:\n    feilds: {}").unwrap_err();
        assert!(err.contains("feilds"), "got: {err}");
    }

    #[test]
    fn misspelled_threshold_key_is_rejected_not_ignored() {
        // The headline deny-unknown pin: a typo on a metric parameter is a hard
        // error, NOT a silent fallback to the metric with a defaulted threshold.
        let err = ToleranceSpec::parse(
            "topics:\n  /a:\n    fields:\n      x:\n        kind: max_abs\n        theshold: 0.5",
        )
        .unwrap_err();
        assert!(err.contains("theshold"), "got: {err}");
    }

    #[test]
    fn extra_field_on_parameterless_metric_is_rejected() {
        // The empty-struct-variant spelling makes even a unit-like metric deny
        // extras (a plain unit variant would SILENTLY ignore `threshold`).
        let err = ToleranceSpec::parse("default_metric:\n  kind: bit_exact\n  threshold: 0.5")
            .unwrap_err();
        assert!(err.contains("threshold"), "got: {err}");
    }

    #[test]
    fn missing_required_metric_parameter_is_rejected() {
        let err = ToleranceSpec::parse("default_metric:\n  kind: max_abs").unwrap_err();
        assert!(err.contains("threshold"), "got: {err}");
    }

    #[test]
    fn unknown_metric_kind_is_rejected() {
        let err = ToleranceSpec::parse("default_metric:\n  kind: nonsense").unwrap_err();
        assert!(err.contains("nonsense"), "got: {err}");
    }

    #[test]
    fn unknown_metric_kind_error_lists_all_eight_valid_kinds() {
        // E-1 (4d): an unknown `kind:` must name EVERY valid metric so the user
        // can self-correct. serde's internally-tagged "unknown variant" error
        // enumerates all snake_case variants — pin that all EIGHT appear.
        let err = ToleranceSpec::parse("default_metric:\n  kind: nonsense").unwrap_err();
        for kind in [
            "bit_exact",
            "max_abs",
            "max_rel",
            "rmse",
            "bbox_iou",
            "set_equal",
            "set_subset",
            "ordered_list_equal",
        ] {
            assert!(
                err.contains(kind),
                "the unknown-kind error must list '{kind}': {err}"
            );
        }
    }

    #[test]
    fn topic_order_is_preserved_from_document() {
        let spec = ToleranceSpec::parse("topics:\n  /z: {}\n  /a: {}\n  /m: {}").unwrap();
        let order: Vec<&str> = spec.topics.keys().map(String::as_str).collect();
        assert_eq!(order, vec!["/z", "/a", "/m"]);
    }

    // ── Range validation ──────────────────────────────────────────────────

    /// A resolver that accepts every topic + field (isolates range checks).
    struct AllowAll;
    impl FieldResolver for AllowAll {
        fn topics(&self) -> Vec<String> {
            vec!["/a".to_string()]
        }
        fn resolve_field(&self, _topic: &str, _path: &str) -> Result<(), FieldResolveError> {
            Ok(())
        }
    }

    #[test]
    fn negative_threshold_is_range_invalid() {
        let spec = ToleranceSpec::parse(
            "topics:\n  /a:\n    fields:\n      x:\n        kind: rmse\n        threshold: -0.1",
        )
        .unwrap();
        let err = spec.validate(&AllowAll).unwrap_err();
        assert!(
            err.contains("negative") && err.contains("field 'x'"),
            "got: {err}"
        );
    }

    #[test]
    fn min_iou_out_of_unit_range_is_invalid() {
        for bad in ["1.5", "-0.2"] {
            let spec = ToleranceSpec::parse(&format!(
                "default_metric:\n  kind: bbox_iou\n  min_iou: {bad}"
            ))
            .unwrap();
            let err = spec.validate(&AllowAll).unwrap_err();
            assert!(
                err.contains("min_iou") && err.contains("default_metric"),
                "got: {err}"
            );
        }
    }

    #[test]
    fn min_iou_at_bounds_is_valid() {
        for ok in ["0.0", "1.0"] {
            let spec = ToleranceSpec::parse(&format!(
                "default_metric:\n  kind: bbox_iou\n  min_iou: {ok}"
            ))
            .unwrap();
            spec.validate(&AllowAll).expect("boundary min_iou is valid");
        }
    }

    #[test]
    fn nan_threshold_is_invalid() {
        let spec =
            ToleranceSpec::parse("default_metric:\n  kind: max_abs\n  threshold: .nan").unwrap();
        let err = spec.validate(&AllowAll).unwrap_err();
        assert!(err.contains("NaN"), "got: {err}");
    }

    // ── Precedence resolution ─────────────────────────────────────────────

    #[test]
    fn resolve_precedence_field_over_topic_over_default() {
        let yaml = "\
default_metric:
  kind: max_rel
  threshold: 0.1
topics:
  /a:
    metric:
      kind: rmse
      threshold: 0.2
    fields:
      x:
        kind: max_abs
        threshold: 0.3
";
        let spec = ToleranceSpec::parse(yaml).unwrap();
        // field override wins
        assert_eq!(
            spec.resolve("/a", "x"),
            &MetricKind::MaxAbs { threshold: 0.3 }
        );
        // topic metric for a field with no override
        assert_eq!(
            spec.resolve("/a", "y"),
            &MetricKind::Rmse { threshold: 0.2 }
        );
        // default for an unlisted topic
        assert_eq!(
            spec.resolve("/b", "z"),
            &MetricKind::MaxRel { threshold: 0.1 }
        );
    }

    #[test]
    fn empty_topic_entry_resolves_to_default() {
        let spec = ToleranceSpec::parse(
            "default_metric:\n  kind: rmse\n  threshold: 0.5\ntopics:\n  /a: {}",
        )
        .unwrap();
        // An empty entry has no metric/fields, so every field falls to default.
        assert_eq!(
            spec.resolve("/a", "anything"),
            &MetricKind::Rmse { threshold: 0.5 }
        );
    }

    // ── Topic-wide + default_metric COVERAGE validation ────────────
    //
    // A schema-modelling double: each topic maps to its top-level field names,
    // and a field named "opaque" is metric-DECODABLE only under bit_exact (like
    // the real registry's publisher-opaque leaf), while "u64" is refused under
    // the lossy `max_rel`/`rmse` metrics (the 64-bit-int composition). Every
    // other field accepts any metric. A topic ABSENT from the map has an
    // UNAVAILABLE schema (`topic_field_names` → `None`).

    struct SchemaResolver {
        /// topic → Some(field names) when the schema is available; None ⇒ absent.
        topics: IndexMap<String, Option<Vec<String>>>,
    }
    impl SchemaResolver {
        fn new(entries: &[(&str, Option<&[&str]>)]) -> Self {
            let topics = entries
                .iter()
                .map(|(t, fs)| {
                    (
                        t.to_string(),
                        fs.map(|f| f.iter().map(|s| s.to_string()).collect()),
                    )
                })
                .collect();
            SchemaResolver { topics }
        }
    }
    impl FieldResolver for SchemaResolver {
        fn topics(&self) -> Vec<String> {
            self.topics.keys().cloned().collect()
        }
        fn resolve_field(&self, topic: &str, path: &str) -> Result<(), FieldResolveError> {
            let seg = path.split('.').next().unwrap_or(path);
            let fields = self.topics.get(topic).and_then(|f| f.as_ref());
            match fields {
                Some(fs) if fs.iter().any(|f| f == seg) => Ok(()),
                Some(fs) => Err(FieldResolveError {
                    segment: seg.to_string(),
                    candidates: fs.clone(),
                    schema_unavailable: false,
                }),
                None => Err(FieldResolveError {
                    segment: seg.to_string(),
                    candidates: vec![],
                    schema_unavailable: true,
                }),
            }
        }
        fn topic_field_names(&self, topic: &str) -> Option<Vec<String>> {
            self.topics.get(topic).and_then(|f| f.clone())
        }
        fn check_field_metric(
            &self,
            _topic: &str,
            path: &str,
            metric: &MetricKind,
        ) -> Result<(), String> {
            if metric.is_bit_exact() {
                return Ok(());
            }
            let seg = path.split('.').next().unwrap_or(path);
            if seg == "opaque" {
                return Err(format!(
                    "the metric '{}' needs a metric-decodable field, but '{path}' is \
                     publisher-opaque; only bit_exact applies — use a decodable layout or \
                     bit_exact",
                    metric.label()
                ));
            }
            if seg == "u64" && matches!(metric, MetricKind::MaxRel { .. } | MetricKind::Rmse { .. })
            {
                return Err(format!(
                    "the metric '{}' is precision-lossy on 64-bit integers (field '{path}'); use \
                     max_abs or bit_exact",
                    metric.label()
                ));
            }
            Ok(())
        }
    }

    #[test]
    fn topic_wide_metric_rejects_an_opaque_field_naming_it_and_the_remedy() {
        // A bare topic-wide non-bit_exact metric must be decodable on EVERY field
        // of the topic — the opaque field is exit-4, named, with the remedy.
        let spec = ToleranceSpec::parse(
            "topics:\n  /a:\n    metric:\n      kind: max_abs\n      threshold: 0.1",
        )
        .unwrap();
        let r = SchemaResolver::new(&[("/a", Some(&["good", "opaque"]))]);
        let err = spec.validate(&r).unwrap_err();
        assert!(
            err.contains("topic-wide metric")
                && err.contains("'opaque'")
                && err.contains("bit_exact"),
            "got: {err}"
        );
    }

    #[test]
    fn topic_wide_metric_passes_when_every_field_is_decodable() {
        let spec =
            ToleranceSpec::parse("topics:\n  /a:\n    metric:\n      kind: set_equal").unwrap();
        let r = SchemaResolver::new(&[("/a", Some(&["good", "also_good"]))]);
        spec.validate(&r)
            .expect("all-decodable topic-wide metric validates");
    }

    #[test]
    fn topic_wide_lossy_metric_on_a_u64_field_is_rejected_naming_it() {
        // The 64-bit-int composition: a topic-wide max_rel over a topic carrying
        // a u64 field is exit-4 naming that field.
        let spec = ToleranceSpec::parse(
            "topics:\n  /a:\n    metric:\n      kind: max_rel\n      threshold: 0.1",
        )
        .unwrap();
        let r = SchemaResolver::new(&[("/a", Some(&["good", "u64"]))]);
        let err = spec.validate(&r).unwrap_err();
        assert!(
            err.contains("topic-wide metric") && err.contains("'u64'") && err.contains("lossy"),
            "got: {err}"
        );
    }

    #[test]
    fn default_metric_validates_across_every_graph_topic_field() {
        // A non-bit_exact default_metric covers every field of every topic — an
        // opaque field on ANY graph topic is exit-4, scoped as default_metric.
        let spec =
            ToleranceSpec::parse("default_metric:\n  kind: max_abs\n  threshold: 0.1").unwrap();
        let r = SchemaResolver::new(&[
            ("/a", Some(&["good"])),
            ("/b", Some(&["also_good", "opaque"])),
        ]);
        let err = spec.validate(&r).unwrap_err();
        assert!(
            err.contains("default_metric") && err.contains("'opaque'") && err.contains("'/b'"),
            "got: {err}"
        );
    }

    #[test]
    fn field_override_shadows_topic_metric_in_coverage() {
        // A bit_exact per-field override on the opaque field SHADOWS the
        // non-bit_exact topic-wide metric there, so the topic validates (the
        // topic metric only covers the decodable sibling).
        let spec = ToleranceSpec::parse(
            "topics:\n  /a:\n    metric:\n      kind: max_abs\n      threshold: 0.1\n    fields:\n      opaque:\n        kind: bit_exact",
        )
        .unwrap();
        let r = SchemaResolver::new(&[("/a", Some(&["good", "opaque"]))]);
        spec.validate(&r)
            .expect("a bit_exact override shields the opaque field from the topic metric");
    }

    #[test]
    fn bit_exact_topic_metric_shadows_a_nonbit_default_no_coverage_error() {
        // A bit_exact topic-wide metric pins the whole topic byte-exact, shadowing
        // a non-bit_exact default — so the topic's opaque field is NOT gated.
        let spec = ToleranceSpec::parse(
            "default_metric:\n  kind: max_abs\n  threshold: 0.1\ntopics:\n  /a:\n    metric:\n      kind: bit_exact",
        )
        .unwrap();
        let r = SchemaResolver::new(&[("/a", Some(&["good", "opaque"]))]);
        spec.validate(&r)
            .expect("a bit_exact topic metric shields the topic from the non-bit default");
    }

    #[test]
    fn explicit_topic_metric_on_unavailable_schema_is_rejected() {
        // A user who nominates a topic-wide non-bit_exact metric on a topic whose
        // schema is unavailable gets a loud exit-4 (it cannot be decoded).
        let spec = ToleranceSpec::parse(
            "topics:\n  /ext:\n    metric:\n      kind: max_abs\n      threshold: 0.1",
        )
        .unwrap();
        let r = SchemaResolver::new(&[("/ext", None)]);
        let err = spec.validate(&r).unwrap_err();
        assert!(
            err.contains("/ext") && err.contains("not resolvable"),
            "got: {err}"
        );
    }

    #[test]
    fn default_metric_leaves_unavailable_schema_topics_byte_exact() {
        // A GLOBAL default_metric does NOT reject an unresolvable-schema topic —
        // it simply stays byte-exact there (stricter, never a false pass). A
        // decodable sibling topic still validates.
        let spec =
            ToleranceSpec::parse("default_metric:\n  kind: max_abs\n  threshold: 0.1").unwrap();
        let r = SchemaResolver::new(&[("/ext", None), ("/a", Some(&["good"]))]);
        spec.validate(&r)
            .expect("default_metric skips an unavailable-schema topic");
    }

    // ── Name resolution + suggestions ─────────────────────────────────────

    /// A resolver over a fixed topic + field set for suggestion tests.
    struct FixedResolver {
        topics: Vec<String>,
        fields: Vec<String>,
    }
    impl FieldResolver for FixedResolver {
        fn topics(&self) -> Vec<String> {
            self.topics.clone()
        }
        fn resolve_field(&self, _topic: &str, path: &str) -> Result<(), FieldResolveError> {
            // Only single-segment paths in these tests; resolve the whole path
            // as one field name.
            let seg = path.split('.').next().unwrap_or(path);
            if self.fields.iter().any(|f| f == seg) {
                Ok(())
            } else {
                Err(FieldResolveError {
                    segment: seg.to_string(),
                    candidates: self.fields.clone(),
                    schema_unavailable: false,
                })
            }
        }
    }

    fn fixed() -> FixedResolver {
        FixedResolver {
            topics: vec![
                "/perception/detections".to_string(),
                "/imu/data".to_string(),
            ],
            fields: vec!["bbox".to_string(), "score".to_string(), "label".to_string()],
        }
    }

    #[test]
    fn valid_topic_and_field_pass() {
        let spec = ToleranceSpec::parse(
            "topics:\n  /perception/detections:\n    fields:\n      bbox:\n        kind: set_equal",
        )
        .unwrap();
        spec.validate(&fixed()).expect("valid names pass");
    }

    #[test]
    fn typoed_topic_is_rejected_with_suggestion() {
        let spec = ToleranceSpec::parse("topics:\n  /perception/detectons: {}").unwrap();
        let err = spec.validate(&fixed()).unwrap_err();
        assert!(
            err.contains("/perception/detectons")
                && err.contains("did you mean")
                && err.contains("/perception/detections"),
            "got: {err}"
        );
    }

    #[test]
    fn typoed_field_is_rejected_with_suggestion() {
        let spec = ToleranceSpec::parse(
            "topics:\n  /imu/data:\n    fields:\n      scor:\n        kind: max_abs\n        threshold: 0.1",
        )
        .unwrap();
        let err = spec.validate(&fixed()).unwrap_err();
        assert!(
            err.contains("scor") && err.contains("did you mean") && err.contains("score"),
            "got: {err}"
        );
    }

    #[test]
    fn composed_topic_and_field_typo_surfaces_both_suggestions() {
        // The category-A load-bearing pin: topic '/perception/detectons' (typo of
        // '/perception/detections') AND field 'bbx' (typo of 'bbox') both wrong.
        // ONE exit-4 error must name BOTH corrections.
        let spec = ToleranceSpec::parse(
            "topics:\n  /perception/detectons:\n    fields:\n      bbx:\n        kind: set_equal",
        )
        .unwrap();
        let err = spec.validate(&fixed()).unwrap_err();
        assert!(
            err.contains("/perception/detections"),
            "topic suggestion missing: {err}"
        );
        assert!(err.contains("bbox"), "field suggestion missing: {err}");
        assert!(err.contains("bbx"), "offending field named: {err}");
    }

    #[test]
    fn edit_budget_scales_with_length() {
        // <= 2 for short names, ceil(len/3) for long ones.
        assert_eq!(edit_budget(3), 2);
        assert_eq!(edit_budget(6), 2);
        assert_eq!(edit_budget(9), 3);
        assert_eq!(edit_budget(30), 10);
    }

    #[test]
    fn levenshtein_hand_oracles() {
        assert_eq!(levenshtein("bbox", "bbx"), 1);
        assert_eq!(levenshtein("score", "scor"), 1);
        assert_eq!(levenshtein("kitten", "sitting"), 3);
        assert_eq!(levenshtein("", "abc"), 3);
        assert_eq!(levenshtein("abc", "abc"), 0);
    }

    #[test]
    fn suggestions_sorted_by_distance_then_name() {
        // Two candidates at distance 1 ("aa","ac") and one at distance 2 ("bb")
        // from "ab": distance-1 group first, name-sorted; then distance 2.
        let cands = vec!["ac".to_string(), "aa".to_string(), "bb".to_string()];
        let s = suggest("ab", &cands);
        assert_eq!(
            s,
            vec!["aa".to_string(), "ac".to_string(), "bb".to_string()]
        );
    }

    #[test]
    fn far_candidates_are_not_suggested() {
        let cands = vec!["completely_different".to_string()];
        assert!(suggest("xy", &cands).is_empty());
    }

    #[test]
    fn schema_unavailable_field_error_is_distinct() {
        struct NoSchema;
        impl FieldResolver for NoSchema {
            fn topics(&self) -> Vec<String> {
                vec!["/ext".to_string()]
            }
            fn resolve_field(&self, _topic: &str, _path: &str) -> Result<(), FieldResolveError> {
                Err(FieldResolveError {
                    segment: String::new(),
                    candidates: vec![],
                    schema_unavailable: true,
                })
            }
        }
        let spec = ToleranceSpec::parse(
            "topics:\n  /ext:\n    fields:\n      x:\n        kind: set_equal",
        )
        .unwrap();
        let err = spec.validate(&NoSchema).unwrap_err();
        assert!(err.contains("schema is not resolvable"), "got: {err}");
    }
}
