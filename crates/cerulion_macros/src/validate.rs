// SPDX-License-Identifier: AGPL-3.0-only
//! Validation for parsed `#[cerulion_node(...)]` attributes.
//!
//! Checks:
//! - Port names are valid identifiers (enforced by syn parsing)
//! - No duplicate port names across inputs and outputs
//! - Trigger policy inference rules (declarative mode only)
//!
//! There is no `type_name` validation — the node type is the
//! folder name (`nodes/<type>/`) and is enforced by the CLI /
//! graph runtime, not the macro.

use std::collections::HashSet;

use proc_macro2::Span;

use crate::parse::{FieldAttrs, NodeAttr};

/// Span used by validators when emitting attribute-level errors.
///
/// There is no node-level literal to anchor diagnostics on, so
/// attribute-level errors land at the call site.
fn attr_span(_attr: &NodeAttr) -> Span {
    Span::call_site()
}

/// Validate the parsed node attributes and field attributes.
///
/// Returns a list of errors (empty = valid).
pub fn validate(attr: &NodeAttr, field_attrs: &FieldAttrs) -> Vec<syn::Error> {
    let mut errors = Vec::new();

    validate_no_duplicate_ports(field_attrs, &mut errors);
    validate_input_attrs(field_attrs, &mut errors);
    validate_policy_bounds(attr, &mut errors);
    validate_port_deadlines(field_attrs, &mut errors);

    if field_attrs.is_declarative() {
        validate_trigger_inference(attr, field_attrs, &mut errors);
    }

    errors
}

/// Reject zero values for the time-based trigger-policy attrs.
///
/// `period_ms = 0` or `sync_window_ms = 0`
/// would compile to `Duration::from_millis(0)` and let the
/// scheduler spin-loop on a zero-duration period — a silent
/// runtime hang. Catch them at macro-expansion time so the user
/// sees a clear compile error instead.
fn validate_policy_bounds(attr: &NodeAttr, errors: &mut Vec<syn::Error>) {
    let nl = &attr.node_level;
    if matches!(nl.period_ms, Some(0)) {
        errors.push(syn::Error::new(
            attr_span(attr),
            "`period_ms` must be > 0 (zero-duration period would spin-loop the scheduler)",
        ));
    }
    if matches!(nl.sync_window_ms, Some(0)) {
        errors.push(syn::Error::new(
            attr_span(attr),
            "`sync_window_ms` must be > 0 (zero-window sync rejects every fan-in)",
        ));
    }
    // tick_within_ms must also be > 0 — a
    // zero-duration tick deadline would immediately miss on every fire.
    if matches!(nl.tick_within_ms, Some(0)) {
        errors.push(syn::Error::new(
            attr_span(attr),
            "`tick_within_ms` must be > 0 (zero-duration tick deadline misses every fire)",
        ));
    }
    // Throttle_ms (producer rate cap) must be > 0 — a zero cap
    // would defer every fire forever.
    if matches!(nl.throttle_ms, Some(0)) {
        errors.push(syn::Error::new(
            attr_span(attr),
            "`throttle_ms` must be > 0 (a zero-ms rate cap would defer every fire)",
        ));
    }
    // Throttle_ms is mutually exclusive with period_ms. `period`
    // ALREADY pins the fire rate, so a throttle cap is either redundant
    // (cap >= period) or silently fights it (cap < period). Reject at
    // compile time so the conflict surfaces before the graph runs (this is
    // the macro-level "compile-time prevention over runtime guard" rule).
    if nl.throttle_ms.is_some() && nl.period_ms.is_some() {
        errors.push(syn::Error::new(
            attr_span(attr),
            "`throttle_ms` cannot be combined with `period_ms`: period already pins the fire \
             rate, so a rate cap is redundant or conflicting. Use `throttle_ms` with a \
             data/deadline/sync/external trigger to cap a bursty producer, or just lower \
             `period_ms`.",
        ));
    }
}

/// Per-port deadline values must be > 0.
fn validate_port_deadlines(field_attrs: &FieldAttrs, errors: &mut Vec<syn::Error>) {
    for input in &field_attrs.inputs {
        if matches!(input.expect_within_ms, Some(0)) {
            errors.push(syn::Error::new(
                input.field_name.span(),
                "`#[input(expect_within_ms = N)]` requires N > 0 — zero would miss on every step",
            ));
        }
    }
    for output in &field_attrs.outputs {
        if matches!(output.promise_within_ms, Some(0)) {
            errors.push(syn::Error::new(
                output.field_name.span(),
                "`#[output(promise_within_ms = N)]` requires N > 0 — zero would miss on every step",
            ));
        }
    }
}

fn validate_no_duplicate_ports(field_attrs: &FieldAttrs, errors: &mut Vec<syn::Error>) {
    let mut seen = HashSet::new();

    for input in &field_attrs.inputs {
        let name = input.field_name.to_string();
        if !seen.insert(name.clone()) {
            errors.push(syn::Error::new(
                input.field_name.span(),
                format!("duplicate port name `{name}`"),
            ));
        }
    }
    for output in &field_attrs.outputs {
        let name = output.field_name.to_string();
        if !seen.insert(name.clone()) {
            errors.push(syn::Error::new(
                output.field_name.span(),
                format!("duplicate port name `{name}`"),
            ));
        }
    }
}

/// Mirror of `cerulion_core::graph::topology::MAX_CONSUMER_DEPTH`.
/// Proc-macro crates cannot depend on `cerulion_core` (the dependency
/// points the other way), so the value is duplicated here. The
/// cross-pin lives in `cerulion_core`: the
/// `max_consumer_depth_pinned_at_64` oracle test plus the
/// `depth_above_max` trybuild snapshot both break if either side
/// drifts. The runtime gate (`GraphTopology::validate`) remains the
/// source of truth — it also covers non-macro consumers and cdylibs
/// built against older macro versions; this check is early DX only.
const MAX_CONSUMER_DEPTH: usize = 64;

fn validate_input_attrs(field_attrs: &FieldAttrs, errors: &mut Vec<syn::Error>) {
    for input in &field_attrs.inputs {
        if let Some(depth) = input.depth {
            if depth == 0 {
                errors.push(syn::Error::new(
                    input.field_name.span(),
                    "depth must be >= 1",
                ));
            }
            if depth > MAX_CONSUMER_DEPTH {
                errors.push(syn::Error::new(
                    input.field_name.span(),
                    format!(
                        "depth must be <= {MAX_CONSUMER_DEPTH} (MAX_CONSUMER_DEPTH): every \
                         unit of depth commits a full max_slice_len-sized SHM slot — use a \
                         backpressure policy (drop_oldest / sample(N) / block) instead of a \
                         deeper queue"
                    ),
                ));
            }
        }
    }
}

/// Validate trigger policy inference for declarative mode.
///
/// Inference rules:
/// - 0 trigger inputs + `period_ms`: Period
/// - 0 trigger inputs + `external`: External
/// - 1 trigger input (no node attr needed): DataTrigger
/// - 2+ trigger inputs + `sync_window_ms`: bounded Sync
/// - 2+ trigger inputs + `unbounded_sync`: unbounded Sync
/// - 2+ trigger inputs without `sync_window_ms`/`unbounded_sync`: compile error
/// - trigger inputs + period/external: compile error
/// - `sync_window_ms` + `unbounded_sync` together: compile error (mutually exclusive)
/// - 0 trigger inputs + no node attr: compile error
/// - 0 trigger inputs + `sync_window_ms` or `unbounded_sync`: compile error
/// - 1 trigger input + `sync_window_ms` or `unbounded_sync`: ACCEPTED here (the
///   `1 =>` arm below pushes no error). The macro still emits the Sync policy,
///   and the graph build degrades the node to a DataTrigger on that input with
///   a `warn` naming the node (`cerulion_core::graph::runtime`). Sync
///   semantics need 2 or more trigger inputs to mean anything.
fn validate_trigger_inference(
    attr: &NodeAttr,
    field_attrs: &FieldAttrs,
    errors: &mut Vec<syn::Error>,
) {
    let trigger_count = field_attrs.inputs.iter().filter(|i| i.trigger).count();

    let nl = &attr.node_level;
    let has_period = nl.period_ms.is_some();
    let has_sync = nl.sync_window_ms.is_some();
    let has_unbounded_sync = nl.unbounded_sync;
    let has_external = nl.external;
    let has_time_attr = has_period || has_external;

    // Cannot have trigger inputs with period/external
    if trigger_count > 0 && has_time_attr {
        errors.push(syn::Error::new(
            attr_span(attr),
            "cannot combine `#[input(trigger)]` with `period_ms` or `external`; \
             trigger inputs define data-driven policy, which conflicts with time-driven policy",
        ));
        return;
    }

    // `sync_window_ms` and `unbounded_sync` are mutually exclusive — pick one
    if has_sync && has_unbounded_sync {
        errors.push(syn::Error::new(
            attr_span(attr),
            "cannot specify both `sync_window_ms` and `unbounded_sync` — they're alternative \
             sync semantics; pick one (`sync_window_ms = N` for bounded sync with a timing window, \
             or `unbounded_sync` for loose AND with no timing bound)",
        ));
        return;
    }

    // Multiple conflicting node-level time attrs (count sync attrs as one slot
    // — either bounded sync_window_ms or unbounded_sync, never both)
    let any_sync_attr = (has_sync || has_unbounded_sync) as usize;
    let time_attr_count = has_period as usize + has_external as usize + any_sync_attr;
    if time_attr_count > 1 && trigger_count == 0 {
        errors.push(syn::Error::new(
            attr_span(attr),
            "only one of `period_ms`, `sync_window_ms`, `unbounded_sync`, \
             or `external` can be specified",
        ));
        return;
    }

    match trigger_count {
        0 => {
            // sync_window_ms or unbounded_sync alone (no triggers) is invalid
            // — both require ≥2 trigger inputs to be meaningful.
            if has_sync || has_unbounded_sync {
                let attr_name = if has_sync {
                    "`sync_window_ms`"
                } else {
                    "`unbounded_sync`"
                };
                errors.push(syn::Error::new(
                    attr_span(attr),
                    format!(
                        "{attr_name} requires ≥2 `#[input(trigger)]` fields — add them, or pick \
                         a different policy (`period_ms` / `external`)"
                    ),
                ));
                return;
            }
            // Must have a node-level timing attribute
            if !has_period && !has_external {
                errors.push(syn::Error::new(
                    attr_span(attr),
                    "no trigger policy: add `#[input(trigger)]` to a field, or specify \
                     `period_ms` or `external` on the node",
                ));
            }
        }
        1 => {
            // Single trigger input → DataTrigger, no additional attr needed.
            // sync_window_ms / unbounded_sync are silently ignored with a
            // single trigger — emit a `tracing::warn!` at node-init time so
            // the surprise is visible (codegen-side; see
            // `cerulion_macros::codegen::gen_lifecycle_init`). Not a compile
            // error because removing the sync attr is the user's call.
        }
        _ => {
            // 2+ trigger inputs require one of: sync_window_ms, unbounded_sync
            if !has_sync && !has_unbounded_sync {
                let trigger_names: Vec<String> = field_attrs
                    .inputs
                    .iter()
                    .filter(|i| i.trigger)
                    .map(|i| i.field_name.to_string())
                    .collect();
                errors.push(syn::Error::new(
                    attr_span(attr),
                    format!(
                        "multiple trigger inputs ({}) require one of: `#[cerulion_node(sync_window_ms = N)]` \
                         (bounded — one fire per N-ms window with the latest of each trigger) OR \
                         `#[cerulion_node(unbounded_sync)]` (no time bound — one fire when each trigger \
                         has ≥1 unconsumed message; not recommended for control loops)",
                        trigger_names.join(", "),
                    ),
                ));
            }
        }
    }
}
