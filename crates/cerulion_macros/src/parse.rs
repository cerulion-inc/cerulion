// SPDX-License-Identifier: AGPL-3.0-only
//! Attribute parsing for `#[cerulion_node(...)]`.
//!
//! The node type is inferred from the surrounding folder name —
//! never a struct property. Ports are declared via field-level
//! `#[input]` / `#[output]` attributes on struct fields, paired with
//! `#[cerulion_node_impl]` on the impl block:
//!
//! ```text
//! #[cerulion_node]
//! struct Controller {
//!     #[input(trigger, depth = 1)]
//!     scan: LaserScan,
//!     #[output]
//!     cmd_vel: Twist,
//! }
//! ```
//!
//! (A data-triggered node: the one `#[input(trigger)]` field IS the policy,
//! so the macro takes no policy argument. `#[input(trigger)]` together with
//! `period_ms` or `external` is rejected by `validate.rs`.)
//!
//! What `#[cerulion_node(...)]` accepts, all eight keys (see the `match` in
//! `impl Parse for NodeAttr` below, which is the source of truth):
//!
//! - `period_ms = N`: fire every `N` ms.
//! - `sync_window_ms = N`: fire once per complete set of `#[input(trigger)]`
//!   messages whose timestamps lie within `N` ms of each other.
//! - `unbounded_sync`: fire once per complete set, with no timing bound.
//! - `external`: the node fires itself from its `external_source()`.
//! - `tick_within_ms = N`: tick-duration deadline (counted and warned, never
//!   interrupted).
//! - `throttle_ms = N`: producer rate cap (rejected with `period_ms`).
//! - `allow_non_deterministic`, `uses_live_io`: determinism opt-outs.
//!
//! A node needs exactly one source of trigger policy: one of the first four
//! keys, or exactly one `#[input(trigger)]` field and none of them.
//!
//! Ports are NOT declared as macro arguments: `type_name`, `inputs(...)`
//! and `outputs(...)` are rejected at parse time below, each pointing at the
//! attribute that does the job instead (see `PORT_ARGS_NOT_ACCEPTED_HINT`).

use syn::parse::{Parse, ParseStream};
use syn::{DeriveInput, Ident, LitInt, Token};

/// Hint emitted when ports are passed as macro arguments instead of field
/// attributes. Centralised so every such case tells one story.
const PORT_ARGS_NOT_ACCEPTED_HINT: &str =
    "`inputs(...)` / `outputs(...)` macro args are not accepted. Declare ports via \
     field-level `#[input]` / `#[output]` attributes and pair the struct with \
     `#[cerulion_node_impl]`.";

// ---------------------------------------------------------------------------
// Declarative field-level attributes
// ---------------------------------------------------------------------------

/// Backpressure policy parsed from `#[input(backpressure = drop_oldest)]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParsedBackpressurePolicy {
    DropOldest,
    Block,
    Sample(u64),
}

/// Parsed `#[input(...)]` field attribute.
///
/// The never-executed `fifo` / `lifo` / `max_age_ms` / `filter` attrs
/// were removed (they were parsed but consumed by nothing; separate issues
/// track post-launch reimplementation). The surviving surface is
/// `trigger`, `depth`, `backpressure`, `expect_within_ms`.
#[derive(Debug, Clone)]
pub struct FieldInputAttr {
    pub field_name: Ident,
    pub field_type: syn::Type,
    pub trigger: bool,
    pub depth: Option<usize>,
    pub backpressure: Option<ParsedBackpressurePolicy>,
    /// Per-input expected interval (ms). When
    /// set, the scheduler tracks the last-data time on this input;
    /// if `N` ms elapse without a new message arriving, the node's
    /// `expect_within_missed_count` increments. Independent of
    /// the firing policy — this is a QoS observation, not a trigger.
    pub expect_within_ms: Option<u64>,
}

/// Parsed `#[output]` field attribute.
///
/// User syntax (the variable-field lists are gone):
/// ```text
/// #[output]                          // the canonical form
/// #[output(promise_within_ms = 10)]  // publish-side QoS deadline
/// ```
///
/// `#[output]` takes no field list: codegen emits a uniform
/// `__cer_assign_<field>` / `__cer_fill_from_<field>` shim per schema field,
/// so the `#[cerulion_node_impl]` rewriter is schema-blind and needs no
/// per-port field metadata. A field list is rejected below, pointing at the
/// assignment form instead.
#[derive(Debug, Clone)]
pub struct FieldOutputAttr {
    pub field_name: Ident,
    pub field_type: syn::Type,
    /// Per-output committed interval (ms). When
    /// set, the publisher tracks the last-publish time on this output;
    /// if `N` ms elapse without a publish, the node's
    /// `promise_within_missed_count` increments. Independent of the
    /// firing policy — this is a QoS commitment to downstream
    /// subscribers.
    pub promise_within_ms: Option<u64>,
}

/// Node-level attributes that may accompany declarative mode.
#[derive(Debug, Clone, Default)]
pub struct NodeLevelAttrs {
    pub period_ms: Option<u64>,
    pub sync_window_ms: Option<u64>,
    /// Unbounded-sync opt-in: fires once per complete set, as soon as every
    /// `#[input(trigger)]` port has an unconsumed message, with no timing
    /// bound. Mutually exclusive with `sync_window_ms` /
    /// `period_ms` / `external`. Meant for 2 or more trigger inputs: zero is
    /// a compile error, and exactly ONE compiles but is degraded to a data
    /// trigger on that input, with a `warn`, when the graph is built.
    pub unbounded_sync: bool,
    pub external: bool,
    /// Per-node tick-execution deadline (ms).
    /// Wraps the tick callback with timing; if elapsed > N ms, the
    /// node's `tick_within_missed_count` increments. Orthogonal
    /// to the firing policy — can combine with any of
    /// `period_ms` / `sync_window_ms` / `unbounded_sync` /
    /// `external` / `#[input(trigger)]`.
    pub tick_within_ms: Option<u64>,
    /// Node-level producer rate cap (ms). The scheduler defers the
    /// node's tick when `now - last_fire < N ms` — a "fire no faster than
    /// once per N ms" gate. STACKS with every trigger policy EXCEPT
    /// `period_ms` (period already pins the rate, so a cap is
    /// redundant/conflicting — rejected at compile time). Distinct from the
    /// input-level `sample(N)` (subscriber decimation); this is a
    /// producer-side execution policy, like `tick_within_ms`.
    pub throttle_ms: Option<u64>,
    /// Blanket determinism opt-out. When set, the
    /// `#[cerulion_node_impl]` determinism lint suppresses EVERY banned
    /// symbol (deny AND warn) for this node. The node still compiles, but
    /// the (deferred) CLI half surfaces a graph-load warning naming the
    /// node + this relaxation. Flat flag on `#[cerulion_node(...)]`:
    /// `#[cerulion_node(allow_non_deterministic)]`.
    pub allow_non_deterministic: bool,
    /// IO-class determinism opt-out. When set, the lint suppresses
    /// only the IO-class denies (e.g. `fs::read_dir`) — time / thread /
    /// RNG symbols still fire. The node is marked as performing live IO so
    /// the CLI can flag it at graph load. Flat flag:
    /// `#[cerulion_node(uses_live_io)]`.
    pub uses_live_io: bool,
}

// ---------------------------------------------------------------------------
// Macro-level attribute (#[cerulion_node(...)])
// ---------------------------------------------------------------------------

/// Parsed attributes from `#[cerulion_node(...)]`.
///
/// Carries the node-level attributes: the trigger policy (`period_ms`,
/// `sync_window_ms`, `unbounded_sync`, `external`), the execution limits
/// (`tick_within_ms`, `throttle_ms`) and the determinism opt-outs
/// (`allow_non_deterministic`, `uses_live_io`). Port declarations live on struct fields via
/// `#[input]` / `#[output]` attributes; the node type is the folder
/// name (`nodes/<type>/`), resolved at graph-load time, never a
/// struct property.
pub struct NodeAttr {
    /// Node-level attributes for declarative mode (period_ms, sync_window_ms, etc).
    pub node_level: NodeLevelAttrs,
}

impl Parse for NodeAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut node_level = NodeLevelAttrs::default();

        while !input.is_empty() {
            let key: Ident = input.parse()?;
            match key.to_string().as_str() {
                "type_name" => {
                    return Err(syn::Error::new(
                        key.span(),
                        "`type_name` is not accepted — the node type is the folder name \
                         (`nodes/<type>/`) and is resolved at graph-load time. Remove the \
                         `type_name = \"...\"` argument.",
                    ));
                }
                "inputs" | "outputs" => {
                    return Err(syn::Error::new(key.span(), PORT_ARGS_NOT_ACCEPTED_HINT));
                }
                "period_ms" => {
                    input.parse::<Token![=]>()?;
                    let lit: LitInt = input.parse()?;
                    node_level.period_ms = Some(lit.base10_parse()?);
                }
                "sync_window_ms" => {
                    input.parse::<Token![=]>()?;
                    let lit: LitInt = input.parse()?;
                    node_level.sync_window_ms = Some(lit.base10_parse()?);
                }
                "unbounded_sync" => {
                    node_level.unbounded_sync = true;
                }
                "external" => {
                    node_level.external = true;
                }
                "tick_within_ms" => {
                    input.parse::<Token![=]>()?;
                    let lit: LitInt = input.parse()?;
                    node_level.tick_within_ms = Some(lit.base10_parse()?);
                }
                "throttle_ms" => {
                    input.parse::<Token![=]>()?;
                    let lit: LitInt = input.parse()?;
                    node_level.throttle_ms = Some(lit.base10_parse()?);
                }
                // Determinism opt-outs — flat boolean flags.
                "allow_non_deterministic" => {
                    node_level.allow_non_deterministic = true;
                }
                "uses_live_io" => {
                    node_level.uses_live_io = true;
                }
                other => {
                    return Err(syn::Error::new(
                        key.span(),
                        format!(
                            "unknown attribute `{other}`, expected `period_ms`, \
                             `sync_window_ms`, `unbounded_sync`, `external`, `tick_within_ms`, \
                             `throttle_ms`, `allow_non_deterministic`, or `uses_live_io`"
                        ),
                    ));
                }
            }
            // Consume trailing comma if present
            if !input.is_empty() {
                input.parse::<Token![,]>()?;
            }
        }

        Ok(NodeAttr { node_level })
    }
}

// ---------------------------------------------------------------------------
// Field attribute extraction from DeriveInput
// ---------------------------------------------------------------------------

/// Result of extracting field-level `#[input]`/`#[output]` attributes from a struct.
#[derive(Debug, Default)]
pub struct FieldAttrs {
    pub inputs: Vec<FieldInputAttr>,
    pub outputs: Vec<FieldOutputAttr>,
}

impl FieldAttrs {
    /// Returns true if any field-level attributes were found (declarative mode).
    pub fn is_declarative(&self) -> bool {
        !self.inputs.is_empty() || !self.outputs.is_empty()
    }

    /// Returns the declared input port names (field name = port name).
    pub fn input_names(&self) -> Vec<String> {
        self.inputs
            .iter()
            .map(|i| i.field_name.to_string())
            .collect()
    }

    /// Returns the declared output port names (field name = port name).
    pub fn output_names(&self) -> Vec<String> {
        self.outputs
            .iter()
            .map(|o| o.field_name.to_string())
            .collect()
    }
}

/// Extract `#[input(...)]` and `#[output]` attributes from struct fields.
///
/// Returns `FieldAttrs` with parsed input/output declarations.
/// Unknown attributes are ignored (they may be other derive attributes).
pub fn extract_field_attrs(input: &DeriveInput) -> syn::Result<FieldAttrs> {
    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            syn::Fields::Named(named) => &named.named,
            _ => return Ok(FieldAttrs::default()),
        },
        _ => return Ok(FieldAttrs::default()),
    };

    let mut result = FieldAttrs::default();

    for field in fields {
        let field_name = match &field.ident {
            Some(name) => name.clone(),
            None => continue, // skip unnamed fields (unreachable for Named fields)
        };
        let field_type = field.ty.clone();

        for attr in &field.attrs {
            if attr.path().is_ident("input") {
                let input_attr = parse_input_attr(attr, &field_name, &field_type)?;
                result.inputs.push(input_attr);
            } else if attr.path().is_ident("output") {
                let output_attr = parse_output_attr(attr, &field_name, &field_type)?;
                result.outputs.push(output_attr);
            }
        }
    }

    Ok(result)
}

/// Parse the contents of an `#[input(...)]` attribute.
///
/// Supported syntax:
/// - `#[input]` — bare, all defaults
/// - `#[input(trigger)]`
/// - `#[input(trigger, depth = 1)]`
/// - `#[input(depth = 100, backpressure = drop_oldest)]`
/// - `#[input(expect_within_ms = 200)]`
///
/// Anything else routes to the unknown-attribute error, which names the
/// offending ident and lists the accepted set.
fn parse_input_attr(
    attr: &syn::Attribute,
    field_name: &Ident,
    field_type: &syn::Type,
) -> syn::Result<FieldInputAttr> {
    let mut result = FieldInputAttr {
        field_name: field_name.clone(),
        field_type: field_type.clone(),
        trigger: false,
        depth: None,
        backpressure: None,
        expect_within_ms: None,
    };

    // Handle bare `#[input]` with no parentheses
    let meta = &attr.meta;
    let list = match meta {
        syn::Meta::Path(_) => return Ok(result),
        syn::Meta::List(list) => list,
        syn::Meta::NameValue(nv) => {
            return Err(syn::Error::new_spanned(
                nv,
                "expected `#[input]` or `#[input(...)]`",
            ));
        }
    };

    list.parse_nested_meta(|meta| {
        let ident = meta
            .path
            .get_ident()
            .ok_or_else(|| syn::Error::new_spanned(&meta.path, "expected identifier"))?;

        match ident.to_string().as_str() {
            "trigger" => {
                result.trigger = true;
            }
            "depth" => {
                let value = meta.value()?;
                let lit: LitInt = value.parse()?;
                result.depth = Some(lit.base10_parse()?);
            }
            "backpressure" => {
                let value = meta.value()?;
                let policy_ident: Ident = value.parse()?;
                let policy = match policy_ident.to_string().as_str() {
                    "drop_oldest" => ParsedBackpressurePolicy::DropOldest,
                    "block" => ParsedBackpressurePolicy::Block,
                    "sample" => {
                        let content;
                        syn::parenthesized!(content in value);
                        let lit: LitInt = content.parse()?;
                        ParsedBackpressurePolicy::Sample(lit.base10_parse()?)
                    }
                    other => {
                        return Err(syn::Error::new(
                            policy_ident.span(),
                            format!(
                                "unknown backpressure policy `{other}`, expected \
                                 `drop_oldest`, `block`, or `sample(ms)`"
                            ),
                        ));
                    }
                };
                result.backpressure = Some(policy);
            }
            "expect_within_ms" => {
                let value = meta.value()?;
                let lit: LitInt = value.parse()?;
                result.expect_within_ms = Some(lit.base10_parse()?);
            }
            other => {
                // Bare-ident and `name = value` attrs both land here: the
                // match keys on the ident BEFORE any `= value` is consumed,
                // so every unknown shape gets the same message.
                return Err(syn::Error::new(
                    ident.span(),
                    format!(
                        "unknown input attribute `{other}`. Supported: `trigger`, `depth`, \
                         `backpressure`, `expect_within_ms`."
                    ),
                ));
            }
        }
        Ok(())
    })?;

    Ok(result)
}

/// Parse the contents of an `#[output(...)]` attribute.
///
/// Supported syntax:
/// - `#[output]` — bare; the canonical form for every output port.
/// - `#[output(promise_within_ms = N)]` — publish-side QoS deadline
///   (the `promise_within_ms` field on `FieldOutputAttr`).
///
/// Anything else is rejected, naming the offending ident and the accepted
/// forms. In particular `#[output]` takes NO field list — neither a bare
/// list (`#[output(data, encoding)]`) nor a parenthesized item
/// (`#[output(complex(header))]`) — because codegen emits a uniform
/// `__cer_assign_<field>` shim per schema field, so the rewriter is
/// schema-blind and per-port field metadata has no consumer.
fn parse_output_attr(
    attr: &syn::Attribute,
    field_name: &Ident,
    field_type: &syn::Type,
) -> syn::Result<FieldOutputAttr> {
    let mut result = FieldOutputAttr {
        field_name: field_name.clone(),
        field_type: field_type.clone(),
        promise_within_ms: None,
    };

    // Bare `#[output]` is the canonical form.
    let meta = &attr.meta;
    let list = match meta {
        syn::Meta::Path(_) => return Ok(result),
        syn::Meta::List(list) => list,
        syn::Meta::NameValue(nv) => {
            return Err(syn::Error::new_spanned(
                nv,
                "expected `#[output]` or `#[output(...)]`",
            ));
        }
    };

    // Parse the inner tokens of `#[output(...)]`. The only accepted item is
    // `promise_within_ms = N`; a bare ident, a `name = value` other than
    // that one, and a parenthesized item each get their own diagnostic
    // naming the offending ident and the accepted forms.
    use syn::parse::{Parse, ParseStream};
    use syn::{punctuated::Punctuated, Token};

    struct OutputItem(u64);

    impl Parse for OutputItem {
        fn parse(input: ParseStream) -> syn::Result<Self> {
            let ident: Ident = input.parse()?;
            if input.peek(syn::token::Paren) {
                return Err(syn::Error::new(
                    ident.span(),
                    format!(
                        "unknown output attribute `{ident}(...)`; #[output] accepts only the \
                         bare form or `promise_within_ms = N`"
                    ),
                ));
            }
            if input.peek(Token![=]) {
                // `name = value` form — only `promise_within_ms = N` is accepted.
                if ident != "promise_within_ms" {
                    return Err(syn::Error::new(
                        ident.span(),
                        format!(
                            "unknown output attribute `{ident} = ...`, expected `promise_within_ms = N`"
                        ),
                    ));
                }
                input.parse::<Token![=]>()?;
                let lit: LitInt = input.parse()?;
                return Ok(OutputItem(lit.base10_parse()?));
            }
            // Bare identifier — most likely an attempt to list the message's
            // fields, which #[output] does not take.
            Err(syn::Error::new(
                ident.span(),
                format!(
                    "unknown output attribute `{ident}`; #[output] takes no field list. Write \
                     self.<port>.<field> = expr in the node body instead — the macro resolves \
                     fixed vs variable fields at compile time. #[output] accepts only the bare \
                     form or `promise_within_ms = N`."
                ),
            ))
        }
    }

    let items: Punctuated<OutputItem, Token![,]> =
        list.parse_args_with(Punctuated::<OutputItem, Token![,]>::parse_terminated)?;
    for OutputItem(ms) in items {
        if result.promise_within_ms.is_some() {
            return Err(syn::Error::new_spanned(
                attr,
                "duplicate `promise_within_ms` in #[output]",
            ));
        }
        result.promise_within_ms = Some(ms);
    }

    Ok(result)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    #[test]
    fn test_legacy_inputs_outputs_rejected() {
        // legacy `inputs(...)`/`outputs(...)` are gone.
        let result: syn::Result<NodeAttr> = syn::parse2(quote::quote!(inputs(image)));
        let err = result.err().expect("inputs(...) must be rejected");
        assert!(err.to_string().contains("not accepted"), "got: {err}");

        let result: syn::Result<NodeAttr> = syn::parse2(quote::quote!(outputs(image)));
        let err = result.err().expect("outputs(...) must be rejected");
        assert!(err.to_string().contains("not accepted"), "got: {err}");
    }

    #[test]
    fn test_no_attrs_parses() {
        // Bare `#[cerulion_node]` with no args is the canonical form —
        // the node type is the folder name, not a struct property.
        let _attr: NodeAttr = syn::parse_quote!();
    }

    #[test]
    fn test_type_name_is_rejected() {
        // `type_name = "..."` is no longer accepted.
        let result: syn::Result<NodeAttr> = syn::parse2(quote::quote!(type_name = "camera"));
        let err = result.err().expect("type_name = \"...\" must be rejected");
        assert!(err.to_string().contains("not accepted"), "got: {err}");
    }

    #[test]
    fn test_node_level_period_ms() {
        let attr: NodeAttr = syn::parse_quote!(period_ms = 1000);
        assert_eq!(attr.node_level.period_ms, Some(1000));
    }

    #[test]
    fn test_node_level_external() {
        let attr: NodeAttr = syn::parse_quote!(external);
        assert!(attr.node_level.external);
    }

    #[test]
    fn test_node_level_sync_window() {
        let attr: NodeAttr = syn::parse_quote!(sync_window_ms = 50);
        assert_eq!(attr.node_level.sync_window_ms, Some(50));
    }

    #[test]
    fn test_allow_non_deterministic_flag() {
        let attr: NodeAttr = syn::parse_quote!(allow_non_deterministic);
        assert!(attr.node_level.allow_non_deterministic);
        assert!(!attr.node_level.uses_live_io);
    }

    #[test]
    fn test_uses_live_io_flag() {
        let attr: NodeAttr = syn::parse_quote!(uses_live_io);
        assert!(attr.node_level.uses_live_io);
        assert!(!attr.node_level.allow_non_deterministic);
    }

    #[test]
    fn test_determinism_flag_combines_with_period() {
        // The flat determinism flags stack with a trigger-policy hint.
        let attr: NodeAttr = syn::parse_quote!(period_ms = 10, uses_live_io);
        assert_eq!(attr.node_level.period_ms, Some(10));
        assert!(attr.node_level.uses_live_io);
    }

    #[test]
    fn test_unknown_attr_error_lists_determinism_flags() {
        let result: syn::Result<NodeAttr> = syn::parse2(quote::quote!(bogus_flag));
        let err = result.err().expect("unknown attr must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("allow_non_deterministic") && msg.contains("uses_live_io"),
            "unknown-attr error should name the determinism flags; got: {msg}"
        );
    }

    #[test]
    fn test_extract_bare_input() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[input]
                scan: LaserScan,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert!(attrs.is_declarative());
        assert_eq!(attrs.inputs.len(), 1);
        assert_eq!(attrs.inputs[0].field_name.to_string(), "scan");
        assert!(!attrs.inputs[0].trigger);
        assert!(attrs.inputs[0].depth.is_none());
    }

    #[test]
    fn test_extract_input_with_trigger_depth() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[input(trigger, depth = 1)]
                scan: LaserScan,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert_eq!(attrs.inputs.len(), 1);
        let inp = &attrs.inputs[0];
        assert!(inp.trigger);
        assert_eq!(inp.depth, Some(1));
    }

    #[test]
    fn test_extract_input_with_backpressure_and_depth() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[input(depth = 100, backpressure = block)]
                data: SensorData,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        let inp = &attrs.inputs[0];
        assert_eq!(inp.depth, Some(100));
        assert_eq!(inp.backpressure, Some(ParsedBackpressurePolicy::Block));
    }

    // ---- every unrecognized #[input] attr routes to the unknown-attr error ----

    /// The tail every unknown-`#[input]`-attr error must render, after the
    /// leading "unknown input attribute `<x>`. " prefix (the offending ident
    /// varies per test). Anchored on the accepted-set remedy, which is the
    /// only thing this diagnostic owes the reader.
    const UNKNOWN_INPUT_ATTR_TAIL: &str =
        "Supported: `trigger`, `depth`, `backpressure`, `expect_within_ms`.";

    fn assert_unknown_attr_error(input: DeriveInput, offending: &str) {
        let err = extract_field_attrs(&input).expect_err("unknown input attr must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains(&format!("unknown input attribute `{offending}`")),
            "error must name the offending ident `{offending}`; got: {msg}"
        );
        assert_eq!(
            msg,
            format!("unknown input attribute `{offending}`. {UNKNOWN_INPUT_ATTR_TAIL}"),
            "error must name the offending attr and list the accepted set, and nothing else"
        );
    }

    #[test]
    fn unknown_bare_ident_input_attr_is_rejected_and_lists_the_accepted_set() {
        // Bare-ident form, alongside attrs that ARE accepted.
        assert_unknown_attr_error(
            parse_quote! {
                struct MyNode {
                    #[input(trigger, newest_first, depth = 1)]
                    scan: LaserScan,
                }
            },
            "newest_first",
        );
    }

    #[test]
    fn unknown_bare_ident_input_attr_is_rejected_when_it_is_the_only_attr() {
        assert_unknown_attr_error(
            parse_quote! {
                struct MyNode {
                    #[input(oldest_first)]
                    data: SensorData,
                }
            },
            "oldest_first",
        );
    }

    #[test]
    fn unknown_name_value_input_attr_routes_to_the_same_error_as_a_bare_ident() {
        // Name-value form: the match keys on the ident BEFORE `= 200` is
        // consumed, so it routes to the same unknown-attr arm.
        assert_unknown_attr_error(
            parse_quote! {
                struct MyNode {
                    #[input(max_age_ms = 200)]
                    data: SensorData,
                }
            },
            "max_age_ms",
        );
    }

    #[test]
    fn unknown_name_value_input_attr_with_a_string_value_is_rejected() {
        assert_unknown_attr_error(
            parse_quote! {
                struct MyNode {
                    #[input(trigger, filter = "should_process")]
                    scan: LaserScan,
                }
            },
            "filter",
        );
    }

    #[test]
    fn test_extract_output() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output]
                cmd_vel: Twist,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert!(attrs.is_declarative());
        assert_eq!(attrs.outputs.len(), 1);
        assert_eq!(attrs.outputs[0].field_name.to_string(), "cmd_vel");
    }

    // ---- removed #[output(...)] forms: the bare field list gets a migration
    // ---- error; parenthesized items take the generic unknown-attribute rejection

    #[test]
    fn output_field_list_is_rejected_and_points_at_the_assignment_form() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(data, encoding)]
                image: Image,
            }
        };
        let err =
            extract_field_attrs(&input).expect_err("#[output(field, ...)] lists must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute `data`") && msg.contains("takes no field list"),
            "field-list error must name the offending ident and say #[output] takes no \
             field list; got: {msg}"
        );
        assert!(
            msg.contains("self.<port>.<field> = expr") && msg.contains("promise_within_ms = N"),
            "field-list error must point at the assignment form + the accepted arg; got: {msg}"
        );
    }

    #[test]
    fn a_single_bare_ident_output_attr_is_rejected_the_same_way_as_a_list() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(data)]
                out: RosString,
            }
        };
        let err = extract_field_attrs(&input).expect_err("#[output(data)] must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute `data`") && msg.contains("takes no field list"),
            "single bare ident must render the same diagnostic as a list; got: {msg}"
        );
    }

    #[test]
    fn an_unknown_parenthesized_output_attr_is_rejected_and_lists_the_accepted_forms() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(complex(header))]
                image: Image,
            }
        };
        let err =
            extract_field_attrs(&input).expect_err("#[output(complex(...))] must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute `complex(...)`"),
            "parenthesized form must name the offending ident; got: {msg}"
        );
        assert!(
            msg.contains("bare form") && msg.contains("promise_within_ms = N"),
            "parenthesized form must list the accepted forms; got: {msg}"
        );
    }

    #[test]
    fn a_mixed_unknown_output_list_reports_the_leftmost_offender() {
        // The parse is left-to-right, so the bare ident `data` errors first
        // and the later parenthesized item is never reached.
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(data, encoding, complex(header))]
                image: Image,
            }
        };
        let err = extract_field_attrs(&input).expect_err("unknown output forms must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute `data`"),
            "the LEFTMOST offending item must be the one reported; got: {msg}"
        );
    }

    #[test]
    fn test_output_promise_within_ms_still_accepted() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(promise_within_ms = 10)]
                cmd: Twist,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert_eq!(attrs.outputs[0].promise_within_ms, Some(10));
    }

    #[test]
    fn test_output_duplicate_promise_within_ms_rejected() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(promise_within_ms = 10, promise_within_ms = 20)]
                cmd: Twist,
            }
        };
        let err =
            extract_field_attrs(&input).expect_err("duplicate promise_within_ms must be rejected");
        assert!(err.to_string().contains("duplicate `promise_within_ms`"));
    }

    #[test]
    fn test_output_unknown_paren_form_rejected() {
        // A parenthesized item names the offending ident + accepted forms.
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[output(variable(data))]
                image: Image,
            }
        };
        let err = extract_field_attrs(&input).expect_err("unknown paren form must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown output attribute") && msg.contains("promise_within_ms"),
            "unknown-paren error must name the surviving forms; got: {msg}"
        );
    }

    #[test]
    fn test_extract_mixed_fields() {
        let input: DeriveInput = parse_quote! {
            struct Controller {
                #[input(trigger)]
                scan: LaserScan,
                #[output]
                cmd_vel: Twist,
                min_safe_distance: f64,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert_eq!(attrs.inputs.len(), 1);
        assert_eq!(attrs.outputs.len(), 1);
        // plain field is not extracted
    }

    #[test]
    fn test_no_field_attrs_is_not_declarative() {
        let input: DeriveInput = parse_quote! {
            struct PlainNode {
                counter: u32,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        assert!(!attrs.is_declarative());
    }

    #[test]
    fn a_typo_in_an_input_attr_renders_the_same_accepted_set() {
        // The overwhelmingly common case: a plain typo, which must get the
        // same short "here is what IS accepted" text as any other unknown.
        assert_unknown_attr_error(
            parse_quote! {
                struct MyNode {
                    #[input(foobar)]
                    data: SensorData,
                }
            },
            "foobar",
        );
    }

    #[test]
    fn test_extract_input_with_sample_backpressure() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[input(backpressure = sample(50))]
                data: SensorData,
            }
        };
        let attrs = extract_field_attrs(&input).unwrap();
        let inp = &attrs.inputs[0];
        assert_eq!(inp.backpressure, Some(ParsedBackpressurePolicy::Sample(50)));
    }

    #[test]
    fn test_unknown_backpressure_error() {
        let input: DeriveInput = parse_quote! {
            struct MyNode {
                #[input(backpressure = explode)]
                data: SensorData,
            }
        };
        let result = extract_field_attrs(&input);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("unknown backpressure policy"));
    }
}
