// SPDX-License-Identifier: AGPL-3.0-only
//! Code generation for `#[cerulion_node(...)]`.
//!
//! Generates:
//! 1. The original struct (preserved as-is, with `#[input]`/`#[output]` attrs stripped)
//! 2. A `{Name}Entry` wrapper struct with `NodeEntry` impl
//! 3. `new()` (requires `Default`) and `with_state()` constructors
//! 4. cdylib FFI entry points behind `#[cfg(feature = "cdylib")]`
//!
//! Invariants:
//! - Every `#[cerulion_node]` must declare at least one `#[input]` or
//!   `#[output]` field attribute.
//! - The node type is the folder name (`nodes/<type>/`), resolved at
//!   graph-load time. The macro takes no `type_name` argument; the
//!   generated FFI export carries no `node_type` field.
//! - Diagnostic strings inside generated code use a `snake_case`
//!   conversion of the struct identifier purely for error messages —
//!   never as a "node type."

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::parse::Parser;
use syn::DeriveInput;

use crate::parse::{FieldAttrs, FieldInputAttr, FieldOutputAttr, NodeAttr};
use crate::registry::{self, NodePortEntry, RegisteredPort};
use crate::state_derive;

/// Convert a `CamelCase` / `PascalCase` identifier to `snake_case`.
///
/// `SafetyController` → `safety_controller`. Used by codegen to produce
/// readable diagnostic labels in error messages emitted by the
/// generated `NodeEntry` impl.
///
/// Hand-rolled to avoid pulling in `convert_case` (or `heck`) just for
/// this one conversion. Rules:
/// - Insert `_` between a lowercase/digit and an uppercase letter.
/// - Insert `_` before the last uppercase of a run of uppercases when
///   followed by a lowercase (e.g. `HTTPServer` → `http_server`).
/// - Lowercase everything.
fn to_snake_case(name: &str) -> String {
    let chars: Vec<char> = name.chars().collect();
    let mut out = String::with_capacity(name.len() + 4);
    for i in 0..chars.len() {
        let c = chars[i];
        if i > 0 && c.is_ascii_uppercase() {
            let prev = chars[i - 1];
            let next = chars.get(i + 1).copied();
            // Boundary 1: prev is lowercase or digit -> insert underscore
            // (camelCase -> camel_case).
            if prev.is_ascii_lowercase() || prev.is_ascii_digit() {
                out.push('_');
            } else if prev.is_ascii_uppercase()
                && next.map(|n| n.is_ascii_lowercase()).unwrap_or(false)
            {
                // Boundary 2: run-of-uppercase followed by lowercase
                // (HTTPServer -> http_server). Insert before the current
                // upper so the trailing lowercase joins the next word.
                out.push('_');
            }
        }
        out.push(c.to_ascii_lowercase());
    }
    out
}

/// Generate the full expansion for a `#[cerulion_node]` annotated struct.
pub fn generate(attr: &NodeAttr, field_attrs: &FieldAttrs, input: &DeriveInput) -> TokenStream {
    // Only structs with named or unit fields are supported.
    match &input.data {
        syn::Data::Struct(data) => {
            if !matches!(data.fields, syn::Fields::Named(_))
                && !matches!(data.fields, syn::Fields::Unit)
            {
                return syn::Error::new_spanned(
                    &input.ident,
                    "#[cerulion_node] can only be applied to structs with named or unit fields, not tuple structs",
                )
                .to_compile_error();
            }
        }
        _ => {
            return syn::Error::new_spanned(
                &input.ident,
                "#[cerulion_node] can only be applied to structs",
            )
            .to_compile_error();
        }
    }

    // Every `#[cerulion_node]` must declare at least one `#[input]`
    // or `#[output]` field attribute.
    if !field_attrs.is_declarative() {
        return syn::Error::new_spanned(
            &input.ident,
            "#[cerulion_node] requires at least one #[input] or #[output] field attribute. \
             Use `#[cerulion_node]` + `#[cerulion_node_impl]` with `#[input]` / `#[output]` \
             field attrs.",
        )
        .to_compile_error();
    }

    // Detect whether the user already wrote
    // `#[derive(Default)]` so we don't double-inject. We do NOT reject
    // user-written derives — bundling the hidden runtime context into a
    // single wrapper field with its own `Default` (`CerNodeRuntimeFields`)
    // makes user derives harmless. This sidesteps two bugs the
    // adversarial test suite exposes in a rejection-based design:
    // (1) path-qualified `::std::default::Default` slipped past
    // `is_ident("Default")`, and (2) when `#[derive(Default)]` was
    // positioned BEFORE `#[cerulion_node]`, the derive expansion had
    // already run and the rejection check saw nothing — leaving rustc
    // to surface a confusing `E0119`/`E0063` cascade exposing the
    // hidden field name. Auto-injection + wrapper-field architecture
    // makes both ordering and path syntax irrelevant.
    let user_already_derives_default = user_derives_default(input);

    let struct_name = &input.ident;
    let entry_name = format_ident!("{}Entry", struct_name);
    let vis = &input.vis;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    // Diagnostic label used in error messages emitted by the generated
    // NodeEntry impl. Derived from the struct ident; never a "node type."
    let diag_label = to_snake_case(&struct_name.to_string());

    // Port names come from field attributes (declarative mode only).
    let input_names: Vec<String> = field_attrs.input_names();
    let output_names: Vec<String> = field_attrs.output_names();

    // Register declarative-mode ports + types for the sibling
    // `#[cerulion_node_impl]` macro to discover (this kills
    // the duplicate `inputs(...)`/`outputs(...)` redeclaration footgun).
    let entry = NodePortEntry {
        inputs: field_attrs
            .inputs
            .iter()
            .map(|i| RegisteredPort {
                name: i.field_name.to_string(),
                type_tokens: registry::type_to_string(&i.field_type),
            })
            .collect(),
        outputs: field_attrs
            .outputs
            .iter()
            .map(|o| RegisteredPort {
                name: o.field_name.to_string(),
                type_tokens: registry::type_to_string(&o.field_type),
            })
            .collect(),
        // Carry the determinism opt-out flags from the
        // `#[cerulion_node(...)]` attrs into the registry so the sibling
        // `#[cerulion_node_impl]` can gate its deny lint. (No warn keys are
        // recorded — the macro half emits deny errors only; warn-surfacing is
        // the deferred core half. See `determinism.rs` module docs.)
        allow_non_deterministic: attr.node_level.allow_non_deterministic,
        uses_live_io: attr.node_level.uses_live_io,
        // Record `external` so the sibling
        // `#[cerulion_node_impl]` can require the user's `external_source`
        // method on this node type.
        external: attr.node_level.external,
    };
    registry::register(&struct_name.to_string(), entry);

    let wrapper = gen_wrapper(
        &entry_name,
        struct_name,
        vis,
        &impl_generics,
        &ty_generics,
        where_clause,
    );

    // Inject `now_ns` and `request_shutdown` shim methods
    // on the user struct so node bodies can call `self.now_ns()` /
    // `self.request_shutdown()` directly, regardless of whether they're
    // inside the impl-macro AST rewriter's tick scope.
    let shim_methods = gen_shim_methods(struct_name, &impl_generics, &ty_generics, where_clause);

    // Surface the trigger-input field name for the
    // single-trigger-input case (the canonical `#[input(trigger)]
    // count: T` shape, no node-level policy attr). The macro's own
    // validator (`validate.rs::validate_trigger_inference`) already
    // enforces that 0 or 1 trigger inputs is valid as-is, while 2+
    // require an explicit `sync_window_ms` — which the existing Sync
    // codegen arm handles. So this is exactly: "if there's one
    // trigger input AND no node-level attr, emit `data_trigger`".
    let single_trigger_input_name: Option<String> = {
        let triggers: Vec<&crate::parse::FieldInputAttr> =
            field_attrs.inputs.iter().filter(|i| i.trigger).collect();
        if triggers.len() == 1 {
            Some(triggers[0].field_name.to_string())
        } else {
            None
        }
    };

    // Graph YAML carries no `policy:` block,
    // so the macro's policy must reach the in-process `info()` impl
    // (as well as the cdylib FFI JSON). Build a
    // `MacroPolicy` token expression to chain `.with_policy(...)` onto
    // the `NodeInfo` returned by `info()`.
    let policy_with_call: TokenStream = if let Some(ms) = attr.node_level.period_ms {
        quote! {
            .with_policy(::cerulion_core::graph::node::MacroPolicy::Period { period_ms: #ms })
        }
    } else if let Some(ms) = attr.node_level.sync_window_ms {
        quote! {
            .with_policy(::cerulion_core::graph::node::MacroPolicy::Sync { window_ms: #ms })
        }
    } else if attr.node_level.unbounded_sync {
        quote! {
            .with_policy(::cerulion_core::graph::node::MacroPolicy::UnboundedSync)
        }
    } else if attr.node_level.external {
        quote! {
            .with_policy(::cerulion_core::graph::node::MacroPolicy::External)
        }
    } else if let Some(name) = single_trigger_input_name.as_deref() {
        let name_lit = name.to_string();
        quote! {
            .with_policy(::cerulion_core::graph::node::MacroPolicy::DataTrigger {
                input_name: #name_lit.to_string(),
            })
        }
    } else {
        TokenStream::new()
    };

    // Per-node tick execution deadline. Chains
    // independently of policy_with_call — `tick_within_ms` is a QoS
    // annotation, not a firing rule, so it composes with any trigger
    // policy.
    let tick_within_with_call: TokenStream = if let Some(ms) = attr.node_level.tick_within_ms {
        quote! { .with_tick_within_ms(#ms) }
    } else {
        TokenStream::new()
    };

    // Node-level producer rate cap. Like tick_within_ms it is a
    // node-level execution policy that composes with the trigger policy
    // (mutually exclusive with period_ms, enforced in validate.rs).
    let throttle_with_call: TokenStream = if let Some(ms) = attr.node_level.throttle_ms {
        quote! { .with_throttle_ms(#ms) }
    } else {
        TokenStream::new()
    };

    let params = NodeEntryParams {
        entry_name: &entry_name,
        struct_name,
        diag_label: &diag_label,
        input_names: &input_names,
        outputs: &field_attrs.outputs,
        impl_generics: &impl_generics,
        ty_generics: &ty_generics,
        where_clause,
        policy_with_call: &policy_with_call,
        tick_within_with_call: &tick_within_with_call,
        throttle_with_call: &throttle_with_call,
        input_meta_attrs: &field_attrs.inputs,
        is_external: attr.node_level.external,
    };

    // Declarative mode → `#[cerulion_node_impl]` emits
    // `__cer_zero_copy_tick(ctx)` on the inner struct; we dispatch
    // straight into it.
    let node_entry_impl = gen_zero_copy_node_entry_impl(&params);

    let cdylib = gen_cdylib(
        &entry_name,
        &input.ident,
        &field_attrs.inputs,
        &output_names,
        &field_attrs.outputs,
        &attr.node_level,
        single_trigger_input_name.as_deref(),
    );

    // Strip #[input] and #[output] attributes from the original struct,
    // then inject the hidden runtime-context field and
    // (if not already present) `#[derive(Default)]`.
    let mut stripped_input =
        strip_field_attrs_and_inject_hidden(input, !user_already_derives_default);

    // An EXPLICIT `#[derive(CerulionState)]` on a node struct
    // collides with the fold-in below (E0119). Say so in our own words, at
    // the derive — and DROP the entry from the struct we re-emit, so the user
    // gets exactly this one error instead of it plus the rustc conflict plus
    // whatever the derive's own (port-walking) expansion raises. The other
    // attribute order is caught inside the derive; see `redundant_state_derive`
    // for both, and for the one residual (a renamed import).
    let mut redundant_derive = None;
    if let Some(span) = state_derive::redundant_state_derive(&input.attrs) {
        strip_state_derive(&mut stripped_input.attrs);
        redundant_derive =
            Some(syn::Error::new(span, state_derive::REDUNDANT_DERIVE_MSG).to_compile_error());
    }

    // The state derive is folded in here: the state
    // impl is emitted from the ONE place `#[derive(CerulionState)]` emits it,
    // over this struct's non-port fields, so an ordinary node is capturable
    // with zero new user lines and a node cannot disagree with a hand-derived
    // helper about what a field means.
    //
    // It lands in a PLAIN impl emitted from here, which is what keeps it out
    // of `SelfPortRewriter`'s way: that rewriter only visits
    // `#[cerulion_node_impl]` bodies.
    let state_impl = match state_derive::expand_node(input) {
        Ok(tokens) => tokens,
        Err(err) => err.to_compile_error(),
    };

    quote! {
        #stripped_input

        #redundant_derive
        #state_impl
        #shim_methods
        #wrapper
        #node_entry_impl
        #cdylib
    }
}

/// Drop the `CerulionState` entry from every `#[derive(..)]` on a struct we are
/// about to re-emit.
///
/// Paired with the `compile_error!` at the call site: leaving the entry in
/// place would run the derive on the emitted struct and stack rustc's `E0119`
/// (plus the derive's own port-field failures) on top of a message that already
/// said the whole story. An attribute whose list empties is removed outright
/// rather than left as a bare `#[derive()]`.
///
/// `pub(crate)` because `lib.rs`'s two ERROR arms re-emit the user's struct as
/// well and need the identical strip. One implementation, so the success path
/// and the salvage paths cannot disagree about what a redundant derive is —
/// the same argument the fold-in itself makes for sharing `expand_fields`.
pub(crate) fn strip_state_derive(attrs: &mut Vec<syn::Attribute>) {
    let mut rewritten: Vec<syn::Attribute> = Vec::with_capacity(attrs.len());
    for attr in attrs.drain(..) {
        if !attr.path().is_ident("derive") {
            rewritten.push(attr);
            continue;
        }
        let parser = syn::punctuated::Punctuated::<syn::Path, syn::Token![,]>::parse_terminated;
        let Ok(paths) = attr.parse_args_with(parser) else {
            // Unparseable derive list: leave it exactly as written. rustc will
            // report it far better than a macro guessing at the user's intent.
            rewritten.push(attr);
            continue;
        };
        let kept: Vec<syn::Path> = paths
            .into_iter()
            .filter(|path| !state_derive::path_names_state_derive(path))
            .collect();
        if kept.is_empty() {
            continue;
        }
        rewritten.push(syn::parse_quote!(#[derive(#(#kept),*)]));
    }
    *attrs = rewritten;
}

/// Returns true if the user struct already carries any
/// form of `#[derive(... Default ...)]` derive (single segment, qualified
/// path, in any position relative to `#[cerulion_node]`).
///
/// Used purely to avoid double-deriving — we don't reject either way.
/// Hidden runtime context lives in a single wrapper field with its own
/// `Default`, so user-derived and macro-derived defaults coexist
/// peacefully.
fn user_derives_default(input: &DeriveInput) -> bool {
    for attr in &input.attrs {
        if !attr.path().is_ident("derive") {
            continue;
        }
        let mut found = false;
        let _ = attr.parse_nested_meta(|meta| {
            // Match `Default` regardless of path qualification —
            // `Default`, `core::default::Default`, `::std::default::Default`
            // all have `Default` as the last segment ident.
            if meta
                .path
                .segments
                .last()
                .is_some_and(|seg| seg.ident == "Default")
            {
                found = true;
            }
            Ok(())
        });
        if found {
            return true;
        }
    }
    false
}

/// Strip `#[input(...)]` and `#[output]` field attributes, append the
/// hidden runtime-context field, and optionally inject
/// `#[derive(Default)]` at the struct level.
///
/// The hidden runtime context bundles the runtime clock and shared
/// shutdown signal into one field of type `CerNodeRuntimeFields`. That
/// type implements `Default` itself, so the user can `#[derive(Default)]`
/// freely without colliding with the macro — and conversely the macro
/// can auto-derive on their behalf when they don't write it.
///
/// `inject_default_derive` should be `false` whenever the user struct
/// already carries any form of `Default` derive (per `user_derives_default`)
/// to avoid the `E0119 conflicting impls` error from a double-derive.
fn strip_field_attrs_and_inject_hidden(
    input: &DeriveInput,
    inject_default_derive: bool,
) -> DeriveInput {
    let mut output = input.clone();
    if inject_default_derive {
        let derive_attr: syn::Attribute = syn::parse_quote!(#[derive(::std::default::Default)]);
        output.attrs.push(derive_attr);
    }
    if let syn::Data::Struct(ref mut data) = output.data {
        // Promote a unit struct to a named-fields struct so we can append
        // the hidden runtime field uniformly. This intentionally does not
        // change the user-visible struct kind for non-unit structs.
        if matches!(data.fields, syn::Fields::Unit) {
            data.fields = syn::Fields::Named(syn::FieldsNamed {
                brace_token: syn::token::Brace::default(),
                named: syn::punctuated::Punctuated::new(),
            });
        }
        if let syn::Fields::Named(ref mut fields) = data.fields {
            for field in &mut fields.named {
                let was_port_field = field
                    .attrs
                    .iter()
                    .any(|a| a.path().is_ident("input") || a.path().is_ident("output"));
                // `cerulion` joins the strip list for the SAME reason as the
                // port attrs: `#[cerulion_node]` is an attribute macro, so
                // nothing registers an inert `#[cerulion(...)]` helper for it
                // (only a `#[proc_macro_derive(.., attributes(cerulion))]`
                // does that, and a node struct goes through no derive). Left
                // on the emitted struct it reaches rustc as `cannot find
                // attribute 'cerulion' in this scope` — a second error on top
                // of whatever the user was actually doing. The state impl is
                // built from `input`, which still carries every attribute, so
                // stripping here loses nothing.
                field.attrs.retain(|attr| {
                    !attr.path().is_ident("input")
                        && !attr.path().is_ident("output")
                        && !attr.path().is_ident("cerulion")
                });
                // Port fields are zero-sized markers
                // that the impl-macro AST rewriter accesses via per-tick
                // locals (`__cer_<port>`), never through the user
                // struct's field. From rustc's perspective the field is
                // dead; from the macro's perspective it carries the
                // schema type. Inject `#[allow(dead_code)]` here so
                // user code never has to. (Allowed under the dead-code policy's
                // narrow exception: "intentionally-unused-via-external-
                // mechanism, with a comment explaining why".)
                if was_port_field {
                    let allow_attr: syn::Attribute = syn::parse_quote!(#[allow(dead_code)]);
                    field.attrs.push(allow_attr);
                }
            }
            // Single bundled hidden field. `CerNodeRuntimeFields` has its
            // own `Default`, so any `#[derive(Default)]` (user-written or
            // macro-injected) initialises this field via that impl.
            let rt_field: syn::Field = syn::Field::parse_named
                .parse2(quote! {
                    #[doc(hidden)]
                    pub __cer_rt: ::cerulion_core::graph::node::CerNodeRuntimeFields
                })
                .expect("hidden field __cer_rt parses");
            fields.named.push(rt_field);
        }
    }
    output
}

/// Emit the user-facing shim
/// methods on the user struct so tick / init / shutdown bodies can call
/// them directly without going through `NodeContext`.
///
/// The shim exposes a four-method clock surface:
///
/// - `self.now_ns() -> u64` — the PRIMARY active-source,
///   determinism-safe read for node code. Returns whatever the
///   ACTIVE clock's time is (`CLOCK_MONOTONIC` under `RealClock`, the
///   advanced counter under `VirtualClock`, the latched master time
///   under `ExternalClock`). This is what the scheduler uses to evaluate
///   trigger policies, so a node reading `now_ns()` stays consistent
///   with its own firing — and it replays bit-for-bit under
///   `VirtualClock`. Use this by default.
///
///   (The ambiguity the original `now_ns()` shim had was
///   real-vs-sim — it returned whatever the runtime injected with no way
///   to name a source. This resolves that by KEEPING `now_ns()` as
///   the explicit "active source" read and adding the three
///   source-specific escape hatches below for when a node must name a
///   SPECIFIC clock regardless of which one is active.)
///
/// - `self.real_ns() -> u64` — explicit-source escape hatch: always
///   `CLOCK_MONOTONIC` (or platform equivalent), independent of the
///   runtime's clock injection. Use for benchmarks, timeouts, anything
///   coupled to real-world durations.
/// - `self.virt_ns() -> Option<u64>` — explicit-source escape hatch:
///   `Some(t)` under `VirtualClock`, `None` (warn-once) otherwise.
///   Forces the user to acknowledge "this only exists in replay/test
///   mode."
/// - `self.ext_ns() -> Option<u64>` — explicit-source escape hatch:
///   `Some(t)` under `ExternalClock` (an external time master, e.g. a
///   sim's published `/clock`), `None` (warn-once) otherwise.
///
/// `self.request_shutdown()` is unchanged.
fn gen_shim_methods(
    struct_name: &syn::Ident,
    impl_generics: &syn::ImplGenerics,
    ty_generics: &syn::TypeGenerics,
    where_clause: Option<&syn::WhereClause>,
) -> TokenStream {
    quote! {
        #[automatically_derived]
        impl #impl_generics #struct_name #ty_generics #where_clause {
            /// Active-source clock ns — the PRIMARY determinism-safe read
            /// for node code (clock-source shim).
            ///
            /// Returns whatever the ACTIVE clock reports: kernel-monotonic
            /// wall time under `RealClock` (`CLOCK_MONOTONIC` on Linux,
            /// `CLOCK_UPTIME_RAW` on macOS — see `clock::real_ns`), the
            /// advanced counter under
            /// `VirtualClock`, the latched master time under
            /// `ExternalClock`. This matches the time the scheduler uses
            /// to evaluate trigger policies, so it stays consistent with
            /// the node's own firing and replays bit-for-bit under
            /// `VirtualClock`. Use this by default; reach for
            /// `real_ns()` / `virt_ns()` / `ext_ns()` only when you must
            /// name a SPECIFIC clock source regardless of which one is
            /// active.
            #[inline]
            #[doc(hidden)]
            pub fn now_ns(&self) -> u64 {
                ::cerulion_core::clock::Clock::now_ns(&*self.__cer_rt.clock)
            }

            /// Raw wall-clock ns (clock-source shim):
            /// `CLOCK_MONOTONIC` on Linux, `CLOCK_UPTIME_RAW` on macOS
            /// (see `clock::real_ns`), both suspend-excluding.
            ///
            /// Explicit-source escape hatch: always reads the kernel
            /// monotonic clock — independent of the runtime's clock
            /// injection. Use for latency measurements, timeouts, anything
            /// that must match real-world durations. Cost: one syscall on
            /// Linux/macOS (vDSO-accelerated, ~10ns); one
            /// `QueryPerformanceCounter` on Windows.
            #[inline]
            #[doc(hidden)]
            pub fn real_ns(&self) -> u64 {
                ::cerulion_core::clock::real_ns()
            }

            /// Virtual/controlled-clock ns, or `None` if the runtime is
            /// not using `VirtualClock` (clock-source
            /// shim).
            ///
            /// Explicit-source escape hatch: `Some(t)` under
            /// `VirtualClock` (replay, deterministic tests), `None`
            /// (warn-once) otherwise. Use inside
            /// `if let Some(t) = self.virt_ns() { ... }` blocks for
            /// replay-equivalence assertions; use `now_ns()` for the
            /// active clock or `real_ns()` for any code path that must
            /// work under either runtime.
            #[inline]
            #[doc(hidden)]
            pub fn virt_ns(&self) -> ::std::option::Option<u64> {
                ::cerulion_core::clock::Clock::virt_ns(&*self.__cer_rt.clock)
            }

            /// External-master clock ns, or `None` if the runtime is NOT
            /// using `ExternalClock` (clock-source shim).
            ///
            /// Explicit-source escape hatch: `Some(t)` under
            /// `ExternalClock` — an external time master, e.g. a sim's
            /// published `/clock` — `None` (warn-once) otherwise. Use
            /// `now_ns()` for the active clock when you don't need to
            /// name the external source specifically.
            #[inline]
            #[doc(hidden)]
            pub fn ext_ns(&self) -> ::std::option::Option<u64> {
                ::cerulion_core::clock::Clock::ext_ns(&*self.__cer_rt.clock)
            }

            /// Request graceful shutdown of the graph.
            ///
            /// Idempotent. The runtime polls the shared signal between
            /// ticks and exits cleanly.
            #[inline]
            #[doc(hidden)]
            pub fn request_shutdown(&self) {
                self.__cer_rt.shutdown_signal.request();
            }
        }
    }
}

/// Generate the `{Name}Entry` wrapper struct and constructors.
fn gen_wrapper(
    entry_name: &syn::Ident,
    struct_name: &syn::Ident,
    vis: &syn::Visibility,
    impl_generics: &syn::ImplGenerics,
    ty_generics: &syn::TypeGenerics,
    where_clause: Option<&syn::WhereClause>,
) -> TokenStream {
    // `new()` needs `#struct_name: Default` on top of whatever the user wrote.
    // It cannot be appended as a second `where`: a node declared
    // `struct Node<T> where T: Send` already supplies one, and two on one item
    // is `error: cannot define duplicate 'where' clauses on an item` — which
    // spans back to the USER'S struct, so a node written with a where clause
    // instead of inline bounds reported a syntax error it did not contain.
    // (Pre-existing, and independent of the state forwards below: reproduced
    // with those removed. Pinned by the `WhereClauseNode` half of
    // `tests/ui/pass/generic_node_state_bounds_pass.rs`.)
    let default_where_clause = {
        let existing = where_clause.map(|w| {
            let preds = &w.predicates;
            quote! { #preds }
        });
        quote! {
            where
                #struct_name #ty_generics: Default,
                #existing
        }
    };
    quote! {
        #vis struct #entry_name #impl_generics #where_clause {
            /// The underlying node state.
            inner: #struct_name #ty_generics,
            context: Option<::cerulion_core::graph::node::NodeContext>,
            /// The runtime-classified NON-trigger latest-value input
            /// names, marshalled across the cdylib FFI ONCE via
            /// `cerulion_node_set_snapshot_inputs` and read on every step by
            /// `cerulion_node_snapshot_inputs` so the per-step freeze allocates
            /// nothing. Read ONLY by the `#[cfg(feature = "cdylib")]` FFI module
            /// below; in the in-process (non-cdylib) build the host passes the
            /// names directly to `NodeEntry::snapshot_inputs`, so this field is
            /// unused there — `allow(dead_code)` rather than cfg-gate the field
            /// (a cfg here would trip `unexpected_cfgs` in crates that don't
            /// declare the `cdylib` feature; an empty `Vec` never allocates).
            #[allow(dead_code)]
            __snapshot_input_names: ::std::vec::Vec<::std::string::String>,
        }

        impl #impl_generics #entry_name #ty_generics
            #default_where_clause
        {
            /// Create a new entry with default state.
            pub fn new() -> Self {
                Self {
                    inner: #struct_name::default(),
                    context: None,
                    __snapshot_input_names: ::std::vec::Vec::new(),
                }
            }
        }

        impl #impl_generics #entry_name #ty_generics #where_clause {
            /// Create a new entry with the given initial state.
            pub fn with_state(inner: #struct_name #ty_generics) -> Self {
                Self {
                    inner,
                    context: None,
                    __snapshot_input_names: ::std::vec::Vec::new(),
                }
            }
        }
    }
}

/// Generate the declarative-mode `NodeEntry` impl (zero-copy dispatch).
///
/// Calls `inner.__cer_zero_copy_tick(ctx)` — the method emitted by
/// `#[cerulion_node_impl]` — directly. Every `self.<port>` access in the
/// user's tick body has already been rewritten by the impl macro to
/// dispatch through the SHM-backed proxy/view, so no pre/post-tick
/// snapshot bookkeeping is needed.
fn gen_zero_copy_node_entry_impl(params: &NodeEntryParams) -> TokenStream {
    let NodeEntryParams {
        entry_name,
        struct_name,
        diag_label,
        input_names,
        outputs,
        impl_generics,
        ty_generics,
        where_clause,
        policy_with_call,
        tick_within_with_call,
        throttle_with_call,
        input_meta_attrs: _,
        is_external,
    } = params;
    // An `#[cerulion_node(external)]` node OVERRIDES
    // `NodeEntry::external_source()` to surface the user's required
    // `external_source` method (renamed to `__cer_user_external_source` by
    // `#[cerulion_node_impl]`). Non-external nodes emit NO override and inherit
    // the trait default (`None`) — an external node without the method is a
    // compile error emitted by the impl macro, and the impl macro also injects a
    // `__cer_user_external_source` stub in that error case so this call still
    // resolves (the user sees only the actionable diagnostic, no E0599 cascade).
    let external_source_method: TokenStream = if *is_external {
        quote! {
            fn external_source(
                &mut self,
            ) -> ::std::option::Option<::cerulion_core::graph::node::ExternalSource> {
                ::std::option::Option::Some(self.inner.__cer_user_external_source())
            }
        }
    } else {
        TokenStream::new()
    };
    // Populate `OutputMeta::schema_hash` and
    // `OutputMeta::max_slice_len_default` from the declared field
    // type's `<T as ShmMessage>::{SCHEMA_HASH, MAX_SLICE_LEN}` consts,
    // so the runtime's tier-2 resolution at graph load time can read
    // the per-schema budget without re-entering codegen. Each `#[output]`
    // field's `field_type` is the schema marker (e.g. `Vector3`).
    // Emit full `InputMeta` per `#[input(...)]`
    // field so per-input attrs like `expect_within_ms`, `trigger`,
    // depth, etc. reach the runtime. Earlier codegen used a
    // names-only path which silently dropped field-level metadata.
    let input_meta_exprs: Vec<TokenStream> = params
        .input_meta_attrs
        .iter()
        .map(|i| {
            let name = i.field_name.to_string();
            let ty = &i.field_type;
            let trigger = i.trigger;
            // Depth-default collapse: with the never-executed queue
            // policy gone, an unspecified `depth` is uniformly
            // DEFAULT_CONSUMER_DEPTH — emitted as the const PATH (not a
            // baked literal) so the macro can never drift from the
            // runtime's value.
            let depth_tok: TokenStream = match i.depth {
                Some(d) => quote! { #d },
                None => {
                    quote! { ::cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH }
                }
            };
            let backpressure_tok: TokenStream = match &i.backpressure {
                Some(crate::parse::ParsedBackpressurePolicy::Block) => {
                    quote! { ::cerulion_core::graph::node::BackpressurePolicy::Block }
                }
                // `Some(Sample(n))` needs its own arm: falling into
                // the `_` catch-all below would SILENTLY downgrade it to
                // `DropOldest`, and `#[input(backpressure = sample(50))]`
                // would never reach the runtime as a Sample. Emit the real
                // `Sample(n)` so the graph-runtime enforcement sees it.
                Some(crate::parse::ParsedBackpressurePolicy::Sample(ms)) => {
                    quote! { ::cerulion_core::graph::node::BackpressurePolicy::Sample(#ms) }
                }
                // Explicit `drop_oldest` OR no declaration → the
                // `DropOldest` default.
                Some(crate::parse::ParsedBackpressurePolicy::DropOldest) | None => {
                    quote! { ::cerulion_core::graph::node::BackpressurePolicy::DropOldest }
                }
            };
            let expect_within_tok: TokenStream = match i.expect_within_ms {
                Some(ms) => quote! { ::std::option::Option::Some(#ms) },
                None => quote! { ::std::option::Option::None },
            };
            quote! {
                ::cerulion_core::graph::node::InputMeta {
                    name: #name.to_string(),
                    schema_hash: <#ty as ::cerulion_core::message::ShmMessage>::SCHEMA_HASH,
                    trigger: #trigger,
                    depth: #depth_tok,
                    backpressure: #backpressure_tok,
                    expect_within_ms: #expect_within_tok,
                }
            }
        })
        .collect();

    // Forward `NodeEntry::snapshot_inputs` to the
    // NodeContext. The RUNTIME decides WHICH inputs are non-triggering
    // (latest-value) reads and passes their names (the complement of
    // `build_trigger_edges` — single source of truth; the macro does NOT
    // classify triggers). The runtime (fire-gated) will only invoke
    // this for nodes that have ≥1 non-triggering input, so the body is an
    // unconditional thin delegate.
    let snapshot_inputs_method = quote! {
        fn snapshot_inputs(&mut self, inputs: &[::std::string::String]) {
            if let Some(ctx) = self.context.as_mut() {
                ctx.snapshot_inputs(inputs);
            }
        }

        // A macro node's `snapshot_inputs` REALLY freezes
        // (forwards to `NodeContext::snapshot_inputs`), so report `true`. The
        // level executor uses this to keep macro nodes ON the within-level rayon
        // parallel fire path (a real step-boundary freeze gives them replay =
        // live under parallelism); cdylib/closure nodes inherit the `false`
        // default and fire serially when they have non-trigger inputs.
        fn performs_input_snapshot(&self) -> bool {
            true
        }

        // Forward `NodeEntry::drain_trigger_input` to the
        // `NodeContext`. The runtime calls this (through the node lock, at the
        // level boundary BEFORE decide) for ELIGIBLE data-trigger inputs to
        // drain the BODY subscriber ONCE — freezing the surviving sample so the
        // tick's later `try_view` serves it without a second receive — and
        // returns `(popped, latest_ts)` for `signal_data` / `signal_input_received`.
        // This forwarding is what makes a macro node UNIFIABLE: without it the
        // node would inherit the `(0, None)` default and never fire when the
        // runtime eliminated its separate trigger-drain subscriber. cdylib
        // nodes keep the default and retain the dual-subscriber path unless
        // they export the optional drain FFI symbol; `ClosureNodeEntry`
        // forwards like this too.
        fn drain_trigger_input(
            &mut self,
            input_name: &str,
        ) -> (u64, ::core::option::Option<u64>) {
            if let Some(ctx) = self.context.as_mut() {
                ctx.drain_trigger_input(input_name)
            } else {
                (0, ::core::option::Option::None)
            }
        }

        // Declare the drain capability the forwarding above
        // provides. Decoupled from `performs_input_snapshot` (the rayon flag
        // the unified drain piggybacked on) — the runtime's eligibility predicate
        // (`NodeInfo::unifies_data_trigger`) consumes THIS bool. Macro nodes
        // always unify.
        fn unifies_trigger_drain(&self) -> bool {
            true
        }

        // The between-fires REFILL twin of the forward above. The
        // scheduler's Data burst loop asks for the next queued frame after each
        // fire, and it must ask a question the boundary drain does NOT answer:
        // "did the fire I just ran consume the head?". A macro tick nests one
        // `try_view` per input in DECLARATION order, so an earlier non-trigger
        // `#[input]` with no delivery yet collapses the chain and the trigger's
        // read never runs — the head stays frozen, and the boundary drain's
        // RE-OFFER of it (correct there: Principle #6) would be counted here as
        // a fresh frame and re-fire the node on it.
        fn refill_trigger_input(
            &mut self,
            input_name: &str,
        ) -> (u64, ::core::option::Option<u64>) {
            if let Some(ctx) = self.context.as_mut() {
                ctx.refill_trigger_input(input_name)
            } else {
                (0, ::core::option::Option::None)
            }
        }

        fn refills_trigger_input(&self) -> bool {
            true
        }

        // Forward `NodeEntry::sync_head_op` to the `NodeContext`.
        // The per-set Sync matcher drives its verdicts through ONE multiplexed
        // op rather than four hooks — the four are transitions of ONE state
        // machine, and four hooks would admit partial-capability nodes
        // (advance-without-void) the degrade logic would then have to
        // enumerate.
        //
        // The `None`-context arm answers `Failed`, NOT `Nothing`. It is the
        // same SHAPE the `drain_trigger_input` forward above uses and the same
        // reasoning — report the SAFE answer for this return type — but the
        // safe answer is the opposite one here. `(0, None)` is safe for a
        // drain because "no frames were popped" is true of a node with no
        // subscribers; `Nothing` is NOT safe, because it is positive evidence
        // of SCARCITY that the matcher is entitled to descend on, and a node
        // that has not been init'd has not observed anything at all.
        // `SyncOpAnswer::Failed` is a first-class answer precisely so an
        // op that could not be performed never vouches for an input.
        fn sync_head_op(
            &mut self,
            input_name: &str,
            op: ::cerulion_core::SyncHeadOp,
        ) -> ::cerulion_core::SyncOpAnswer {
            if let Some(ctx) = self.context.as_mut() {
                ctx.sync_head_op(input_name, op)
            } else {
                ::cerulion_core::SyncOpAnswer::Failed
            }
        }

        // Declare the head-op capability the forwarding above
        // provides. A macro node's input reads are the generated `try_view` —
        // the frozen-slot path the ops' contract is written against (a
        // promoted head is SERVED from the slot, and `Void` makes a restored
        // head read as "no frame") — so the contract holds by construction.
        //
        // Emitted for EVERY macro node, not only ones declaring a sync
        // window: the ops are per-INPUT and the GRAPH decides which node gets
        // them, so gating the emission on the node's declared policy would
        // make a runtime capability a function of a declaration the graph is
        // free to override.
        fn supports_sync_head_ops(&self) -> bool {
            true
        }
    };

    // The IN-PROCESS half of the keystone. The fold-in placed an
    // `impl CerulionState` onto every `#[cerulion_node]` struct, but nothing
    // connected it to the runtime's own seams — so an in-process macro node
    // inherited `NodeEntry`'s defaults and `restore_node_states` reported it as
    // declaring no state, exactly like the cdylib. These three forwards are what
    // make the fold-in REACHABLE, and they are the same three the cdylib FFI
    // carries, so the two forms answer identically by construction.
    //
    // `state_shape` reaches the associated const through a generic helper rather
    // than by naming the node type: an associated const cannot be read off a
    // value, and threading the ident into this function would be a second place
    // for the type to be named (and therefore to drift).
    //
    // Two CARRIER forwards sit beside them. They are the
    // erased view the boundary walk reads (`InlineCaptureTarget`): without them
    // a macro node inherits `NodeEntry`'s safe defaults — `inline_safe = false`,
    // so every node takes the fork carrier — and the whole inline path is dead
    // code on a real graph. They ride the same generic-helper trick as
    // `state_shape` for the same reason.
    let state_methods = quote! {
        fn state_shape(&self) -> ::core::option::Option<u64> {
            fn __cer_shape_of<T: ::cerulion_core::state::CerulionState>(_: &T) -> u64 {
                <T as ::cerulion_core::state::CerulionState>::STATE_SHAPE
            }
            ::core::option::Option::Some(__cer_shape_of(&self.inner))
        }

        fn inline_safe(&self) -> bool {
            fn __cer_inline_safe_of<T: ::cerulion_core::state::CerulionState>(_: &T) -> bool {
                <T as ::cerulion_core::state::CerulionState>::INLINE_SAFE
            }
            __cer_inline_safe_of(&self.inner)
        }

        fn cer_probe(&self) -> bool {
            ::cerulion_core::state::CerulionState::cer_probe(&self.inner)
        }

        fn capture_state(
            &self,
            out: &mut dyn ::cerulion_core::state::StateSink,
        ) -> ::cerulion_core::error::TransportResult<()> {
            ::cerulion_core::state::CerulionState::cer_capture(&self.inner, out).map_err(|e| {
                ::cerulion_core::error::TransportError::GraphError {
                    reason: ::std::format!(
                        "node '{}' state capture failed: {}", #diag_label, e,
                    ),
                }
            })
        }

        fn restore_state(
            &mut self,
            payload: &[u8],
        ) -> ::cerulion_core::error::TransportResult<()> {
            let mut cursor = ::cerulion_core::state::StateCursor::new(payload);
            ::cerulion_core::state::CerulionState::cer_restore(&mut self.inner, &mut cursor)
                .map_err(|e| ::cerulion_core::error::TransportError::GraphError {
                    reason: ::std::format!(
                        "node '{}' state restore failed: {}", #diag_label, e,
                    ),
                })?;
            // Trailing bytes mean the running field list disagrees with the
            // recorded one — a drift signal the shape check cannot see, since a
            // `#[cerulion(serde)]` field folds only its NAME into `STATE_SHAPE`.
            cursor.finish().map_err(|e| {
                ::cerulion_core::error::TransportError::GraphError {
                    reason: ::std::format!(
                        "node '{}' state restore failed: {}", #diag_label, e,
                    ),
                }
            })
        }
    };

    let output_meta_exprs: Vec<TokenStream> = outputs
        .iter()
        .map(|o| {
            let name = o.field_name.to_string();
            let ty = &o.field_type;
            // Chain `.with_promise_within_ms(N)` when
            // `#[output(promise_within_ms = N)]` is set, leaving the field
            // None otherwise.
            let promise_within_chain: TokenStream = if let Some(ms) = o.promise_within_ms {
                quote! { .with_promise_within_ms(#ms) }
            } else {
                TokenStream::new()
            };
            // `OutputMeta` is `#[non_exhaustive]`, so callers outside
            // `cerulion_core` (including macro-generated code) must
            // build via `OutputMeta::new(...)` rather than struct-
            // literal syntax.
            //
            // `.with_wire_fixed_size(..)` is UNCONDITIONAL —
            // unlike `promise_within_ms`, this is not a declared attribute
            // whose absence means "not set" but a property every port type
            // has (`<T as ShmMessage>::WIRE_FIXED_SIZE`), and it is what
            // lets the recorder stamp a bag channel from the node rather
            // than from a workspace `schemas/` file that may describe a
            // different layout or none at all. A port that omitted it
            // would silently send the recorder back to the file.
            quote! {
                ::cerulion_core::graph::node::OutputMeta::new(
                    #name.to_string(),
                    <#ty as ::cerulion_core::message::ShmMessage>::SCHEMA_HASH,
                    <#ty as ::cerulion_core::message::ShmMessage>::MAX_SLICE_LEN,
                )
                .with_wire_fixed_size(
                    <#ty as ::cerulion_core::message::ShmMessage>::WIRE_FIXED_SIZE,
                )
                #promise_within_chain
            }
        })
        .collect();
    // The three state forwards above are the only members of this
    // impl raising an obligation the user's own `where` clause does not already
    // carry, so the obligation is stated HERE rather than left to the call
    // sites. ONE predicate — the exact one the bodies raise — for two reasons:
    //
    // 1. A GENERIC node stops compiling without it. The folded state impl is
    //    `impl<T> CerulionState for Node<T> where T: CerulionState`, which is
    //    satisfiable at every instantiation but proves nothing INSIDE
    //    `impl<T>`, where `T` is unknown; so the forwards' obligations went
    //    unsatisfied at the DEFINITION and the author had to hand-write a bound
    //    the macro never asked for. Here the predicate is deferred to the
    //    instantiation instead, which is where `T` is known.
    //
    // 2. `state_derive`'s module docs measure the diagnostic cost of the
    //    alternative ("# The `where` clause is the whole diagnostic design"):
    //    an obligation named only at its uses costs an EXTRA `E0277` block per
    //    span, pointing at the macro rather than at the field. A node with one
    //    un-capturable field reported ONE block before these forwards existed
    //    and FOUR after; this puts it back to one from this impl — the block
    //    that remains is the derive's own, at the field, which is the one that
    //    points somewhere useful.
    //
    // NOT also serde-style `T: CerulionState` per type parameter, though that
    // is what `state_derive::emit_impl` writes. Here it would be pure
    // redundancy: the derive bounds EVERY type parameter unconditionally, so
    // `Node<T>: CerulionState` already implies `T: CerulionState`. MEASURED
    // both ways rather than argued — dropping the per-parameter list fails no
    // test, and the use-site diagnostic for a non-capturable `T` is
    // byte-identical with and without it (it names `NotState` and its missing
    // impl either way, because the predicate chain resolves through the
    // derive's own bound).
    //
    // For a node with no type parameters — every node in this tree today — this
    // is a concrete, already-satisfied bound and nothing changes.
    let state_where_clause = {
        let existing = where_clause.map(|w| {
            let preds = &w.predicates;
            quote! { #preds }
        });
        quote! {
            where
                #struct_name #ty_generics: ::cerulion_core::state::CerulionState,
                #existing
        }
    };

    quote! {
        #[automatically_derived]
        impl #impl_generics ::cerulion_core::graph::node::NodeEntry
            for #entry_name #ty_generics
            #state_where_clause
        {
            fn info(&self) -> ::cerulion_core::error::TransportResult<
                ::cerulion_core::graph::node::NodeInfo,
            > {
                // Emit `OutputMeta` per
                // `#[output]` port populated from the declared field
                // type's trait consts, so the graph runtime's tier-2
                // resolution sees the per-schema budget. Inputs still
                // come through names-only — `InputMeta` plumbing
                // (depth, backpressure, etc.) is a separate concern;
                // tier-2 resolution only depends on outputs.
                // Switch to `with_meta` so
                // per-input QoS (expect_within_ms, depth,
                // trigger flag, etc.) propagates to the runtime.
                // Earlier codegen used `with_input_names_and_output_meta`
                // which discarded all input metadata except the name.
                // `NodeEntry::info` is fallible (cdylib
                // entries can carry corrupted metadata); macro-generated
                // metadata is built in-memory and always returns Ok.
                #[allow(unused_imports)]
                let _ = (#(#input_names),*); // silence unused-name warning
                ::std::result::Result::Ok(
                    ::cerulion_core::graph::node::NodeInfo::with_meta(
                        vec![#(#input_meta_exprs),*],
                        vec![#(#output_meta_exprs),*],
                    )
                    #policy_with_call
                    #tick_within_with_call
                    #throttle_with_call
                )
            }

            fn init(
                &mut self,
                mut context: ::cerulion_core::graph::node::NodeContext,
            ) -> ::cerulion_core::error::TransportResult<()> {
                if self.context.is_some() {
                    return Err(::cerulion_core::error::TransportError::NodeError {
                        node_id: #diag_label.into(),
                        reason: "already initialized (double init)".into(),
                    });
                }
                // Overwrite the user struct's hidden
                // runtime-context fields with the live clock + shared
                // shutdown signal from NodeContext, so `self.now_ns()` /
                // `self.request_shutdown()` shim methods see the real
                // runtime values rather than the neutral defaults that
                // were placed at struct construction.
                self.inner.__cer_rt.clock =
                    ::std::sync::Arc::clone(context.clock());
                self.inner.__cer_rt.shutdown_signal =
                    context.shutdown_signal().clone();
                // Invoke the user's optional `init`
                // method (renamed to `__cer_user_init` by
                // `#[cerulion_node_impl]`; a no-op stub is appended
                // when the user doesn't write one). The user signs as
                // `Result<(), NodeError>`; the impl macro wraps it to
                // return `TransportResult<()>` at the boundary.
                self.inner.__cer_user_init(&mut context)?;
                self.context = Some(context);
                Ok(())
            }

            fn tick(&mut self) -> ::cerulion_core::error::TransportResult<()> {
                let ctx = self.context.as_mut().ok_or_else(|| {
                    ::cerulion_core::error::TransportError::NodeError {
                        node_id: #diag_label.into(),
                        reason: "node not initialized".into(),
                    }
                })?;
                self.inner.__cer_zero_copy_tick(ctx)
            }

            fn shutdown(&mut self) -> ::cerulion_core::error::TransportResult<()> {
                // Invoke the user's optional `shutdown`
                // method before dropping the context (so the user can
                // still touch `self.now_ns()` / write outputs / etc.
                // during shutdown). Errors are propagated; the wrapper
                // still drops the context so a buggy shutdown doesn't
                // leave the node hung in a partially-shut-down state.
                let user_result = self.inner.__cer_user_shutdown();
                self.context = None;
                user_result
            }

            // Drive late-joiner history delivery for this
            // node's publishers without publishing (runtime cadence for
            // quiescent publishers). NOT on the deterministic firing path.
            fn pump_history(&mut self) {
                if let Some(ctx) = self.context.as_mut() {
                    ctx.pump_history();
                }
            }

            #snapshot_inputs_method

            #state_methods

            #external_source_method
        }
    }
}

/// Common parameters for NodeEntry impl generation.
struct NodeEntryParams<'a> {
    entry_name: &'a syn::Ident,
    /// The USER's struct, which `#entry_name` wraps as `inner`. Needed because
    /// the state forwards below name its type in a `where` predicate rather
    /// than only at their uses — see `state_where_clause`.
    struct_name: &'a syn::Ident,
    /// Diagnostic label for error messages — never a "node type."
    diag_label: &'a str,
    input_names: &'a [String],
    /// Full per-output `#[output]` attribute payload, including the
    /// declared `field_type` per port. Consumed by
    /// `gen_zero_copy_node_entry_impl` to populate `OutputMeta`'s
    /// `schema_hash` and `max_slice_len_default` from
    /// `<T as ShmMessage>::SCHEMA_HASH` and `MAX_SLICE_LEN`.
    /// Output names are derived from
    /// `outputs[i].field_name` rather than kept in a second list.
    outputs: &'a [FieldOutputAttr],
    impl_generics: &'a syn::ImplGenerics<'a>,
    ty_generics: &'a syn::TypeGenerics<'a>,
    where_clause: Option<&'a syn::WhereClause>,
    /// The chained `.with_policy(MacroPolicy::...)`
    /// call appended to the `NodeInfo` builder in `info()`. Empty
    /// when no node-level policy attribute is set and no single
    /// `#[input(trigger)]` field is declared (the macro emits
    /// `NodeInfo` with `policy = None` in that case).
    policy_with_call: &'a TokenStream,
    /// Chained `.with_tick_within_ms(N)` call
    /// appended after `policy_with_call`. Empty when
    /// `#[cerulion_node(tick_within_ms = N)]` is not set.
    tick_within_with_call: &'a TokenStream,
    /// Chained `.with_throttle_ms(N)` call appended after
    /// `tick_within_with_call`. Empty when
    /// `#[cerulion_node(throttle_ms = N)]` is not set.
    throttle_with_call: &'a TokenStream,
    /// Full input-attr list, so the codegen
    /// can emit `InputMeta` entries that carry per-input QoS
    /// (expect_within_ms, depth, etc.) through to the
    /// runtime — a names-only path would
    /// silently drop field-level metadata.
    input_meta_attrs: &'a [FieldInputAttr],
    /// The node carries `#[cerulion_node(external)]`. When set,
    /// the generated `NodeEntry` impl OVERRIDES `external_source()` to return
    /// `Some(self.inner.__cer_user_external_source())` (the user's required
    /// method, renamed by `#[cerulion_node_impl]`); non-external nodes emit no
    /// override and inherit the trait default `None`.
    is_external: bool,
}

/// Escape a string for safe embedding in a JSON string literal.
fn json_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Generate cdylib FFI entry points behind `#[cfg(feature = "cdylib")]`.
///
/// ABI v3: handle-based design supporting multiple instances
/// from the same cdylib, plus rich error messaging via per-cdylib LAST_ERROR
/// thread-local + `cerulion_take_last_error` / `cerulion_free_error` exports.
/// - `cerulion_node_init(*mut NodeContext) -> u64` — takes ownership, returns handle (0 = error).
///   Also installs a cdylib-local stderr tracing subscriber (a stopgap)
///   so node-side `tracing` events are host-visible; RUST_LOG comes from the
///   context's frozen env snapshot. See `install_cdylib_stderr_tracing`.
/// - `cerulion_node_tick(u64) -> i32` — tick by handle
/// - `cerulion_node_pump_history(u64) -> i32` — service quiescent late joiners by handle (ABI v7)
/// - `cerulion_node_shutdown(u64) -> i32` — shutdown by handle, removes from map
///
/// The JSON returned by `cerulion_node_info()` carries
/// no `node_type` field. The shape is `{"inputs": [...], "outputs": [...]}` —
/// the loader resolves the type from the cdylib's folder name, not from the
/// FFI export.
fn gen_cdylib(
    entry_name: &syn::Ident,
    // The USER struct's ident (the wrapper's `inner` field type).
    // The state FFI needs it because `CerulionState::STATE_SHAPE` is an
    // associated const on the NODE type, not on the wrapper, and an associated
    // const cannot be reached through a trait object or through a handle.
    node_name: &syn::Ident,
    // ABI v6: inputs become JSON OBJECTS carrying an
    // optional per-input `expect_within_ms` (symmetric with the output
    // objects), so the loader can route the input-watchdog QoS
    // through `InputMeta` across the cdylib FFI. ABI v8 adds
    // the declared `#[input(depth = N)]` AND (same bump) the declared
    // backpressure to the same objects — both are topology-real (buffer
    // sizing / the block pre-fire gate / sample decimation), so leaving
    // them off the wire is a silent test/live divergence. We take the
    // full `FieldInputAttr` slice (not bare names) for these fields.
    // ABI v9 adds the per-input `#[input(trigger)]` mark to the
    // same objects: the host CANNOT derive trigger truth from the
    // policy alone — `MacroPolicy::Sync` / `UnboundedSync` carry no input
    // identity (they only say "align all trigger inputs"), so a
    // `sync`-policy node's per-input trigger membership is unknowable from
    // the policy. The mark rides the wire so the runtime's
    // classification sees the same truth on the FFI path as in-process.
    inputs: &[crate::parse::FieldInputAttr],
    output_names: &[String],
    outputs: &[FieldOutputAttr],
    node_level: &crate::parse::NodeLevelAttrs,
    // When exactly one `#[input(trigger)]` field
    // is declared and no node-level policy attribute is present, the
    // macro's `policy_json` arm below emits a `data_trigger` payload
    // carrying this field name. Earlier builds passed `None` here; the
    // existing four arms keep their original behavior unchanged. The
    // single-trigger filtering happens at the call site so the
    // existing `field_attrs` parsing is the source of truth and this
    // function stays focused on JSON formatting.
    single_trigger_input_name: Option<&str>,
) -> TokenStream {
    // ABI v6: build each input as a JSON OBJECT
    // `{"name":"x","expect_within_ms":N}` (the `expect_within_ms` key is
    // emitted only when the `#[input(expect_within_ms = N)]` attr is
    // present; absent → omitted, parsed via serde default to `None`).
    // ABI v8: the object additionally carries the declared
    // `#[input(depth = N)]` — same present-vs-absent handling: the key is
    // emitted only for an EXPLICIT declaration. An absent key means "not
    // declared", and the HOST resolves it to `DEFAULT_CONSUMER_DEPTH` —
    // deliberately host-side, because this array is a compile-time string
    // and inlining the const's VALUE here would bake a driftable literal
    // into every cdylib (the in-process `InputMeta` emission uses the
    // const PATH for the same reason).
    //
    // Same v8 bump: the declared backpressure rides the same
    // objects. STABLE JSON shape (serde external tagging on the host's
    // `BackpressureJson`): `"backpressure":"drop_oldest"`,
    // `"backpressure":"block"`, or `"backpressure":{"sample":N}`. Same
    // present-vs-absent rule — emitted only when explicitly declared;
    // absent = the host-side `DropOldest` default. Without it a dylib `block`
    // input silently degrades to `DropOldest` in production (Principle-#6
    // adjacent: `block` exists to prevent data loss). This mirrors the always-objects
    // output template below and the parser's untagged `InputJson` enum,
    // which ALSO accepts the legacy bare-string form for forward/backward
    // compat across the ABI gate.
    //
    // The input object ALSO carries `"schema_hash":<T as
    // ShmMessage>::SCHEMA_HASH` — the port type's layout hash. Like the
    // outputs' `schema_hash`, this is a trait const NOT known at proc-macro
    // expansion time, so (unlike the QoS keys below) the inputs array cannot
    // be a plain compile-time string; it is built at first call inside
    // `__cer_build_info_json` (see the `OnceLock` block). Without it the FFI info
    // JSON would carry NO input schema_hash, so every `DylibNodeEntry`-loaded
    // consumer would land `InputMeta.schema_hash == 0` and the ingress-hash
    // resolver would refuse every cdylib consumer ("no consumer with a declared
    // schema"). The conditional QoS keys (trigger / expect_within_ms / depth /
    // backpressure) ARE known at expansion time, so we pre-format them per
    // input into a SUFFIX string; the runtime splices
    // `{"name":..,"schema_hash":<const><suffix>}`.
    let input_field_names: Vec<String> = inputs
        .iter()
        .map(|i| json_escape(&i.field_name.to_string()))
        .collect();
    let input_field_types: Vec<&syn::Type> = inputs.iter().map(|i| &i.field_type).collect();
    let input_key_suffixes: Vec<String> = inputs
        .iter()
        .map(|i| {
            let mut suffix = String::new();
            // ABI v9: the declared `#[input(trigger)]` mark.
            // Present-vs-absent convention (like `depth`): emit
            // `"trigger":true` only for a trigger input; a non-trigger
            // input omits the key and the host serde-defaults it to
            // `false` (a latest-value read).
            if i.trigger {
                suffix.push_str(r#","trigger":true"#);
            }
            if let Some(ms) = i.expect_within_ms {
                suffix.push_str(&format!(r#","expect_within_ms":{ms}"#));
            }
            if let Some(d) = i.depth {
                suffix.push_str(&format!(r#","depth":{d}"#));
            }
            match &i.backpressure {
                Some(crate::parse::ParsedBackpressurePolicy::DropOldest) => {
                    suffix.push_str(r#","backpressure":"drop_oldest""#);
                }
                Some(crate::parse::ParsedBackpressurePolicy::Block) => {
                    suffix.push_str(r#","backpressure":"block""#);
                }
                Some(crate::parse::ParsedBackpressurePolicy::Sample(n)) => {
                    suffix.push_str(&format!(r#","backpressure":{{"sample":{n}}}"#));
                }
                None => {}
            }
            suffix
        })
        .collect();
    // Outputs become a JSON array of objects
    // (`{"name", "schema_hash", "max_slice_len_default"}` — plus
    // `promise_within_ms` and `wire_fixed_size`)
    // so the loader can route tier-2 max_slice_len
    // resolution through `OutputMeta` without re-entering codegen. The
    // `schema_hash`, `max_slice_len_default` and `wire_fixed_size` values
    // are not known at proc-macro expansion time (they depend on
    // `<T as ShmMessage>` codegen), so we emit format-string fragments
    // that the cdylib resolves at first-call via `OnceLock + format!`.
    let _ = output_names; // names are derived from `outputs` instead.
    let output_field_names: Vec<String> = outputs
        .iter()
        .map(|o| json_escape(&o.field_name.to_string()))
        .collect();
    let output_field_types: Vec<&syn::Type> = outputs.iter().map(|o| &o.field_type).collect();
    // ABI v6: per-output `promise_within_ms`. Unlike
    // `schema_hash`/`max_slice_len_default` (trait consts resolved at
    // first-call), this is a plain `Option<u64>` known at expansion time.
    // ALWAYS present in the output object (`null` when None) — matching
    // the existing always-present `schema_hash`/`max_slice_len_default`
    // template fields, so the parser's `OutputJson::Full` reads it via
    // serde default.
    let output_promise_within: Vec<String> = outputs
        .iter()
        .map(|o| match o.promise_within_ms {
            Some(ms) => ms.to_string(),
            None => "null".to_string(),
        })
        .collect();
    // Macro→runtime policy plumbing: serialise the `period_ms` /
    // `sync_window_ms` / `external` macro attrs into the FFI info
    // JSON so the loader can surface them through `NodeInfo::policy`.
    // Without this plumbing the macro accepts these but `let _ = attr;`-discards
    // them; the operator has to redeclare every policy in YAML and
    // the runtime's no-policy warning is misleading.
    // Direct `if let` chain rather than `match` + guard with `.unwrap()`
    // — newer clippy stable trips `unnecessary_unwrap` on the latter and
    // the project's clippy guidance explicitly bans the
    // `is_some()` + `.unwrap()` shape.
    let policy_json = if let Some(ms) = node_level.period_ms {
        format!(r#","policy":{{"period_ms":{ms}}}"#)
    } else if let Some(ms) = node_level.sync_window_ms {
        format!(r#","policy":{{"sync_window_ms":{ms}}}"#)
    } else if node_level.unbounded_sync {
        // Unbounded Sync (loose AND, no timing bound). Without this arm the
        // chain has NO `unbounded_sync` case, so an
        // `#[cerulion_node(unbounded_sync)]` cdylib emits info JSON with no
        // `"policy"` key → the host parses `PolicyJson::default()` /
        // `NodeInfo::policy() == None` → the runtime defaults the node to
        // `TriggerPolicy::Data` (fires on ANY single input arrival) instead of
        // waiting for ALL inputs. A fusion cdylib silently loses its all-inputs
        // contract. Placed AFTER `sync_window_ms` / BEFORE `external` to mirror
        // the in-process `policy_with_call` precedence (period → sync_window →
        // unbounded_sync → external → single-trigger). The node-level trigger
        // attrs are mutually exclusive (enforced by
        // `validate.rs::validate_trigger_inference`), so this ordering is
        // belt-and-suspenders. Host shape: `PolicyJson.unbounded_sync`
        // (`{"unbounded_sync":true}` — see `graph::node::PolicyJson`).
        r#","policy":{"unbounded_sync":true}"#.to_string()
    } else if node_level.external {
        r#","policy":{"external":true}"#.to_string()
    } else if let Some(name) = single_trigger_input_name {
        // Nested-object shape mirrors the parser
        // contract in `cerulion_core::graph::node::parse_info_json`.
        // `json_escape` handles names with embedded quotes / backslashes
        // /  control bytes — Rust idents don't allow those, but the
        // macro's input survives a `syn::parse` that is more permissive
        // about whitespace than a strict ident, and we don't want a
        // weird ident smuggled through `quote!` to break the JSON.
        format!(
            r#","policy":{{"data_trigger":{{"input_name":"{}"}}}}"#,
            json_escape(name)
        )
    } else {
        String::new()
    };
    // ABI v6: top-level node-level QoS knobs.
    // `tick_within_ms` (per-node tick-execution budget) and `throttle_ms`
    // (producer rate cap) are emitted ONLY when declared — conditional
    // presence, exactly like `policy` above. Absent → the parser's
    // `#[serde(default)]` yields `None`. Both fragments lead with a comma
    // so they splice into the top-level object after `outputs`.
    let mut qos_json = String::new();
    if let Some(ms) = node_level.tick_within_ms {
        qos_json.push_str(&format!(r#","tick_within_ms":{ms}"#));
    }
    if let Some(ms) = node_level.throttle_ms {
        qos_json.push_str(&format!(r#","throttle_ms":{ms}"#));
    }

    // OPTIONAL external-source FFI export, emitted ONLY for
    // `#[cerulion_node(external)]` nodes. The host (`DylibNodeEntry`) resolves
    // `cerulion_node_external_source` best-effort (absent = non-external / older
    // cdylib → `external_source()` is `None`), so this is ADDITIVE and does NOT
    // bump `CERULION_ABI_VERSION`. It queries the node's `NodeEntry::external_source`
    // and collapses the result to a kind code + optional raw fd (see the
    // `EXTERNAL_SOURCE_KIND_*` consts in `cerulion_core::graph::node`):
    //   1 = device fd (writes the raw fd into `*out_fd`)
    //   2 = doorbell fd (a `Blocking` closure can't cross the C ABI, so the
    //       cdylib spawns a pipe-backed helper here and returns the READ end via
    //       `*out_fd`; the host drains it to EAGAIN each sweep)
    //   3 = HostDriven (`*out_fd` untouched)
    //  -1 = error (LAST_ERROR set)
    let external_source_ffi: TokenStream = if node_level.external {
        quote! {
            // Query this external node's `ExternalSource` and
            // collapse it to the C-ABI kind code. `Blocking` (tier 2) can't cross
            // the FFI as a closure, so it is materialized here into a pipe +
            // detached helper thread (via `cerulion_core`), returning the READ end.
            #[no_mangle]
            pub extern "C" fn cerulion_node_external_source(
                handle: u64,
                out_fd: *mut i64,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    use ::cerulion_core::graph::node::NodeEntry;
                    if out_fd.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_external_source: out_fd pointer was null",
                        ));
                        return ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR;
                    }
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_external_source: NODES mutex poisoned",
                            ));
                            return ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            // NODES poison containment: drive
                            // the user's `external_source()` under an INNER
                            // `catch_unwind` so a panicking override does NOT unwind
                            // through the LIVE `NODES` MutexGuard (`guard`) — an
                            // unwind through it would POISON `NODES` and brick every
                            // node of this cdylib in-process. `external_source` is a
                            // one-shot pre-binding query; unlike `tick`, torn node
                            // state is no concern — on panic the node simply stays
                            // inert and never ticks. The guard then drops NORMALLY
                            // (no poison); the OUTER catch_unwind stays the backstop.
                            let source = match ::std::panic::catch_unwind(
                                ::std::panic::AssertUnwindSafe(|| node.external_source()),
                            ) {
                                ::std::result::Result::Ok(s) => s,
                                ::std::result::Result::Err(_) => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_external_source: external_source \
                                         panicked; node has no external source",
                                    ));
                                    return ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR;
                                }
                            };
                            match source {
                                ::std::option::Option::Some(
                                    ::cerulion_core::graph::node::ExternalSource::Fd(raw),
                                ) => {
                                    // SAFETY: out_fd is a non-null, aligned, host-owned
                                    // i64 slot (checked above); the host reads it back
                                    // immediately after this call returns 1.
                                    unsafe {
                                        *out_fd = raw as i64;
                                    }
                                    ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_DEVICE_FD
                                }
                                ::std::option::Option::Some(
                                    ::cerulion_core::graph::node::ExternalSource::Blocking(closure),
                                ) => {
                                    // Tier-2 collapse: hand the closure to cerulion_core,
                                    // which spawns the pipe-backed doorbell helper and
                                    // returns Some(READ end), or None on pipe failure.
                                    match ::cerulion_core::graph::node::spawn_cdylib_blocking_doorbell(
                                        closure,
                                    ) {
                                        ::std::option::Option::Some(read_fd) => {
                                            // SAFETY: as above — non-null aligned i64 slot.
                                            unsafe {
                                                *out_fd = read_fd as i64;
                                            }
                                            ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_DOORBELL_FD
                                        }
                                        ::std::option::Option::None => {
                                            // The pipe(2) failure was logged inside
                                            // cerulion_core against the CDYLIB's own
                                            // tracing subscriber. The stderr stopgap installs that
                                            // subscriber at `cerulion_node_init` (which
                                            // runs before this external-source query), so
                                            // the log lands on stderr — but it is
                                            // still subject to RUST_LOG filtering. Carry
                                            // the cause in LAST_ERROR: that is the reliable
                                            // STRUCTURED channel the host reads after a
                                            // non-zero return, independent of log level.
                                            __cer_set_last_error(::std::string::String::from(
                                                "cerulion_node_external_source: failed to create the \
                                                 Blocking doorbell pipe/helper (pipe(2) failed); node \
                                                 stays inert",
                                            ));
                                            ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR
                                        }
                                    }
                                }
                                ::std::option::Option::Some(
                                    ::cerulion_core::graph::node::ExternalSource::HostDriven,
                                ) => ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_HOST_DRIVEN,
                                // `ExternalSource` is `#[non_exhaustive]` — a future
                                // variant this cdylib's cerulion_core doesn't know how to
                                // collapse fails loudly rather than mis-binding.
                                ::std::option::Option::Some(_) => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_external_source: unsupported ExternalSource \
                                         variant for this cerulion_core version",
                                    ));
                                    ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR
                                }
                                // The generated external override always returns
                                // `Some(..)`; `None` here means a hand-rolled entry
                                // returned it — treat as an error (not silently inert).
                                ::std::option::Option::None => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_external_source: external node returned no \
                                         ExternalSource (None)",
                                    ));
                                    ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR
                                }
                            }
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_external_source: handle {} not found", handle,
                            ));
                            ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_external_source: panic caught by catch_unwind",
                        ));
                        ::cerulion_core::graph::node::EXTERNAL_SOURCE_KIND_ERROR
                    }
                }
            }
        }
    } else {
        TokenStream::new()
    };

    quote! {
        #[allow(unexpected_cfgs)]
        #[cfg(feature = "cdylib")]
        mod __cerulion_cdylib {
            use super::*;

            // Info JSON is built at first call
            // via `OnceLock` because each `#[output]` port's
            // `schema_hash` and `max_slice_len_default` come from
            // `<T as ShmMessage>` trait consts that are not known at
            // proc-macro expansion time. The `OnceLock` initializes
            // exactly once and the resulting `CString` is owned by
            // the static — `cerulion_node_info()` returns its raw
            // pointer, valid for the lifetime of the cdylib.
            static INFO_JSON: ::std::sync::OnceLock<::std::ffi::CString> =
                ::std::sync::OnceLock::new();

            fn __cer_build_info_json() -> ::std::ffi::CString {
                // Build the inputs array at first call, like the
                // outputs below — each input's `schema_hash` is the port
                // type's `<T as ShmMessage>::SCHEMA_HASH` trait const, not
                // known at proc-macro expansion time. `#input_key_suffixes`
                // is the expansion-time-known conditional-QoS fragment
                // (leading-comma, may be empty). Empty `inputs` → `[]`.
                let mut inputs = ::std::string::String::new();
                inputs.push('[');
                let mut first_input = true;
                #(
                    if !first_input { inputs.push(','); }
                    first_input = false;
                    inputs.push_str(&::std::format!(
                        r#"{{"name":"{}","schema_hash":{}{}}}"#,
                        #input_field_names,
                        <#input_field_types as ::cerulion_core::message::ShmMessage>::SCHEMA_HASH,
                        #input_key_suffixes,
                    ));
                )*
                inputs.push(']');
                let mut outputs = ::std::string::String::new();
                outputs.push('[');
                let mut first = true;
                #(
                    if !first { outputs.push(','); }
                    first = false;
                    let max_slice_len_default =
                        match <#output_field_types as ::cerulion_core::message::ShmMessage>::MAX_SLICE_LEN {
                            ::std::option::Option::Some(n) =>
                                ::std::format!("{}", n),
                            ::std::option::Option::None =>
                                ::std::string::String::from("null"),
                        };
                    // `wire_fixed_size` is the port type's
                    // `<T as ShmMessage>::WIRE_FIXED_SIZE`, emitted
                    // UNCONDITIONALLY on every output object (a trait
                    // const, so — like `schema_hash` — it is not known at
                    // proc-macro expansion time and is formatted here).
                    // It is what lets `graph run --record` stamp a bag
                    // channel's `SchemaDescriptor` from the NODE rather
                    // than from a workspace `schemas/` file that may
                    // describe a different layout, or none at all. The
                    // key is ADDITIVE and carries no ABI bump: a host
                    // that has not learned it defaults the field to "no
                    // claim" and sizes from the file exactly as before.
                    outputs.push_str(&::std::format!(
                        r#"{{"name":"{}","schema_hash":{},"max_slice_len_default":{},"promise_within_ms":{},"wire_fixed_size":{}}}"#,
                        #output_field_names,
                        <#output_field_types as ::cerulion_core::message::ShmMessage>::SCHEMA_HASH,
                        max_slice_len_default,
                        #output_promise_within,
                        <#output_field_types as ::cerulion_core::message::ShmMessage>::WIRE_FIXED_SIZE,
                    ));
                )*
                outputs.push(']');
                // ABI v6: `#qos_json` splices the
                // node-level `tick_within_ms` / `throttle_ms` (each a
                // leading-comma fragment, present only when declared)
                // after `outputs` and before `policy`.
                let json = ::std::format!(
                    r#"{{"inputs":{},"outputs":{}{}{}}}"#,
                    inputs,
                    outputs,
                    #qos_json,
                    #policy_json,
                );
                // If the JSON contains an
                // embedded NUL byte (only possible for pathological
                // schema names) the CString construction fails. Emit
                // deliberately invalid JSON so `parse_info_json` on
                // the host returns Err and surfaces a clear diagnostic
                // — better than silently returning empty meta and
                // ghost-loading the node.
                ::std::ffi::CString::new(json).unwrap_or_else(|_| {
                    ::std::ffi::CString::new(
                        r#"<<INVALID JSON: cerulion_node_info() encountered an embedded NUL byte; \
    report at https://github.com/cerulion-inc/cerulion/issues>>"#,
                    )
                    .expect("static literal contains no nul bytes")
                })
            }

            fn __cer_info_bytes() -> &'static [u8] {
                INFO_JSON.get_or_init(__cer_build_info_json).as_bytes_with_nul()
            }

            static NEXT_HANDLE: ::std::sync::atomic::AtomicU64 =
                ::std::sync::atomic::AtomicU64::new(1); // 0 = error sentinel

            static NODES: ::std::sync::Mutex<Option<::std::collections::HashMap<u64, #entry_name>>> =
                ::std::sync::Mutex::new(None);

            // Per-cdylib thread-local
            // for the most recent FFI error message. Populated by tick /
            // shutdown / init when they return non-success. The host
            // pulls it via `cerulion_take_last_error()` immediately
            // after the offending FFI call and frees it via
            // `cerulion_free_error()`.
            //
            // Thread-local (rather than a global Mutex) because the
            // graph runtime is single-threaded per node — a separate
            // tick on a different thread can't clobber another thread's
            // unread error. This also avoids any lock contention on
            // the hot path when there's no error.
            ::std::thread_local! {
                static LAST_ERROR: ::std::cell::RefCell<::std::option::Option<::std::ffi::CString>>
                    = const { ::std::cell::RefCell::new(::std::option::Option::None) };
            }

            fn __cer_set_last_error(msg: ::std::string::String) {
                let cstring = ::std::ffi::CString::new(msg.replace('\0', "\\0"))
                    .unwrap_or_else(|_| ::std::ffi::CString::new("error message contained nul byte").unwrap());
                LAST_ERROR.with(|cell| {
                    *cell.borrow_mut() = ::std::option::Option::Some(cstring);
                });
            }

            #[no_mangle]
            pub extern "C" fn cerulion_abi_version() -> u32 {
                ::cerulion_core::CERULION_ABI_VERSION
            }

            // ABI v22: reports the rustc that compiled THIS cdylib, so the
            // host can refuse a load where the two sides' compilers disagree
            // on how a `repr(Rust)` type crossing the FFI (for example
            // `Option<FrozenSlot>`) is laid out, a class the ABI version above
            // cannot see, since two different rustc releases can agree on
            // every struct size and offset and still encode `None`
            // differently. See `cerulion_core::rustc_fingerprint`.
            #[no_mangle]
            pub extern "C" fn cerulion_rustc_fingerprint() -> *const ::std::ffi::c_char {
                ::cerulion_core::rustc_fingerprint_cstr()
            }

            #[no_mangle]
            pub extern "C" fn cerulion_node_info() -> *const ::std::ffi::c_char {
                // Wrap the
                // first-call info-JSON build in `catch_unwind`. The
                // `format!` + `CString::new` path is panic-free in
                // practice (NUL bytes are handled via `unwrap_or_else`
                // inside `__cer_build_info_json`), but an unwind across
                // the `extern "C"` boundary is UB. On panic, return null
                // — the host (`DylibNodeEntry::info`) already treats a
                // null pointer as "use empty NodeInfo defaults" with a
                // loud `error!`, so a panicking build degrades to the
                // same ghost-load path as a null-returning cdylib rather
                // than aborting the process. Mirrors the
                // `catch_unwind` wrappers on init/tick/shutdown below.
                let result = ::std::panic::catch_unwind(|| {
                    __cer_info_bytes().as_ptr() as *const ::std::ffi::c_char
                });
                match result {
                    ::std::result::Result::Ok(ptr) => ptr,
                    ::std::result::Result::Err(_) => ::std::ptr::null(),
                }
            }

            // Take the most recent
            // error message off the thread-local. Returns null if no
            // error is buffered. Caller MUST pass the returned pointer
            // back to `cerulion_free_error` to release it.
            #[no_mangle]
            pub extern "C" fn cerulion_take_last_error() -> *mut ::std::ffi::c_char {
                LAST_ERROR.with(|cell| match cell.borrow_mut().take() {
                    ::std::option::Option::Some(cstr) => cstr.into_raw(),
                    ::std::option::Option::None => ::std::ptr::null_mut(),
                })
            }

            // Free a string returned
            // by `cerulion_take_last_error`. Idempotent on null. Must be
            // called from this cdylib (CString allocator pairing).
            //
            // Marked `unsafe` because the function
            // dereferences a raw pointer arg (`CString::from_raw`); per
            // `clippy::not_unsafe_ptr_arg_deref` (deny-by-default), a
            // function that does so MUST be `unsafe fn`. Macro-expanded
            // code skips clippy by default, so the lint does not enforce it;
            // the safety contract is "ptr must be one
            // previously returned by `cerulion_take_last_error` from
            // this same cdylib", and `unsafe` makes it explicit. The host's
            // `Symbol<unsafe extern "C" fn(*mut c_char)>` accepts the
            // unsafe form without any host-side change.
            #[no_mangle]
            pub unsafe extern "C" fn cerulion_free_error(ptr: *mut ::std::ffi::c_char) {
                if ptr.is_null() {
                    return;
                }
                // SAFETY: pointer originated from `CString::into_raw` in
                // `cerulion_take_last_error`. Reclaiming via `from_raw`
                // returns ownership and the CString drops here.
                let _ = ::std::ffi::CString::from_raw(ptr);
            }

            // FFI error codes (for tick/shutdown i32 returns):
            //   0 = success
            //   1 = logic error (init/tick/shutdown returned Err)
            //   2 = panic (caught by catch_unwind)
            //   3 = mutex poisoned
            //   4 = handle not found
            //
            // For codes 1-4 the cdylib also stashes a human-readable
            // error string in LAST_ERROR; the host calls
            // `cerulion_take_last_error()` after a non-zero return to
            // surface the actual NodeError message rather than the
            // generic class label.
            //
            // Init returns u64 handle (0 = failure; LAST_ERROR also set).

            #[no_mangle]
            pub extern "C" fn cerulion_node_init(ctx_ptr: *mut ::cerulion_core::graph::node::NodeContext) -> u64 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    use ::cerulion_core::graph::node::NodeEntry;

                    if ctx_ptr.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_init: NodeContext pointer was null",
                        ));
                        return 0_u64;
                    }

                    // Safety: caller (DylibNodeEntry::init) transferred ownership via Box::into_raw.
                    // We always consume the pointer here — on both success and failure paths
                    // the Box drops naturally (either stored in entry or dropped on error).
                    let ctx = unsafe { *Box::from_raw(ctx_ptr) };

                    // STOPGAP: install a cdylib-local stderr tracing
                    // subscriber so node-side `tracing` events (including
                    // cerulion_core's loud OutputProxy discard error) become
                    // host-visible. This cdylib statically links its OWN tracing
                    // static, uninitialized by the host — without this, node-side
                    // logs dispatch to a no-op. RUST_LOG is read from the frozen
                    // env snapshot (determinism), empty ⇒ unset ⇒ "info".
                    {
                        let __cer_rust_log = ctx.env_str("RUST_LOG", "");
                        ::cerulion_core::graph::node::install_cdylib_stderr_tracing(
                            if __cer_rust_log.is_empty() {
                                ::std::option::Option::None
                            } else {
                                ::std::option::Option::Some(__cer_rust_log.as_str())
                            },
                        );
                    }

                    // This cdylib statically links its OWN copy of
                    // `iceoryx2` + `iceoryx2-log`, so it has its OWN
                    // `LOG_LEVEL` static — the host's `set_log_level` does
                    // NOTHING for it and it stays at iceoryx2's crate default
                    // (`Info`), which lets every iceoryx2 `warn!` emitted by
                    // NODE-side transport code print regardless of
                    // `IOX2_LOG_LEVEL`. That made a documented
                    // knob inert while ~2500 `FailedToDeliverSignal` warnings/s
                    // (~5 MB/s) filled the disk. Set this copy's level from the
                    // FROZEN env snapshot (never live `std::env` — replay
                    // determinism, exactly like RUST_LOG above); empty ⇒ unset
                    // ⇒ the Cerulion default (`error`).
                    {
                        let __cer_iox2_log = ctx.env_str("IOX2_LOG_LEVEL", "");
                        ::cerulion_core::iceoryx_logger::init_iceoryx_log_level(
                            if __cer_iox2_log.is_empty() {
                                ::std::option::Option::None
                            } else {
                                ::std::option::Option::Some(__cer_iox2_log.as_str())
                            },
                        );
                    }

                    let mut entry = #entry_name::with_state(Default::default());
                    if let ::std::result::Result::Err(e) = entry.init(ctx) {
                        __cer_set_last_error(::std::format!("init failed: {}", e));
                        return 0;
                    }

                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_init: NODES mutex poisoned",
                            ));
                            return 0;
                        }
                    };
                    let map = guard.get_or_insert_with(::std::collections::HashMap::new);
                    let handle = NEXT_HANDLE.fetch_add(1, ::std::sync::atomic::Ordering::Relaxed);
                    map.insert(handle, entry);
                    handle
                }));
                match result {
                    Ok(handle) => handle,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_init: panic caught by catch_unwind",
                        ));
                        0
                    }
                }
            }

            #[no_mangle]
            pub extern "C" fn cerulion_node_tick(handle: u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    use ::cerulion_core::graph::node::NodeEntry;
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_tick: NODES mutex poisoned",
                            ));
                            return 3_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => match node.tick() {
                            ::std::result::Result::Ok(()) => 0,
                            ::std::result::Result::Err(e) => {
                                __cer_set_last_error(::std::format!("tick failed: {}", e));
                                1
                            }
                        },
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_tick: handle {} not found", handle,
                            ));
                            4
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_tick: panic caught by catch_unwind",
                        ));
                        2
                    }
                }
            }

            // Mirror of `cerulion_node_tick` that
            // services quiescent late joiners. `pump_history()` returns
            // `()`, so the success path returns 0; the error codes match
            // tick exactly (3 NODES poisoned / 4 handle not found /
            // 2 panic).
            #[no_mangle]
            pub extern "C" fn cerulion_node_pump_history(handle: u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    use ::cerulion_core::graph::node::NodeEntry;
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_pump_history: NODES mutex poisoned",
                            ));
                            return 3_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            node.pump_history();
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_pump_history: handle {} not found", handle,
                            ));
                            4
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_pump_history: panic caught by catch_unwind",
                        ));
                        2
                    }
                }
            }

            #[no_mangle]
            pub extern "C" fn cerulion_node_shutdown(handle: u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    use ::cerulion_core::graph::node::NodeEntry;
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_shutdown: NODES mutex poisoned",
                            ));
                            return 3_i32;
                        }
                    };
                    // A missing handle returns
                    // code 4 + LAST_ERROR (mirrors cerulion_node_tick).
                    // Returning 0 instead would make a buggy
                    // loader sending a stale or never-registered handle
                    // look like a successful shutdown.
                    let mut node = match guard.as_mut().and_then(|m| m.remove(&handle)) {
                        Some(n) => n,
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_shutdown: handle {} not found (already shut down or never registered)",
                                handle,
                            ));
                            return 4_i32;
                        }
                    };
                    if let ::std::result::Result::Err(e) = node.shutdown() {
                        __cer_set_last_error(::std::format!("shutdown failed: {}", e));
                        return 1_i32;
                    }
                    0
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_shutdown: panic caught by catch_unwind",
                        ));
                        2
                    }
                }
            }

            // OPTIONAL snapshot FFI (cross-step HOLD of non-trigger
            // latest-value `#[input]`s for cdylib nodes). The host
            // (`DylibNodeEntry`) resolves both symbols as OPTIONAL — a
            // cdylib that doesn't export them still loads and
            // behaves as before (its non-trigger inputs read live), so these
            // are ADDITIVE and do NOT bump `CERULION_ABI_VERSION`. The same
            // precedent governs the `cerulion_node_drain_trigger_input`
            // below (unified trigger drain): optional, additive, no ABI bump —
            // a symbol-less cdylib keeps the dual-subscriber drain. It also
            // governs `cerulion_node_refill_trigger_input` and
            // `cerulion_node_sync_head_op`.
            //
            // A symbol being bump-free is NOT the same as its FEATURE being
            // bump-free, and the head-op export is the case that makes the distinction
            // worth writing down: the head-op symbol above rides the
            // presence-is-capability precedent and bumps nothing on its own,
            // while the feature it belongs to DOES bump `CERULION_ABI_VERSION`
            // 14 -> 15 — a bump earned by the `next_head` field added to
            // `CerulionSubscriber`, which changes the LAYOUT of a type the
            // `NodeContext` owns across the `init()` boundary. ONE bump covers
            // the whole feature; the optional symbol rides it rather than
            // adding a second.
            //
            // FFI return codes (negative = failure; the host pulls LAST_ERROR):
            //   0  = success
            //  -1  = NODES mutex poisoned
            //  -2  = handle not found
            //  -3  = name(s) not valid UTF-8 (set_snapshot_inputs +
            //        drain_trigger_input)
            //  -4  = panic caught by catch_unwind
            //
            // `cerulion_node_set_snapshot_inputs`: store (ONCE, at the host's
            // first call) the runtime-classified non-trigger input NAMES on the
            // node's wrapper, so the per-step `cerulion_node_snapshot_inputs`
            // below allocates nothing.
            #[no_mangle]
            pub extern "C" fn cerulion_node_set_snapshot_inputs(
                handle: u64,
                names_ptr: *const u8,
                names_len: usize,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    // Parse the `\n`-joined UTF-8 names into owned Strings. The
                    // host marshals an EMPTY string for an empty name set, which
                    // must become an EMPTY vec (NOT `[""]`, which `"".split('\n')`
                    // yields), so split only the non-empty case.
                    let names: ::std::vec::Vec<::std::string::String> = if names_len == 0 {
                        ::std::vec::Vec::new()
                    } else {
                        // SAFETY: the host (DylibNodeEntry::snapshot_inputs)
                        // passes a pointer to a live, immutable byte slice of
                        // exactly `names_len` bytes, valid for this call; we only
                        // read it and copy out owned Strings, never retaining the
                        // pointer.
                        let bytes = unsafe {
                            ::std::slice::from_raw_parts(names_ptr, names_len)
                        };
                        match ::std::str::from_utf8(bytes) {
                            ::std::result::Result::Ok(s) => s
                                .split('\n')
                                .map(::std::string::String::from)
                                .collect(),
                            ::std::result::Result::Err(_) => {
                                __cer_set_last_error(::std::string::String::from(
                                    "cerulion_node_set_snapshot_inputs: names were not valid UTF-8",
                                ));
                                return -3_i32;
                            }
                        }
                    };
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_set_snapshot_inputs: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            node.__snapshot_input_names = names;
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_set_snapshot_inputs: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_set_snapshot_inputs: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // `cerulion_node_snapshot_inputs`: forward the per-step freeze to
            // the node's NodeContext using the names stored by
            // `set_snapshot_inputs`. The SAME call the macro's in-process
            // `NodeEntry::snapshot_inputs` makes. Allocates nothing (the names
            // were captured once) — safe to call every step.
            #[no_mangle]
            pub extern "C" fn cerulion_node_snapshot_inputs(handle: u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_snapshot_inputs: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            // Disjoint field borrows: bind the shared borrow of
                            // `__snapshot_input_names` first, then take the mutable
                            // borrow of the disjoint `context` field — NLL allows
                            // simultaneous borrows of distinct struct fields.
                            let names = node.__snapshot_input_names.as_slice();
                            if let Some(ctx) = node.context.as_mut() {
                                ctx.snapshot_inputs(names);
                            }
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_snapshot_inputs: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_snapshot_inputs: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // OPTIONAL unified-trigger-drain FFI. Symbol
            // PRESENCE is the capability gate (`NodeEntry::unifies_trigger_drain`
            // on the host's `DylibNodeEntry`) that lets the runtime elide the
            // separate trigger-drain subscriber for a cdylib data-trigger input
            // (one iceoryx2 receive per hop, the unified-drain win, extended across the
            // FFI). Only MACRO-generated cdylibs export it, and their input
            // reads are generated `try_view` — the one read path the unified
            // drain's frozen slot serves — so the capability-by-symbol gate is
            // also the structural READ-PATH CONTRACT enforcement for the
            // production surface (a raw-FFI cdylib free to `try_receive` lacks
            // the symbol and keeps the dual-subscriber path).
            //
            // Out-params are written ONLY on success (return 0): `out_popped` =
            // frames drained off the body subscriber, `out_latest_ts` +
            // `out_has_ts` (1/0) = the latest wire timestamp, if any. Zero-alloc
            // on the happy path: the name is read as a BORROWED `&str` (no
            // String copy — this runs on the per-step drain path) and
            // `NodeContext::drain_trigger_input` allocates nothing.
            #[no_mangle]
            pub extern "C" fn cerulion_node_drain_trigger_input(
                handle: u64,
                name_ptr: *const u8,
                name_len: usize,
                out_popped: *mut u64,
                out_latest_ts: *mut u64,
                out_has_ts: *mut i32,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    // SAFETY: the host (`DylibNodeEntry::drain_trigger_input`)
                    // passes a pointer to a live, immutable byte slice of exactly
                    // `name_len` bytes, valid for this call; we only borrow it
                    // (no copy) and never retain the pointer. `from_raw_parts`
                    // with len 0 and a dangling-but-aligned ptr is sound, and the
                    // host always passes `str::as_ptr()` (non-null).
                    let bytes = unsafe {
                        ::std::slice::from_raw_parts(name_ptr, name_len)
                    };
                    let name = match ::std::str::from_utf8(bytes) {
                        ::std::result::Result::Ok(s) => s,
                        ::std::result::Result::Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_drain_trigger_input: name was not valid UTF-8",
                            ));
                            return -3_i32;
                        }
                    };
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_drain_trigger_input: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            let (popped, latest_ts) = match node.context.as_mut() {
                                Some(ctx) => ctx.drain_trigger_input(name),
                                // Not yet init'd: nothing to drain — success
                                // with zero, mirroring `cerulion_node_snapshot_
                                // inputs`' silent no-context arm.
                                None => (0_u64, ::core::option::Option::None),
                            };
                            // SAFETY: the host passes valid, writable
                            // out-pointers (its own stack slots) for this call.
                            unsafe {
                                *out_popped = popped;
                                match latest_ts {
                                    ::core::option::Option::Some(ts) => {
                                        *out_latest_ts = ts;
                                        *out_has_ts = 1;
                                    }
                                    ::core::option::Option::None => {
                                        *out_latest_ts = 0;
                                        *out_has_ts = 0;
                                    }
                                }
                            }
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_drain_trigger_input: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_drain_trigger_input: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // The OPTIONAL between-fires REFILL export — the drain
            // export's twin, ADDITIVE and resolved best-effort by the host, so
            // `CERULION_ABI_VERSION` is deliberately NOT bumped and a
            // earlier cdylib (which exports the drain and not this) keeps
            // loading, keeps its Unified binding, and simply has its bursts
            // served one frame per step.
            //
            // It cannot ride the drain export, because the two sites disagree
            // about what an UNSERVED FROZEN HEAD means: the boundary drain
            // RE-OFFERS it (a frame still owed a fire — Principle #6), while a
            // refill must report "nothing new" (the fire it follows did not
            // consume the head, so re-offering would fire the node a second
            // time on one frame — up to the per-step cap, every step).
            //
            // Same contract as the drain export in every other respect: the
            // name is BORROWED (no copy — this runs per fire of a burst),
            // out-params are written ONLY on success (return 0), and
            // `NodeContext::refill_trigger_input` allocates nothing.
            #[no_mangle]
            pub extern "C" fn cerulion_node_refill_trigger_input(
                handle: u64,
                name_ptr: *const u8,
                name_len: usize,
                out_popped: *mut u64,
                out_latest_ts: *mut u64,
                out_has_ts: *mut i32,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    // SAFETY: the host (`DylibNodeEntry::refill_trigger_input`)
                    // passes a pointer to a live, immutable byte slice of exactly
                    // `name_len` bytes, valid for this call; we only borrow it
                    // (no copy) and never retain the pointer. `from_raw_parts`
                    // with len 0 and a dangling-but-aligned ptr is sound, and the
                    // host always passes `str::as_ptr()` (non-null).
                    let bytes = unsafe {
                        ::std::slice::from_raw_parts(name_ptr, name_len)
                    };
                    let name = match ::std::str::from_utf8(bytes) {
                        ::std::result::Result::Ok(s) => s,
                        ::std::result::Result::Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_refill_trigger_input: name was not valid UTF-8",
                            ));
                            return -3_i32;
                        }
                    };
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_refill_trigger_input: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            let (popped, latest_ts) = match node.context.as_mut() {
                                Some(ctx) => ctx.refill_trigger_input(name),
                                // Not yet init'd: nothing to refill — success
                                // with zero, mirroring the drain export's
                                // silent no-context arm.
                                None => (0_u64, ::core::option::Option::None),
                            };
                            // SAFETY: the host passes valid, writable
                            // out-pointers (its own stack slots) for this call.
                            unsafe {
                                *out_popped = popped;
                                match latest_ts {
                                    ::core::option::Option::Some(ts) => {
                                        *out_latest_ts = ts;
                                        *out_has_ts = 1;
                                    }
                                    ::core::option::Option::None => {
                                        *out_latest_ts = 0;
                                        *out_has_ts = 0;
                                    }
                                }
                            }
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_refill_trigger_input: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_refill_trigger_input: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // The OPTIONAL per-set Sync HEAD-OP export. ADDITIVE and
            // resolved best-effort by the host, so symbol PRESENCE is the
            // capability (`NodeEntry::supports_sync_head_ops` on the host's
            // `DylibNodeEntry`) exactly as it is for the drain symbol
            // above — a raw-FFI or earlier cdylib keeps loading and simply
            // reports that it cannot serve the per-set matcher, which is what
            // lets the scheduler degrade that node rather than refuse it.
            //
            // Presence-as-capability is also the READ-PATH CONTRACT here, and
            // that is the load-bearing half: the ops promote a chosen frame
            // into the frozen slot and `Void` makes a RESTORED head read as
            // "no frame", both of which are only true for the generated
            // `try_view`. Only MACRO-generated cdylibs export this symbol, and
            // their reads ARE that `try_view`; a raw-FFI cdylib free to call
            // `try_receive` lacks the symbol and is never handed a head.
            //
            // WHY THE TWO FILL OPS ARE NOT HERE. `SyncHeadOp::FillBoundary`
            // and `SyncHeadOp::FillRefill` deliberately do NOT cross this
            // symbol: filling a head at a level boundary and filling it
            // between two fires of one step already have a home — the
            // `cerulion_node_drain_trigger_input` and the
            // `cerulion_node_refill_trigger_input` exports above, which draw
            // exactly that distinction (a boundary RE-OFFERS an unserved
            // frozen head per Principle #6; a refill must report "nothing
            // new"). A second crossing for the same two questions would be two
            // FFI paths that must agree about one frozen head — a class of
            // duplication that drifts — so this symbol carries only
            // the four ops that have no existing crossing. An op code outside
            // that set is an ERROR, never a silent no-op: a host that sends
            // one is asking for something this ABI does not carry, and
            // answering `Nothing` would let it read a missing capability as
            // evidence of scarcity.
            //
            // ABI: this symbol does NOT bump `CERULION_ABI_VERSION` on its own
            // (the symbol-presence-is-capability precedent). That is
            // NOT the same as the feature being bump-free: this feature DOES bump
            // 14 -> 15, and the bump is already taken by the `next_head` field
            // added to `CerulionSubscriber`, which is a LAYOUT change to a type
            // the `NodeContext` owns across the `init()` boundary. ONE bump
            // covers the whole feature; this export rides it.
            //
            // OP CODES (`op`) and ANSWER KINDS (`*out_kind`) are the SHARED
            // `SYNC_HEAD_OP_*` / `SYNC_OP_ANSWER_*` consts in
            // `cerulion_core::graph::node`, reached by PATH below and never
            // re-spelled as literals here — the host resolver decodes with the
            // same consts, which is what makes their "the two spellings of one
            // wire cannot drift" claim true of the emitting side as well:
            //   op:       PROBE_NEXT (0) · PEEK_NEXT (1) · ADVANCE (2) · VOID (3)
            //   out_kind: NOTHING (0) · PRESENT (1) · HEAD (2) · STAMP (3)
            //
            // `*out_ts` carries the stamp for HEAD and STAMP; for NOTHING and
            // PRESENT the host is entitled to treat it as untouched and never
            // reads it. We write 0 there anyway — a strict superset of the
            // contract, so a host that neglected to initialize its own slot
            // reads a defined value rather than whatever was on its stack.
            //
            // RETURN CODES. A NONZERO return means the op FAILED and the
            // out-params were NOT written. -1..-4 are the drain export's codes
            // with the drain export's meanings:
            //   0  = success
            //  -1  = NODES mutex poisoned
            //  -2  = handle not found
            //  -3  = name was not valid UTF-8
            //  -4  = panic caught by catch_unwind
            //  -5  = unknown op code (this ABI does not carry it)
            //  -6  = the op itself answered `SyncOpAnswer::Failed`
            //  -7  = a null out-param
            //  -8  = the name argument is not a readable slice (null pointer,
            //        or a length past `isize::MAX`)
            //
            // -6 exists because `Failed` is a FIRST-CLASS answer of
            // `SyncOpAnswer` (a poisoned lock deeper down, an iceoryx2 error, a
            // not-yet-init'd node) and the four answer kinds above have no room
            // for it. It must not be smuggled in as `Nothing`: the R-Fail
            // policy that reads it is POSITION-AWARE, and a failure laundered
            // into `Nothing` would VOUCH FOR SCARCITY on an input nobody
            // verified. -7 is checked UP FRONT, before the lock and before the
            // op runs, so a caller passing null is refused deterministically
            // rather than only on the arms that happen to write a stamp.
            //
            // -8 is the same rule applied to the IN-param, and it is checked
            // FIRST — before -5's op-code match and before -7 — because the
            // name is the only argument this function DEREFERENCES.
            // `slice::from_raw_parts` requires a NON-NULL, aligned pointer
            // EVEN AT LENGTH ZERO, so calling it on a null `name_ptr` is
            // undefined behaviour outright rather than merely a bad read: a
            // debug build ABORTS on the precondition and a release build
            // compiles that check out and reads from address 0. Refusing it
            // BEFORE the deref is the whole of the guard (the established
            // precedent: validate, then answer with a dedicated code). u8's
            // alignment is 1, so non-null IS the entire remaining
            // precondition once this arm has run.
            //
            // It is its OWN code rather than -7's because the two are
            // different caller bugs with different fixes — a null out-param
            // means the host forgot its own stack slots, a null name means it
            // has no port to ask about — the same argument rmw's
            // `topic=`/`service=` split makes for keeping such distinctions in
            // the vocabulary. It is deliberately NOT laundered into an EMPTY
            // NAME either: rmw's hand-built empty slice exists so a caller
            // mistake still reaches a LATCHED, COUNTED reject arm, and this
            // export has neither a latch nor a counter — the return code is
            // the whole diagnostic, so an empty name resolving to "no such
            // port" (-6 `Failed`) would report a wiring desync as an ordinary
            // transport failure and hide the caller bug behind R-Fail. A
            // NON-NULL pointer with `name_len == 0` still takes that ordinary
            // path: it is SOUND to borrow, and an empty name really is a
            // lookup that finds nothing.
            //
            // Zero-alloc on the happy path, same as the two exports above: the
            // name is read as a BORROWED `&str` (no String copy — this runs per
            // alignment pass, per input) and `NodeContext::sync_head_op`
            // allocates nothing.
            #[no_mangle]
            pub extern "C" fn cerulion_node_sync_head_op(
                handle: u64,
                name_ptr: *const u8,
                name_len: usize,
                op: u32,
                out_ts: *mut u64,
                out_kind: *mut i32,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    // Refused FIRST, before the deref below: a null `name_ptr`
                    // is UNDEFINED BEHAVIOUR to hand to `from_raw_parts` even
                    // at length 0, so it cannot be checked afterwards.
                    if name_ptr.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_sync_head_op: name_ptr was null",
                        ));
                        return -8_i32;
                    }
                    // The other half of the same precondition: `from_raw_parts`
                    // also requires the slice's total size to fit in `isize`.
                    // Same code, because it is the same caller mistake — the
                    // name argument is not a readable slice — and the message
                    // says which half it was.
                    if name_len > (::std::isize::MAX as usize) {
                        __cer_set_last_error(::std::format!(
                            "cerulion_node_sync_head_op: name_len {} exceeds isize::MAX",
                            name_len,
                        ));
                        return -8_i32;
                    }
                    // SAFETY: the host (`DylibNodeEntry::sync_head_op`) passes a
                    // pointer to a live, immutable byte slice of exactly
                    // `name_len` bytes, valid for this call; we only borrow it
                    // (no copy) and never retain the pointer. Non-null is
                    // established by the refusal directly above and u8's
                    // alignment is 1, so the `from_raw_parts` preconditions
                    // hold for every `name_len`, including 0.
                    let bytes = unsafe {
                        ::std::slice::from_raw_parts(name_ptr, name_len)
                    };
                    let name = match ::std::str::from_utf8(bytes) {
                        ::std::result::Result::Ok(s) => s,
                        ::std::result::Result::Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_sync_head_op: name was not valid UTF-8",
                            ));
                            return -3_i32;
                        }
                    };
                    // The two FILL ops are absent BY DESIGN (see above), so an
                    // op code outside this set is refused loudly and named.
                    //
                    // The codes are the SHARED consts the host resolver decodes
                    // with, reached by PATH and never re-spelled as literals —
                    // that sharing is the only thing making
                    // `cerulion_core::graph::node`'s "the two spellings of one
                    // wire cannot drift" true of this side of the wall.
                    let head_op = match op {
                        ::cerulion_core::graph::node::SYNC_HEAD_OP_PROBE_NEXT => {
                            ::cerulion_core::SyncHeadOp::ProbeNext
                        }
                        ::cerulion_core::graph::node::SYNC_HEAD_OP_PEEK_NEXT => {
                            ::cerulion_core::SyncHeadOp::PeekNext
                        }
                        ::cerulion_core::graph::node::SYNC_HEAD_OP_ADVANCE => {
                            ::cerulion_core::SyncHeadOp::Advance
                        }
                        ::cerulion_core::graph::node::SYNC_HEAD_OP_VOID => {
                            ::cerulion_core::SyncHeadOp::Void
                        }
                        other => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_sync_head_op: unknown op code {} (this ABI carries \
                                 0=ProbeNext, 1=PeekNext, 2=Advance, 3=Void; the two FILL ops ride \
                                 cerulion_node_drain_trigger_input / \
                                 cerulion_node_refill_trigger_input)",
                                other,
                            ));
                            return -5_i32;
                        }
                    };
                    // Refused BEFORE the lock and before the op runs: a null
                    // out-param is a caller bug, and a check deferred to the
                    // write site would accept it on every answer that carries
                    // no stamp and reject it on the ones that do.
                    if out_ts.is_null() || out_kind.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_sync_head_op: out_ts or out_kind pointer was null",
                        ));
                        return -7_i32;
                    }
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_sync_head_op: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            let answer = match node.context.as_mut() {
                                Some(ctx) => ctx.sync_head_op(name, head_op),
                                // Not yet init'd. Deliberately NOT the drain
                                // export's silent success-with-zero: this
                                // return type's zero is `Nothing`, which is
                                // positive evidence of scarcity the matcher may
                                // descend on. A node that has observed nothing
                                // vouches for nothing.
                                None => ::cerulion_core::SyncOpAnswer::Failed,
                            };
                            let (kind, ts) = match answer {
                                ::cerulion_core::SyncOpAnswer::Nothing => (
                                    ::cerulion_core::graph::node::SYNC_OP_ANSWER_NOTHING,
                                    0_u64,
                                ),
                                ::cerulion_core::SyncOpAnswer::Present => (
                                    ::cerulion_core::graph::node::SYNC_OP_ANSWER_PRESENT,
                                    0_u64,
                                ),
                                ::cerulion_core::SyncOpAnswer::Head(ts) => (
                                    ::cerulion_core::graph::node::SYNC_OP_ANSWER_HEAD,
                                    ts,
                                ),
                                ::cerulion_core::SyncOpAnswer::Stamp(ts) => (
                                    ::cerulion_core::graph::node::SYNC_OP_ANSWER_STAMP,
                                    ts,
                                ),
                                ::cerulion_core::SyncOpAnswer::Failed => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_sync_head_op: the op answered Failed",
                                    ));
                                    return -6_i32;
                                }
                            };
                            // SAFETY: both pointers were null-checked above, and
                            // the host passes writable slots it owns for the
                            // duration of this call.
                            unsafe {
                                *out_ts = ts;
                                *out_kind = kind;
                            }
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_sync_head_op: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_sync_head_op: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // ================================================================
            // THE KEYSTONE: state capture/restore across the wall
            // ================================================================
            //
            // Three OPTIONAL, ADDITIVE exports, resolved best-effort by the host
            // exactly like the snapshot pair and the drain
            // symbol above — symbol PRESENCE is the capability, and
            // `CERULION_ABI_VERSION` is deliberately NOT bumped, so a raw-FFI or
            // earlier cdylib still loads and simply declares no restorable state.
            //
            // WHY IT MUST BE HERE AND NOT HOST-SIDE. The FFI boundary means that
            // the host cannot reach a cdylib node's internals: the node instance
            // lives in this library's process-global `NODES`, and the host holds
            // a `u64` plus fn pointers. So the encode and the decode both run
            // INSIDE the library, on its own data, through the SAME generated
            // `CerulionState` impl `#[cerulion_node]` folds in — which is what
            // makes a cdylib's bytes identical to the in-process form's.
            //
            // THE STREAMING SINK IS LOAD-BEARING. The capture hands the
            // host a callback rather than a buffer, so an arbitrarily large state
            // crosses with no size pre-pass and no allocation crossing the
            // boundary — hence no `free` symbol and no
            // `Vec::from_raw_parts(ptr, len, len)` UB trap. A refusal rides the
            // callback's non-zero return, so the caller's own bound (the
            // inline carrier's `BoundedSink`, a test's `VecSink`) works through
            // the wall with no extra symbol.
            //
            // `capacity_hint` exists for ONE measured reason and is not
            // decoration: `StateSink::remaining_hint` is how a hash-like
            // container refuses BEFORE building its sort index, and a
            // callback ABI has nowhere to put that number. Without it a cdylib
            // node capturing a 30-million-entry `HashMap` into a 64 KiB sink
            // would allocate a ~240 MB index on the node thread and only THEN
            // discover it cannot fit — the exact cost `remaining_hint` was added
            // to avoid, fully re-opened on the SHIPPING path. `u64::MAX` means
            // unbounded, matching `remaining_hint`'s `None`.
            //
            // Return codes extend the set documented above:
            //   0  = success
            //  -1  = NODES mutex poisoned
            //  -2  = handle not found
            //  -3  = invalid argument (null sink / null payload with a length /
            //        null out-pointer)
            //  -4  = panic caught by catch_unwind
            //  -5  = the node's own encode/decode FAILED (message in LAST_ERROR)
            //  -6  = the HOST's sink refused the bytes (its buffer is too small)
            //
            // -5 and -6 are deliberately distinct: they are the same distinction
            // the carrier already draws between `ForkReason::EncoderError` and
            // `ForkReason::ArenaOverflow`, and they have opposite remedies — one
            // says the node is broken, the other says try a bigger buffer.

            /// The host's sink callback: `(user, chunk, len) -> 0 on accept`.
            type __CerStateSinkFn =
                extern "C" fn(*mut ::core::ffi::c_void, *const u8, usize) -> i32;

            /// A [`StateSink`] over the host's callback.
            ///
            /// `refused` LATCHES for the same reason `BoundedSink`'s does: a
            /// small write following a refused large one would land immediately
            /// after the last accepted byte, producing a byte stream missing a
            /// field in the middle that still looks structurally valid.
            struct __CerFfiStateSink {
                sink: __CerStateSinkFn,
                user: *mut ::core::ffi::c_void,
                hint: u64,
                written: u64,
                refused: bool,
            }

            impl ::cerulion_core::state::StateSink for __CerFfiStateSink {
                fn write(
                    &mut self,
                    bytes: &[u8],
                ) -> ::std::result::Result<(), ::cerulion_core::state::SinkFull> {
                    if self.refused {
                        return ::std::result::Result::Err(
                            ::cerulion_core::state::SinkFull,
                        );
                    }
                    // The host's trampoline guards a zero length before it builds
                    // a slice, so an empty write is sound on both sides.
                    let rc = (self.sink)(self.user, bytes.as_ptr(), bytes.len());
                    if rc == 0 {
                        self.written =
                            self.written.saturating_add(bytes.len() as u64);
                        ::std::result::Result::Ok(())
                    } else {
                        self.refused = true;
                        ::std::result::Result::Err(
                            ::cerulion_core::state::SinkFull,
                        )
                    }
                }

                fn remaining_hint(&self) -> ::core::option::Option<usize> {
                    if self.hint == u64::MAX {
                        ::core::option::Option::None
                    } else {
                        let left = self.hint.saturating_sub(self.written);
                        // A 64-bit hint on a 32-bit target saturates to
                        // `usize::MAX`, which is the SAFE direction: the hint may
                        // only ever make the encoder refuse EARLY, and `write`'s
                        // own bound is what actually holds the line.
                        ::core::option::Option::Some(
                            usize::try_from(left).unwrap_or(usize::MAX),
                        )
                    }
                }

                fn refuse(&mut self) {
                    self.refused = true;
                }
            }

            // `cerulion_node_state_shape`: this node type's
            // `CerulionState::STATE_SHAPE`.
            //
            // It takes NO handle, on purpose. The shape is a compile-time const
            // of the TYPE and one cdylib carries one node type (the same reason
            // `cerulion_node_info()` takes no handle), so the host resolves it
            // ONCE at load and never needs the `NODES` lock, an initialized
            // instance, or a particular call order to answer "does this node
            // declare restorable state?".
            #[no_mangle]
            pub extern "C" fn cerulion_node_state_shape(out_shape: *mut u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    if out_shape.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_state_shape: out_shape pointer was null",
                        ));
                        return -3_i32;
                    }
                    // SAFETY: non-null (checked), and the host passes a writable
                    // slot it owns for the duration of this call.
                    unsafe {
                        *out_shape =
                            <#node_name as ::cerulion_core::state::CerulionState>::STATE_SHAPE;
                    }
                    0
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_state_shape: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // `cerulion_node_inline_safe`: this node type's
            // `CerulionState::INLINE_SAFE`.
            //
            // Handle-less for the same reason as `cerulion_node_state_shape`: it
            // is a compile-time const of the TYPE. The host resolves it ONCE at
            // load, so the boundary's carrier decision costs no lock, no FFI
            // call and no initialized instance.
            //
            // Without it every cdylib node inherits the host's SAFE default
            // (`false`) and takes the fork carrier every cadence. That is not a
            // wrong capture — both carriers emit identical bytes — but it is a
            // `fork(2)` per anchor for nodes that could have been encoded inline
            // in microseconds, and the failure is SILENT: the anchors still
            // appear.
            #[no_mangle]
            pub extern "C" fn cerulion_node_inline_safe() -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    i32::from(
                        <#node_name as ::cerulion_core::state::CerulionState>::INLINE_SAFE,
                    )
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_inline_safe: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // `cerulion_node_cer_probe`: the pre-fork lock probe —
            // `CerulionState::cer_probe`, total over this node's DECLARED state.
            //
            // Takes the handle, because unlike the two consts above it is a
            // question about the INSTANCE: is any lock in its state graph held
            // right now?
            //
            // `try_lock` on `NODES`, never `lock`. This runs on the node thread
            // at a step boundary, where the executor's only blocking primitive
            // must be none at all — and a probe that BLOCKED to answer "is
            // anything blocking?" would be the very stall it exists to detect.
            // Every refusal (poisoned, contended, unknown handle, panicking
            // probe) answers 0 = NOT QUIESCENT, which costs one skipped anchor
            // and is the only safe direction: forking into a held lock leaves
            // the child holding it forever, since a fork child has one thread.
            #[no_mangle]
            pub extern "C" fn cerulion_node_cer_probe(handle: u64) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    let guard = match NODES.try_lock() {
                        Ok(g) => g,
                        Err(_) => {
                            // Contended or poisoned — either way this cdylib is
                            // not in a state to be forked from.
                            return 0_i32;
                        }
                    };
                    match guard.as_ref().and_then(|m| m.get(&handle)) {
                        Some(node) => i32::from(
                            ::cerulion_core::state::CerulionState::cer_probe(&node.inner),
                        ),
                        None => 0_i32,
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_cer_probe: the node's lock probe panicked",
                        ));
                        0
                    }
                }
            }

            // `cerulion_node_capture_state`: encode this node's state through the
            // host's sink, using the SAME generated `cer_capture` the in-process
            // form runs — so the two emit identical bytes.
            #[no_mangle]
            pub extern "C" fn cerulion_node_capture_state(
                handle: u64,
                sink: ::core::option::Option<__CerStateSinkFn>,
                user: *mut ::core::ffi::c_void,
                capacity_hint: u64,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    let sink = match sink {
                        ::core::option::Option::Some(f) => f,
                        ::core::option::Option::None => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_capture_state: sink callback was null",
                            ));
                            return -3_i32;
                        }
                    };
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_capture_state: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            let mut out = __CerFfiStateSink {
                                sink,
                                user,
                                hint: capacity_hint,
                                written: 0,
                                refused: false,
                            };
                            // Drive the user's encoder under an
                            // INNER `catch_unwind` so a panicking
                            // `CerulionState` impl does NOT unwind through the
                            // LIVE `NODES` MutexGuard (`guard`). The state
                            // contract is explicit that a capture panic escaping
                            // the guard is not a mere poisoned mutex: it kills
                            // every SUBSEQUENT operation on every node of this
                            // cdylib for the process lifetime, and misreports
                            // the cause as a tick panic that never happened.
                            // The inline carrier wraps its own capture for
                            // exactly this reason ("the node mutex is NOT
                            // poisoned"), and the cdylib's `NODES` is that
                            // mutex's twin on this side of the wall. Same
                            // containment as `cerulion_node_external_source`'s
                            // INNER `catch_unwind` above; the OUTER one stays the
                            // backstop for a panic raised outside this call.
                            //
                            // A capture is READ-ONLY, so an unwound one leaves
                            // no torn state — only a discarded encoding, which
                            // the -4 return already tells the host to distrust.
                            let outcome = match ::std::panic::catch_unwind(
                                ::std::panic::AssertUnwindSafe(|| {
                                    ::cerulion_core::state::CerulionState::cer_capture(
                                        &node.inner,
                                        &mut out,
                                    )
                                }),
                            ) {
                                ::std::result::Result::Ok(o) => o,
                                ::std::result::Result::Err(_) => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_capture_state: the node's state \
                                         encoder panicked; nothing was captured",
                                    ));
                                    return -4_i32;
                                }
                            };
                            match outcome {
                                ::std::result::Result::Ok(()) if !out.refused => 0,
                                // A sink that reports success while having REFUSED
                                // a write is a partial encoding claiming to be
                                // whole — the same arm `walk_inline` takes.
                                ::std::result::Result::Ok(()) => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_capture_state: the host sink refused a \
                                         write, so the capture is partial",
                                    ));
                                    -6
                                }
                                ::std::result::Result::Err(e) => {
                                    let full = out.refused
                                        || ::std::matches!(
                                            e,
                                            ::cerulion_core::state::StateError::SinkFull
                                        );
                                    __cer_set_last_error(::std::format!(
                                        "cerulion_node_capture_state: {}", e,
                                    ));
                                    if full { -6 } else { -5 }
                                }
                            }
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_capture_state: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_capture_state: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            // `cerulion_node_restore_state`: apply a recorded anchor's PAYLOAD
            // (framing already stripped and shape already checked by the host)
            // to this node, through the generated `cer_restore` — field by
            // field, so a `#[cerulion(reconstruct)]` field is left untouched.
            //
            // A decode failure leaves the node PARTIALLY restored and says so
            // with a non-zero code. That is deliberate:
            // the caller must fail the run, because the alternative — rolling
            // back to `Default` — is fabricated state (Principle #13) and
            // produces a divergence report about an execution that never
            // happened.
            #[no_mangle]
            pub extern "C" fn cerulion_node_restore_state(
                handle: u64,
                payload_ptr: *const u8,
                payload_len: usize,
            ) -> i32 {
                let result = ::std::panic::catch_unwind(::std::panic::AssertUnwindSafe(|| {
                    if payload_len > 0 && payload_ptr.is_null() {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_restore_state: payload pointer was null with a \
                             non-zero length",
                        ));
                        return -3_i32;
                    }
                    // A node whose every field is escaped captures ZERO bytes, so
                    // an empty payload is legitimate — and `from_raw_parts` over a
                    // null pointer is UB even at length zero (the established
                    // precedent), so the empty slice is built by hand.
                    let payload: &[u8] = if payload_len == 0 {
                        &[]
                    } else {
                        // SAFETY: non-null (checked above) and the host passes a
                        // live, immutable byte slice of exactly `payload_len`
                        // bytes, valid for this call; we only read it.
                        unsafe { ::std::slice::from_raw_parts(payload_ptr, payload_len) }
                    };
                    let mut guard = match NODES.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            __cer_set_last_error(::std::string::String::from(
                                "cerulion_node_restore_state: NODES mutex poisoned",
                            ));
                            return -1_i32;
                        }
                    };
                    match guard.as_mut().and_then(|m| m.get_mut(&handle)) {
                        Some(node) => {
                            let mut cursor =
                                ::cerulion_core::state::StateCursor::new(payload);
                            // The same INNER `catch_unwind` as the capture
                            // export, for the same reason — a panicking
                            // decoder must not poison `NODES` and brick every
                            // node of this cdylib. UNLIKE the capture, a
                            // restore MUTATES, so an unwound one leaves the
                            // node PARTIALLY restored: that is reported with
                            // the same words the `Err` arm below uses, because
                            // it is the same consequence — the caller must
                            // fail the run rather than continue against state
                            // that is neither the recording's nor the node's.
                            let decoded = ::std::panic::catch_unwind(
                                ::std::panic::AssertUnwindSafe(|| {
                                    ::cerulion_core::state::CerulionState::cer_restore(
                                        &mut node.inner,
                                        &mut cursor,
                                    )
                                }),
                            );
                            let decoded = match decoded {
                                ::std::result::Result::Ok(d) => d,
                                ::std::result::Result::Err(_) => {
                                    __cer_set_last_error(::std::string::String::from(
                                        "cerulion_node_restore_state: the node's state \
                                         decoder panicked; the node is PARTIALLY restored \
                                         and the run must fail",
                                    ));
                                    return -4_i32;
                                }
                            };
                            if let ::std::result::Result::Err(e) = decoded {
                                __cer_set_last_error(::std::format!(
                                    "cerulion_node_restore_state: {}", e,
                                ));
                                return -5_i32;
                            }
                            // Trailing bytes mean the running field list disagrees
                            // with the recorded one — a real drift signal the
                            // shape check could not see (a `#[cerulion(serde)]`
                            // field folds only its NAME into `STATE_SHAPE`), so it
                            // is an error rather than something to ignore.
                            if let ::std::result::Result::Err(e) = cursor.finish() {
                                __cer_set_last_error(::std::format!(
                                    "cerulion_node_restore_state: {}", e,
                                ));
                                return -5_i32;
                            }
                            0
                        }
                        None => {
                            __cer_set_last_error(::std::format!(
                                "cerulion_node_restore_state: handle {} not found", handle,
                            ));
                            -2
                        }
                    }
                }));
                match result {
                    Ok(code) => code,
                    Err(_) => {
                        __cer_set_last_error(::std::string::String::from(
                            "cerulion_node_restore_state: panic caught by catch_unwind",
                        ));
                        -4
                    }
                }
            }

            #external_source_ffi
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
// CI hardening: no ADVERTISED attribute may be silently
// discarded on the cdylib path.
// ═════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod advertised_attribute_parity_tests {
    //! **Every attribute the macro tells the user it accepts must reach the
    //! runtime, or be declared compile-time-only with a stated reason.**
    //!
    //! The defect has two known shapes. The comment above `policy_json`
    //! describes the first: *"Without this plumbing the macro accepts these
    //! but `let _ = attr;`-discards them; the operator has to redeclare every policy in
    //! YAML."* The second is the same defect on one arm — `unbounded_sync` missing from
    //! that chain, so a cdylib emits info JSON with no `"policy"` key, the
    //! host reads `policy: None`, and a fusion node silently degrades to `Data`
    //! (fires on ANY single input) instead of the all-inputs contract. Neither
    //! shape is detectable by any
    //! test in the tree: an accepted-but-discarded attribute compiles,
    //! expands, loads and runs — it just does nothing.
    //!
    //! The oracle is a DIFFERENTIAL over the real emitter. For each attribute,
    //! [`gen_cdylib`] is invoked twice — once on a node that declares it, once
    //! on an otherwise-identical node that does not — and the attribute's
    //! marker must appear in the first expansion and be ABSENT from the
    //! second. Present-only would pass against a key hardcoded into every
    //! expansion; the absence half is what proves the declaration CAUSED it.
    //!
    //! The accepted set is READ OUT of `parse.rs` rather than hand-listed, so
    //! a new attribute fails this test until somebody classifies it, and it is
    //! cross-checked against the set the unknown-attribute diagnostics
    //! ADVERTISE to the user, so the two halves cannot drift apart either.

    use super::gen_cdylib;
    use crate::parse::{FieldInputAttr, FieldOutputAttr, NodeLevelAttrs, ParsedBackpressurePolicy};
    use std::collections::BTreeSet;

    /// Node-level attributes that are deliberately compile-time-only — they
    /// never reach the cdylib info JSON, and that is correct.
    ///
    /// Read the reasons before adding an entry: "it does not reach the
    /// runtime" is the DEFECT this test exists to catch, so an entry here is a
    /// claim that the attribute has no runtime meaning at all.
    const NODE_LEVEL_COMPILE_TIME_ONLY: &[(&str, &str)] = &[
        (
            "allow_non_deterministic",
            "Blanket determinism-lint opt-out: consumed entirely at expansion time by the \
             `#[cerulion_node_impl]` determinism lint (the `determinism.rs` BANNED table). \
             Nothing downstream of the compiler can act on it.",
        ),
        (
            "uses_live_io",
            "IO-class determinism-lint opt-out: same expansion-time-only consumption \
             (`impl_macro.rs` reads it to suppress the IO-class deny rows). NOTE: its own \
             diagnostic text promises more than it delivers — it tells the user to declare \
             it so the framework \"records it for replay verification\", and nothing \
             records anything (it is not in the info JSON, not in `NodeInfo`, not in the \
             bag, not in replay). This entry classifies \
             the attribute as it BEHAVES and deliberately does not bless the promise.",
        ),
    ];

    // ── the emitter probe ────────────────────────────────────────────────

    fn ident(name: &str) -> syn::Ident {
        syn::Ident::new(name, proc_macro2::Span::call_site())
    }

    fn port_type() -> syn::Type {
        syn::parse_str::<syn::Type>("Vector3").expect("a port type parses")
    }

    fn base_input() -> FieldInputAttr {
        FieldInputAttr {
            field_name: ident("inp"),
            field_type: port_type(),
            trigger: false,
            depth: None,
            backpressure: None,
            expect_within_ms: None,
        }
    }

    fn base_output() -> FieldOutputAttr {
        FieldOutputAttr {
            field_name: ident("out"),
            field_type: port_type(),
            promise_within_ms: None,
        }
    }

    /// Run the REAL cdylib emitter over a synthetic one-input/one-output node
    /// and return the expansion as searchable text.
    ///
    /// Two normalisations, both load-bearing. The JSON template literals reach
    /// `TokenStream::to_string` ESCAPED (`\"period_ms\":16`), so the escapes
    /// are undone; and `to_string` spaces tokens unpredictably, so whitespace
    /// is squeezed out. Both happen AFTER the emitter runs — nothing about the
    /// emitted code is being assumed, only re-read.
    fn expand(
        node_level: &NodeLevelAttrs,
        inputs: &[FieldInputAttr],
        outputs: &[FieldOutputAttr],
    ) -> String {
        let entry = ident("ProbeNodeEntry");
        let node = ident("ProbeNode");
        let output_names: Vec<String> = outputs.iter().map(|o| o.field_name.to_string()).collect();
        let ts = gen_cdylib(
            &entry,
            &node,
            inputs,
            &output_names,
            outputs,
            node_level,
            None,
        );
        ts.to_string()
            .replace("\\\"", "\"")
            .replace([' ', '\n'], "")
    }

    /// A node-level attribute, the declaration that sets it, and the marker
    /// its declaration must put into the expansion.
    ///
    /// Probe VALUES are deliberately distinctive (`4242`, `4343`, …) so a
    /// marker cannot accidentally match unrelated generated code.
    fn node_level_probe(attr: &str) -> Option<(NodeLevelAttrs, String)> {
        let base = NodeLevelAttrs::default();
        Some(match attr {
            "period_ms" => (
                NodeLevelAttrs {
                    period_ms: Some(4242),
                    ..base
                },
                r#""policy":{"period_ms":4242}"#.to_string(),
            ),
            "sync_window_ms" => (
                NodeLevelAttrs {
                    sync_window_ms: Some(4343),
                    ..base
                },
                r#""policy":{"sync_window_ms":4343}"#.to_string(),
            ),
            "unbounded_sync" => (
                NodeLevelAttrs {
                    unbounded_sync: true,
                    ..base
                },
                r#""policy":{"unbounded_sync":true}"#.to_string(),
            ),
            "external" => (
                NodeLevelAttrs {
                    external: true,
                    ..base
                },
                r#""policy":{"external":true}"#.to_string(),
            ),
            "tick_within_ms" => (
                NodeLevelAttrs {
                    tick_within_ms: Some(4444),
                    ..base
                },
                r#""tick_within_ms":4444"#.to_string(),
            ),
            "throttle_ms" => (
                NodeLevelAttrs {
                    throttle_ms: Some(4545),
                    ..base
                },
                r#""throttle_ms":4545"#.to_string(),
            ),
            _ => return None,
        })
    }

    /// An input-level attribute, the declaration that sets it, and the marker
    /// its declaration must put into the expansion.
    fn input_level_probe(attr: &str) -> Option<(FieldInputAttr, String)> {
        let base = base_input();
        Some(match attr {
            "trigger" => (
                FieldInputAttr {
                    trigger: true,
                    ..base
                },
                r#""trigger":true"#.to_string(),
            ),
            "depth" => (
                FieldInputAttr {
                    depth: Some(4646),
                    ..base
                },
                r#""depth":4646"#.to_string(),
            ),
            "backpressure" => (
                FieldInputAttr {
                    backpressure: Some(ParsedBackpressurePolicy::Sample(4747)),
                    ..base
                },
                r#""backpressure":{"sample":4747}"#.to_string(),
            ),
            "expect_within_ms" => (
                FieldInputAttr {
                    expect_within_ms: Some(4848),
                    ..base
                },
                r#""expect_within_ms":4848"#.to_string(),
            ),
            _ => return None,
        })
    }

    // ── reading the accepted set out of parse.rs ─────────────────────────

    /// Comment-stripped view (house pattern; block comments NEST, and an
    /// unterminated block swallows the rest so the walk fails CLOSED).
    ///
    /// Load-bearing twice over: `parse.rs` names REMOVED attributes (`fifo`,
    /// `lifo`, `max_age_ms`, `filter`) in doc comments, and this module's own
    /// header names every attribute it checks.
    pub(super) fn code_only(src: &str) -> String {
        let chars: Vec<char> = src.chars().collect();
        let mut out = String::with_capacity(src.len());
        let mut i = 0usize;
        let mut depth = 0usize;
        while i < chars.len() {
            if depth > 0 {
                if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                    depth += 1;
                    i += 2;
                } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                    depth -= 1;
                    i += 2;
                } else {
                    if chars[i] == '\n' {
                        out.push('\n');
                    }
                    i += 1;
                }
                continue;
            }
            if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                depth = 1;
                i += 2;
            } else if chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
                while i < chars.len() && chars[i] != '\n' {
                    i += 1;
                }
            } else {
                out.push(chars[i]);
                i += 1;
            }
        }
        out
    }

    /// One arm of a `match` on an attribute name.
    struct MatchArm {
        /// The names on the left of `=>` (a `"a" | "b"` pattern yields two).
        names: Vec<String>,
        /// Whether the arm body rejects the name instead of accepting it.
        rejects: bool,
    }

    /// The OUTERMOST arms of the `match` written as `match_expr`, searched
    /// for after `anchor` (the enclosing item, so two functions matching on
    /// the same expression cannot be confused).
    ///
    /// Brace-depth tracked, so a NESTED match — the backpressure-policy match
    /// inside `parse_input_attr`'s `backpressure` arm — contributes nothing.
    /// Both naive alternatives were tried and are wrong: a depth-blind scan
    /// captured `drop_oldest`/`block`/`sample` as if they were input
    /// attributes, and truncating at the first `other =>` instead lost
    /// `expect_within_ms` entirely. `match_expr` is likewise not optional:
    /// the FIRST `match` inside `parse_input_attr` is `match meta`, whose
    /// arms are `syn::Meta::*` paths and yield an empty (silently vacuous)
    /// accepted set.
    fn outer_match_arms(stripped: &str, anchor: &str, match_expr: &str) -> Vec<MatchArm> {
        let at = stripped
            .find(anchor)
            .unwrap_or_else(|| panic!("`{anchor}` not found in parse.rs — this walk is stale"));
        let m = stripped[at..]
            .find(match_expr)
            .map(|p| at + p)
            .unwrap_or_else(|| {
                panic!("`{match_expr}` not found under `{anchor}` — this walk is stale")
            });
        let open = stripped[m..]
            .find('{')
            .map(|p| m + p + 1)
            .expect("the match has a body");

        let chars: Vec<char> = stripped[open..].chars().collect();
        let mut arms = Vec::new();
        let mut depth = 1usize; // inside the match body
        let mut pattern = String::new();
        let mut body = String::new();
        let mut in_body = false;
        let mut i = 0usize;
        while i < chars.len() {
            let c = chars[i];
            if c == '{' {
                depth += 1;
                if in_body {
                    body.push(c);
                }
                i += 1;
                continue;
            }
            if c == '}' {
                depth -= 1;
                if depth == 0 {
                    break; // end of the match
                }
                if in_body {
                    body.push(c);
                    if depth == 1 {
                        // The arm's brace body just closed.
                        arms.push(finish_arm(&pattern, &body));
                        pattern.clear();
                        body.clear();
                        in_body = false;
                    }
                }
                i += 1;
                continue;
            }
            if depth == 1 && !in_body && c == '=' && chars.get(i + 1) == Some(&'>') {
                in_body = true;
                i += 2;
                continue;
            }
            if depth == 1 && in_body && c == ',' {
                // A non-brace arm body (`"x" => expr,`).
                arms.push(finish_arm(&pattern, &body));
                pattern.clear();
                body.clear();
                in_body = false;
                i += 1;
                continue;
            }
            if in_body {
                body.push(c);
            } else {
                pattern.push(c);
            }
            i += 1;
        }
        arms
    }

    fn finish_arm(pattern: &str, body: &str) -> MatchArm {
        let names = pattern
            .split('|')
            .filter_map(|p| {
                let p = p.trim().trim_end_matches(',').trim();
                let name = p.trim_matches('"').trim();
                if p.starts_with('"') && !name.is_empty() {
                    Some(name.to_string())
                } else {
                    None
                }
            })
            .collect();
        MatchArm {
            names,
            // The arm's OWN first statement, not anything nested inside it:
            // an accepting arm may legitimately contain a nested match whose
            // catch-all rejects (that is exactly `backpressure`'s shape), and
            // a `contains` check would classify the whole arm as rejecting.
            rejects: body
                .trim_start()
                .strip_prefix('{')
                .unwrap_or(body)
                .trim_start()
                .starts_with("return Err("),
        }
    }

    /// The names an arm-set ACCEPTS (rejecting arms and the `other` catch-all
    /// contribute nothing).
    /// The output-level attributes `parse_output_attr` ACCEPTS, derived from
    /// its source rather than hand-named.
    ///
    /// It is an if-chain, not a `match`, so `outer_match_arms` cannot read it.
    /// The shape it does have is a guard: inside the `name = value` branch it
    /// rejects everything that is not one of a small set of identifiers
    /// (`if ident != "promise_within_ms" { return Err(..) }`). So the accepted
    /// set is exactly the identifiers compared with `ident !=` there — add a
    /// second accepted attribute and its comparison joins the set, which makes
    /// the probe inventory below fail until somebody probes it.
    ///
    /// The `ident ==` comparisons are deliberately NOT collected: those are
    /// the REJECTING arms, which name an identifier in order to refuse it.
    fn accepted_output_attrs(stripped: &str) -> BTreeSet<String> {
        let region = parse_output_attr_body(stripped);
        let mut out = BTreeSet::new();
        let needle = "ident != \"";
        let mut from = 0usize;
        while let Some(hit) = region[from..].find(needle) {
            let at = from + hit + needle.len();
            let Some(close) = region[at..].find('"') else {
                break;
            };
            let name = &region[at..at + close];
            if !name.is_empty() {
                out.insert(name.to_string());
            }
            from = at + close;
        }
        out
    }

    /// `parse_output_attr`'s own BALANCED body.
    ///
    /// Slicing from the function to the next COLUMN-ZERO `fn` /
    /// `pub fn`, with a fall back to end-of-file, is wrong here. There IS
    /// no such `fn`: `parse_output_attr` is followed by an INDENTED module, so the
    /// region would run to EOF and every later `ident != "…"` guard in the file —
    /// belonging to some unrelated parser — would count as an accepted OUTPUT
    /// attribute. The gate would then fail with a stale-inventory message
    /// while the output parser had not changed at all: a false alarm pointing
    /// at the wrong function, which is worse than the drift the gate exists to
    /// catch. Such a slice passes only by the accident of what happens to follow.
    ///
    /// Brace-matched from the signature's opening `{`, so the region cannot
    /// outlive the function whatever follows it.
    fn parse_output_attr_body(stripped: &str) -> &str {
        let start = stripped
            .find("fn parse_output_attr")
            .expect("`parse_output_attr` not found — this walk is stale");
        let rest = &stripped[start..];
        let open = rest
            .find('{')
            .expect("`parse_output_attr` has no body — this walk is stale");
        let mut depth = 0i32;
        for (i, c) in rest[open..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &rest[open..open + i];
                    }
                }
                _ => {}
            }
        }
        panic!("`parse_output_attr`'s body is unbalanced — this walk is stale");
    }

    #[test]
    fn the_output_attr_walker_stops_at_the_end_of_its_own_function() {
        // The reported shape: an INDENTED guard after the parser, which the
        // column-zero-terminator slice swallowed all the way to EOF.
        let src = "\
fn parse_output_attr(x: u8) -> u8 {
    if ident != \"promise_within_ms\" {
        return 0;
    }
    1
}

mod later {
    fn unrelated() {
        if ident != \"not_an_output_attr\" {
            return;
        }
    }
}
";
        let got = accepted_output_attrs(src);
        assert!(
            got.contains("promise_within_ms"),
            "must still read the parser's own guard: {got:?}"
        );
        assert!(
            !got.contains("not_an_output_attr"),
            "a guard in a LATER module is not an accepted output attribute: {got:?}"
        );
        assert_eq!(got.len(), 1, "exactly the parser's own set: {got:?}");
    }

    fn accepted(arms: &[MatchArm]) -> BTreeSet<String> {
        arms.iter()
            .filter(|a| !a.rejects)
            .flat_map(|a| a.names.iter().cloned())
            .collect()
    }

    /// The lower_snake_case names quoted in backticks inside `text` — how
    /// every macro diagnostic in this crate advertises its accepted set.
    fn backticked(text: &str) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        let mut rest = text;
        while let Some(open) = rest.find('`') {
            let after = &rest[open + 1..];
            let Some(close) = after.find('`') else { break };
            let name = after[..close].trim();
            if !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
            {
                out.insert(name.to_string());
            }
            rest = &after[close + 1..];
        }
        out
    }

    fn parse_rs() -> String {
        std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("src")
                .join("parse.rs"),
        )
        .expect("read parse.rs")
    }

    /// The remainder of the string literal that `anchor` appears inside.
    ///
    /// Terminates at the first UNESCAPED `"` — these diagnostics are single
    /// literals broken across lines with `\`-continuations, so a
    /// "look for `\")`" terminator ran straight past the end of one message
    /// and swallowed the NEXT diagnostic (which is how `promise_within_ms`,
    /// an OUTPUT attribute, first showed up in the input-level advertised
    /// set).
    fn diagnostic_text(stripped: &str, anchor: &str) -> String {
        let at = stripped
            .find(anchor)
            .unwrap_or_else(|| panic!("diagnostic `{anchor}` not found in parse.rs"));
        let tail: Vec<char> = stripped[at..].chars().collect();
        let mut out = String::new();
        let mut i = 0usize;
        while i < tail.len() {
            if tail[i] == '\\' {
                // Skip the escape and whatever it escapes.
                i += 2;
                continue;
            }
            if tail[i] == '"' {
                break;
            }
            out.push(tail[i]);
            i += 1;
        }
        out
    }

    // ── the gates ────────────────────────────────────────────────────────

    #[test]
    fn the_comment_stripper_removes_both_syntaxes_and_nothing_else() {
        assert_eq!(
            code_only("\"period_ms\" => {} // fifo\n"),
            "\"period_ms\" => {} \n"
        );
        assert_eq!(code_only("/// fifo\ncode\n"), "\ncode\n");
        assert_eq!(code_only("a /* x /* y */ fifo */ b"), "a  b");
        assert_eq!(code_only("// /* fifo\nreal\n"), "\nreal\n");
        assert_eq!(code_only("code /* fifo"), "code ");
    }

    #[test]
    fn the_arm_reader_reads_outer_arms_only_and_separates_accept_from_reject() {
        let src = r#"
fn probe() {
    match key {
        "legacy" => {
            return Err(oops);
        }
        "kept" => {
            inner = match v {
                "nested_a" => A,
                "nested_b" => B,
                other => return Err(oops),
            };
        }
        "a" | "b" => {
            flag = true;
        }
        "terse" => x = 1,
        other => {
            return Err(oops);
        }
    }
}
"#;
        let arms = outer_match_arms(src, "fn probe", "match key");
        assert_eq!(
            accepted(&arms),
            ["a", "b", "kept", "terse"]
                .iter()
                .map(|s| s.to_string())
                .collect::<BTreeSet<_>>(),
            "the reader must take the outer arms only, drop rejecting arms, split `|` \
             patterns, and handle a non-brace arm body"
        );
    }

    #[test]
    fn the_accepted_set_and_the_advertised_set_cannot_drift() {
        let stripped = code_only(&parse_rs());

        // ── node level ────────────────────────────────────────────────────
        let node_accepted = accepted(&outer_match_arms(
            &stripped,
            "impl Parse for NodeAttr",
            "match key.to_string().as_str()",
        ));
        let node_advertised: BTreeSet<String> = backticked(&diagnostic_text(
            &stripped,
            "unknown attribute `{other}`, expected",
        ))
        .into_iter()
        .filter(|n| n != "other")
        .collect();
        assert_eq!(
            node_accepted, node_advertised,
            "the node-level attributes `#[cerulion_node(...)]` ACCEPTS and the set its \
             unknown-attribute error ADVERTISES have drifted.\n  accepted:   \
             {node_accepted:?}\n  advertised: {node_advertised:?}\nFIX whichever half is \
             wrong: an accepted-but-unadvertised attribute is undiscoverable, an \
             advertised-but-unaccepted one is a lie."
        );
        assert!(
            node_accepted.len() >= 8,
            "only {} node-level attributes were extracted from parse.rs — the reader has \
             broken, which would make every check below vacuous",
            node_accepted.len()
        );

        // ── input level ───────────────────────────────────────────────────
        let in_accepted = accepted(&outer_match_arms(
            &stripped,
            "fn parse_input_attr",
            "match ident.to_string().as_str()",
        ));
        let in_msg = diagnostic_text(&stripped, "unknown input attribute");
        let sup = in_msg
            .find("Supported:")
            .expect("the input diagnostic lists a `Supported:` set");
        let in_advertised = backticked(&in_msg[sup..]);
        assert_eq!(
            in_accepted, in_advertised,
            "the `#[input(...)]` attributes ACCEPTED and those ADVERTISED in the \
             unknown-input-attribute error have drifted.\n  accepted:   {in_accepted:?}\n  \
             advertised: {in_advertised:?}"
        );
        assert!(
            in_accepted.len() >= 4,
            "only {} input-level attributes were extracted — the reader has broken",
            in_accepted.len()
        );
    }

    #[test]
    fn no_advertised_macro_attribute_is_discarded_on_the_cdylib_path() {
        let stripped = code_only(&parse_rs());
        let exempt: BTreeSet<&str> = NODE_LEVEL_COMPILE_TIME_ONLY
            .iter()
            .map(|(n, _)| *n)
            .collect();

        let baseline_inputs = [base_input()];
        let baseline_outputs = [base_output()];
        let baseline = expand(
            &NodeLevelAttrs::default(),
            &baseline_inputs,
            &baseline_outputs,
        );

        let mut findings: Vec<String> = Vec::new();
        let mut probed = 0usize;

        let check = |what: String, marker: &str, declared: &str, findings: &mut Vec<String>| {
            if !declared.contains(marker) {
                findings.push(format!(
                    "  {what} is accepted by parse.rs but its expansion carries NO `{marker}` \
                     in the cdylib info JSON — a cdylib declaring it would load with the \
                     setting silently absent"
                ));
            }
            if baseline.contains(marker) {
                findings.push(format!(
                    "  `{marker}` is already present in the expansion of a node that does NOT \
                     declare {what} — that probe proves nothing; pick a marker the \
                     declaration actually causes"
                ));
            }
        };

        // ── node level ────────────────────────────────────────────────────
        for attr in accepted(&outer_match_arms(
            &stripped,
            "impl Parse for NodeAttr",
            "match key.to_string().as_str()",
        )) {
            if exempt.contains(attr.as_str()) {
                continue;
            }
            let Some((node_level, marker)) = node_level_probe(&attr) else {
                findings.push(format!(
                    "  `#[cerulion_node({attr})]` is accepted by parse.rs but this test has no \
                     probe for it — add one to `node_level_probe` (it must reach the cdylib \
                     info JSON), or classify it in `NODE_LEVEL_COMPILE_TIME_ONLY` with the \
                     reason"
                ));
                continue;
            };
            probed += 1;
            let declared = expand(&node_level, &baseline_inputs, &baseline_outputs);
            check(
                format!("`#[cerulion_node({attr})]`"),
                &marker,
                &declared,
                &mut findings,
            );
        }

        // ── input level ───────────────────────────────────────────────────
        for attr in accepted(&outer_match_arms(
            &stripped,
            "fn parse_input_attr",
            "match ident.to_string().as_str()",
        )) {
            let Some((input, marker)) = input_level_probe(&attr) else {
                findings.push(format!(
                    "  `#[input({attr})]` is accepted by parse.rs but this test has no probe \
                     for it — add one to `input_level_probe`"
                ));
                continue;
            };
            probed += 1;
            let declared = expand(&NodeLevelAttrs::default(), &[input], &baseline_outputs);
            check(
                format!("`#[input({attr})]`"),
                &marker,
                &declared,
                &mut findings,
            );
        }

        // ── output level ──────────────────────────────────────────────────
        // `parse_output_attr` is an if-chain rather than a match, so its one
        // accepted item is named here — and the name is cross-checked against
        // the diagnostic so this hand-naming cannot go stale silently.
        {
            assert!(
                stripped.contains("expected `promise_within_ms = N`"),
                "`parse_output_attr` no longer advertises `promise_within_ms` as its only \
                 accepted item — this test's hand-named output set is stale; re-read the \
                 function and update it"
            );
            // The assert above pins the DIAGNOSTIC; on its own it cannot see a
            // second accepted attribute added without touching that message —
            // which would then be accepted, unprobed and possibly unemitted.
            // So the accepted set is DERIVED and required to equal what this
            // block actually probes.
            let output_accepted = accepted_output_attrs(&stripped);
            let output_probed: BTreeSet<String> =
                ["promise_within_ms".to_string()].into_iter().collect();
            assert_eq!(
                output_accepted, output_probed,
                "`parse_output_attr` accepts an output attribute this test does not probe (or \
                 the reverse). Accepted (derived from its source): {output_accepted:?}; \
                 probed here: {output_probed:?}. FIX: add a probe for each new attribute \
                 beside the `promise_within_ms` one below."
            );
            probed += 1;
            let declared = expand(
                &NodeLevelAttrs::default(),
                &baseline_inputs,
                &[FieldOutputAttr {
                    promise_within_ms: Some(4949),
                    ..base_output()
                }],
            );
            // The output object's `"promise_within_ms":{}` template key is
            // UNCONDITIONAL (absent declarations emit `null`), so the marker
            // is the VALUE, which is what a declaration actually causes.
            check(
                "`#[output(promise_within_ms = N)]`".to_string(),
                "\"4949\"",
                &declared,
                &mut findings,
            );
        }

        assert!(
            probed >= 11,
            "only {probed} attributes were probed — the accepted-set extraction has broken, \
             which would make this gate vacuous"
        );
        assert!(
            findings.is_empty(),
            "an advertised macro attribute does not survive to the cdylib info JSON:\n{}\n\n\
             This is the `let _ = attr;` class: the node compiles, the cdylib \
             loads, and the setting is simply gone. FIX: emit the key from `gen_cdylib`, or \
             — only if the attribute genuinely has NO runtime meaning — classify it in \
             `NODE_LEVEL_COMPILE_TIME_ONLY` with the reason.",
            findings.join("\n")
        );
    }

    #[test]
    fn every_compile_time_only_exemption_is_still_accepted_and_still_unemitted() {
        let stripped = code_only(&parse_rs());
        let node_accepted = accepted(&outer_match_arms(
            &stripped,
            "impl Parse for NodeAttr",
            "match key.to_string().as_str()",
        ));
        let baseline_inputs = [base_input()];
        let baseline_outputs = [base_output()];

        for (name, reason) in NODE_LEVEL_COMPILE_TIME_ONLY {
            assert!(
                node_accepted.contains(*name),
                "STALE exemption `{name}` — parse.rs no longer accepts it. FIX: delete the \
                 entry from `NODE_LEVEL_COMPILE_TIME_ONLY` (reason on file: {reason})"
            );
            assert!(
                !reason.trim().is_empty(),
                "exemption `{name}` carries no reason"
            );
            // And the "compile-time only" claim is CHECKED, not asserted: if
            // the attribute does reach the expansion it is parseable, and it
            // owes the host a parser arm rather than an exemption.
            let mut node_level = NodeLevelAttrs::default();
            match *name {
                "allow_non_deterministic" => node_level.allow_non_deterministic = true,
                "uses_live_io" => node_level.uses_live_io = true,
                other => panic!(
                    "exemption `{other}` has no probe here — add one so its \
                     compile-time-only claim is CHECKED rather than taken on trust"
                ),
            }
            let declared = expand(&node_level, &baseline_inputs, &baseline_outputs);
            assert!(
                !declared.contains(&format!("\"{name}\"")),
                "`{name}` is classified compile-time-only but DOES appear in the cdylib \
                 expansion — delete the exemption and give it a host-side parser arm"
            );
        }
    }
}
