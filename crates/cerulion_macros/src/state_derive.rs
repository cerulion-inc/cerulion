//! `#[derive(CerulionState)]`.
//!
//! The capture machinery, emitted from ONE place. `#[cerulion_node]` calls
//! into this module too ([`expand_node`]), so a node and a hand-derived helper
//! struct cannot disagree about what a field means.
//!
//! # A node writes nothing at all
//!
//! `#[cerulion_node]` emits the state impl itself, over the struct's non-port
//! fields, so an ordinary node is capturable with ZERO new lines — no derive,
//! no attribute, no import. MEASURED over this tree: **668 node structs, 651
//! of them (97.5%) capturable with no source change whatsoever**, and every
//! one of the remaining 17 fails to COMPILE at the offending field with the
//! one-line fix named, rather than losing that field's state silently.
//!
//! That last property is the whole trade and it is the decision's own stated
//! intent: an uncapturable field is a hard error. The alternative — an
//! implicit `reconstruct` — is state loss a resim would then attribute to node
//! logic, which is the outcome this design exists to prevent.
//!
//! # What the user writes
//!
//! Nothing, on an ordinary struct:
//!
//! ```text
//! #[derive(CerulionState)]
//! struct Pose { x: f64, y: f64 }
//! ```
//!
//! and one attribute on the exceptional field:
//!
//! ```text
//! #[derive(CerulionState)]
//! struct Slam {
//!     pose: Pose,
//!     #[cerulion(reconstruct)] cuda: CudaContext,
//! }
//! ```
//!
//! # The four field classes
//!
//! | Class | Chosen by | Capture | Restore |
//! |---|---|---|---|
//! | captured | the default | the field type's own `cer_capture` | its own `cer_restore` |
//! | `#[cerulion(reconstruct)]` | the user, OR the [resource inventory](resource) | nothing | left UNTOUCHED |
//! | `#[cerulion(serde)]` | the user | `u32` length + the field's `Serialize` bytes | `Deserialize` |
//! | `#[cerulion(unordered)]` | the user | iteration order, no `Ord` on the key | rebuilt, DUPLICATES REFUSED |
//!
//! # Observability is not state (by design)
//!
//! **This is the rule, written once, for the whole system.** Rollback netcode
//! draws the line in the same place and has for thirty years: GGPO re-simulates
//! the game and never the RENDERER, because a frame counter, a particle system
//! and a debug overlay do not decide what the game DOES. Restore one and you
//! have restored nothing that matters; fail to restore one and you have lost
//! nothing that matters.
//!
//! A field is **observability** when it exists to be READ BY A HUMAN OR A
//! DASHBOARD and nothing the node computes depends on it: a lifetime frame
//! tally, a bytes-sent total, a `render()` string, a histogram of latencies.
//!
//! **The sharp line: a counter that GATES BEHAVIOUR is state, full stop.** The
//! moment a `u64` is compared against a threshold that changes what the node
//! publishes, retries, or refuses, it stopped being a readout and became part
//! of the simulation — and no amount of "it's just a metric" naming changes
//! that. A flood latch's `armed` bit decides whether the NEXT line is loud, so
//! it is behaviour, even though the behaviour is only a log level. A dropped-
//! frame counter that trips a degrade at 100 is state. The same `u64` with no
//! reader but a dashboard is not.
//!
//! **The classification is per-field JUDGEMENT, and the machine cannot check
//! it.** There is no type, no name and no attribute that distinguishes "read by
//! a human" from "read by an `if`" — the two are the same `u64`. So this rule
//! is deliberately NOT wired into an inventory the way [`resource`] is: a name
//! list here would guess at intent, and guessing wrong SILENTLY drops real
//! state, which is the one failure this whole design exists to prevent. The
//! author of the field decides, and writes `#[cerulion(reconstruct)]` when the
//! answer is "observability".
//!
//! **Which is not the same as "always exclude it".** Where a field is
//! observability AND trivially capturable — two primitives, an atomic — the
//! cheap answer is to CARRY it, because reconstructing zeroes a number an
//! operator reads while saving nothing. Excluding earns its keep when capture
//! would drag a derive through a closure of types that are themselves handles,
//! or would record something meaningless in another process. The rule tells you
//! it is SAFE to drop the field; it does not tell you to.
//!
//! Worked, in the tree: `FloodCounter` and `FloodLatch` are observability and
//! are CAPTURED anyway (two primitives each — see the note on their
//! declarations). `TranscodeLoop::factory` and `DdsBridge::pump_shutdown` are
//! not observability at all — they are handles, and take `reconstruct` for the
//! resource reason.
//!
//! # The `where` clause is the whole diagnostic design
//!
//! One `T: CerulionState` predicate per captured field, `quote_spanned!` at
//! the FIELD'S TYPE, plus serde-style `T: CerulionState` for every type
//! parameter. Naming the obligation only at its USES (three consts, the probe,
//! capture, read, restore) is the natural way to write this and it is worse.
//!
//! **MEASURED against THIS emission** — deleting the
//! predicate and letting the body's uses speak — on
//! `tests/ui/type_error/state_derive_uncapturable_field.rs`:
//!
//! | Emission | `E0277` blocks for ONE bad field |
//! |---|---|
//! | `where`-clause predicate (shipped) | **1**, at the field's type span |
//! | per-use obligations only | **2**, the second at the `#[derive(..)]` span |
//!
//! The memo reports **6** for the same mutation. That number is not
//! reproduced here and the difference is not a disagreement: the memo measured
//! a sketch that named the obligation at six *separately spanned* sites, while
//! this emission spans only the predicate and lets the body inherit the derive
//! call site, so rustc dedupes five of the six into one. What matters is
//! unchanged and is what the table pins — the extra block points at
//! `#[derive(CerulionState)]` rather than at the field, which is precisely the
//! diagnostic an IDE squiggle would put in the wrong place.
//!
//! The message itself is not written here: `CerulionState` carries
//! `#[diagnostic::on_unimplemented]` (`crates/cerulion_core/src/state.rs`), so the
//! compiler prints the field, its type and both fixes unaided. All this
//! module has to do is point rustc at the right span.
//!
//! The COUNT is pinned two ways, with deliberately different strengths: the
//! `trybuild` snapshot above is exact but lives in the `#[ignore]`d group
//! (rustc rendering drifts between toolchains), while
//! `a_captured_field_gets_exactly_one_where_predicate_not_one_per_use` in this
//! module's tests gates every PR by counting predicates in the emitted tokens.
//!
//! ## …except inside an enum, where that message is false
//!
//! `CerulionState`'s handle fix is "mark the field
//! `#[cerulion(reconstruct)]`", and [`expand_enum`] REFUSES that attribute on
//! a variant's field. So an `enum Link { Connected(TcpStream) }` was a closed
//! loop — MEASURED both halves: the `E0277` names an escape, and writing it is
//! a hard error naming the `E0277`. A variant member is therefore bound
//! through `CerulionVariantMember` instead: a blanket alias over
//! `CerulionState`, identical in force, carrying the message that is true
//! there. EVERY member use names it, because one `CerulionState` use left in
//! the emission raises its own obligation and prints the misleading note as a
//! second block — the same one-versus-two count as the table above, pinned the
//! same two ways (`tests/ui/type_error/state_derive_uncapturable_variant_field.rs`
//! for the rendering, `an_enum_variants_members_are_bound_through_the_diagnostic_that_is_true_there`
//! for every PR).
//!
//! Why the derive cannot simply auto-classify such a field the way a STRUCT
//! field is auto-classified from the [resource inventory](resource) — two
//! independent reasons, one semantic and one about which way the rule fails —
//! is recorded on `CerulionVariantMember` itself and pinned by
//! `a_recognised_handle_auto_reconstructs_in_a_struct_and_is_bound_inside_a_variant`.
//!
//! # Bound placement is serde's, deliberately
//!
//! Every type parameter is bounded, like `#[derive(Serialize)]` and
//! `#[derive(Clone)]` — not a "perfect derive" that bounds only parameters
//! reachable from captured fields. Narrowing a bound later is
//! backward-compatible; widening one is not, so the conservative choice is the
//! reversible one. Measured cost today: zero — the tree contains no generic
//! node structs.

use crate::crate_root;
use proc_macro2::TokenStream;
use quote::{format_ident, quote, quote_spanned};
use syn::spanned::Spanned;
use syn::{Data, DeriveInput, Field, Fields, Ident, Type};

pub mod resource;

/// The most variants an enum may declare.
///
/// The tag rides ONE byte, matching `Option`/`Result` in the inventory and
/// reusing their `StateError::InvalidTag` verbatim. An enum past this is a
/// loud compile error rather than a silently widened tag, because widening it
/// would change the bytes of every enum already recorded.
const MAX_ENUM_VARIANTS: usize = 256;

/// How a struct field is treated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Escape {
    /// Captured through the field type's own `CerulionState`.
    Captured,
    /// Not captured; left untouched on restore and rebuilt in `restored()`.
    Reconstruct,
    /// Captured through the field's `Serialize`/`Deserialize`.
    Serde,
    /// A hash-like container captured in ITERATION order (no `Ord` key).
    Unordered,
}

/// One field, classified.
struct Classified<'a> {
    ty: &'a Type,
    escape: Escape,
}

/// Parse a field's `#[cerulion(...)]` attribute.
///
/// Returns `None` when the field carries none — the caller then consults the
/// resource inventory. An UNKNOWN key is rejected with the error-listing shape
/// `parse.rs` already uses for node-level attributes: its rule applied
/// literally, since a typo'd `#[cerulion(recontsruct)]` that silently meant
/// "capture it" would restore a stale handle.
fn parse_field_escape(field: &Field) -> Result<Option<Escape>, syn::Error> {
    let mut found: Option<Escape> = None;
    for attr in &field.attrs {
        if !attr.path().is_ident("cerulion") {
            continue;
        }
        attr.parse_nested_meta(|meta| {
            let key = meta
                .path
                .get_ident()
                .ok_or_else(|| meta.error("expected a bare key such as `reconstruct`"))?;
            let escape = match key.to_string().as_str() {
                "reconstruct" => Escape::Reconstruct,
                "serde" => Escape::Serde,
                "unordered" => Escape::Unordered,
                other => {
                    return Err(syn::Error::new(
                        key.span(),
                        format!(
                            "unknown `#[cerulion(...)]` key `{other}`, expected \
                             `reconstruct`, `serde`, or `unordered`"
                        ),
                    ));
                }
            };
            if found.is_some() {
                return Err(syn::Error::new(
                    key.span(),
                    "a field may carry only one `#[cerulion(...)]` key",
                ));
            }
            found = Some(escape);
            Ok(())
        })?;
    }
    Ok(found)
}

/// Classify one struct field: the explicit attribute wins, then the inventory.
///
/// The two `Reconstruct` provenances — the user said so, and the inventory
/// recognised it — deliberately produce IDENTICAL code and an identical shape:
/// the semantics are the same, so making them differ would mean adding an
/// explicit attribute to an already-recognised field changed the recording.
/// Reporting WHICH one fired belongs to the manifest, a later chunk.
fn classify<'a>(field: &'a Field) -> Result<Classified<'a>, syn::Error> {
    let escape = match parse_field_escape(field)? {
        Some(declared) => declared,
        None if resource::is_resource(&field.ty) => Escape::Reconstruct,
        None => Escape::Captured,
    };
    Ok(Classified {
        ty: &field.ty,
        escape,
    })
}

/// The shape contribution of an escaped field.
///
/// An escaped field folds its NAME and its ESCAPE KIND, never its type's
/// shape. That is load-bearing: if it folded nothing,
/// ADDING `#[cerulion(reconstruct)]` to a field would leave `STATE_SHAPE`
/// unchanged, so a bag captured before the change would decode
/// "successfully" against a node that no longer captures that field — the
/// silent-divergence class the shape exists to prevent.
fn escape_shape(escape: Escape) -> TokenStream {
    let __cer_root = crate_root::root();
    let marker = match escape {
        Escape::Reconstruct => "cerulion::reconstruct",
        Escape::Serde => "cerulion::serde",
        _ => unreachable!("only escaped fields carry an escape shape"),
    };
    quote! { #__cer_root::state::StateShape::of(#marker).finish() }
}

/// The ONE text both redundant-derive detectors emit.
///
/// Written once so the two orders (see [`redundant_state_derive`]) cannot
/// drift into telling the same user two different stories about one mistake.
pub(crate) const REDUNDANT_DERIVE_MSG: &str = "\
remove `#[derive(CerulionState)]` — `#[cerulion_node]` emits this impl \
itself, so carrying both is a conflicting-implementation error (E0119). The node's impl is the better one: it is built from the \
NON-PORT fields, while the derive would also walk the `#[input]`/`#[output]` \
ports, whose declared types are zero-sized SHM markers that carry no \
`CerulionState` and never will. Per-field escapes are unaffected — \
`#[cerulion(reconstruct)]`, `#[cerulion(serde)]` and `#[cerulion(unordered)]` \
are read by the fold-in from the same place the derive reads them.";

/// Does an attribute list carry a `derive(..)` naming `CerulionState`?
///
/// Returns the span of the offending entry, for a `compile_error!` that points
/// at the derive rather than at the struct.
///
/// Matched on the LAST PATH SEGMENT, exactly like `codegen::user_derives_default`
/// does for `Default`, so `CerulionState`, `cerulion_core::state::CerulionState`
/// and `::cerulion_macros::CerulionState` all hit.
///
/// # Residuals, stated in the direction they fail
///
/// Both are FAIL-OPEN into rustc's own `E0119`, which names both impls and the
/// two spans — loud, just not ours:
///
/// - **A RENAMED import** (`use ..CerulionState as Capturable;` then
///   `#[derive(Capturable)]`) is not recognised. A proc macro sees tokens, and
///   the token here is `Capturable`.
/// - Nothing else: the ORDER is covered on both sides. `#[cerulion_node]` above
///   the derive sees the derive in its own attrs and calls this; the derive
///   above `#[cerulion_node]` is expanded FIRST by rustc (which strips the
///   `derive` attribute before the attribute macro runs, so the attribute macro
///   can no longer see it) — but in that order the DERIVE still sees
///   `#[cerulion_node]`, and [`node_attr_span`] catches it there.
pub(crate) fn redundant_state_derive(attrs: &[syn::Attribute]) -> Option<proc_macro2::Span> {
    attrs
        .iter()
        .filter(|attr| attr.path().is_ident("derive"))
        .find_map(|attr| {
            let mut hit = None;
            let _ = attr.parse_nested_meta(|meta| {
                if path_names_state_derive(&meta.path) {
                    hit = Some(meta.path.span());
                }
                Ok(())
            });
            hit
        })
}

/// Is this one entry of a `derive(..)` list our `CerulionState`?
///
/// Shared by the detector above and by `codegen::strip_state_derive`, which
/// removes exactly the entries this recognises: two copies of the rule would
/// let the macro refuse a spelling it then failed to strip, producing the
/// double diagnostic the strip exists to prevent.
pub(crate) fn path_names_state_derive(path: &syn::Path) -> bool {
    path.segments
        .last()
        .is_some_and(|seg| seg.ident == "CerulionState")
}

/// Does an attribute list carry `#[cerulion_node(..)]`?
///
/// The mirror of [`redundant_state_derive`], consulted from inside the DERIVE.
/// It can only be true when the derive was written ABOVE `#[cerulion_node]`,
/// because an attribute macro consumes its own attribute — so a derive
/// expanding after `#[cerulion_node]` never sees one.
fn node_attr_span(attrs: &[syn::Attribute]) -> Option<proc_macro2::Span> {
    attrs
        .iter()
        .find(|attr| {
            attr.path()
                .segments
                .last()
                .is_some_and(|seg| seg.ident == "cerulion_node")
        })
        .map(|attr| attr.path().span())
}

/// Is this struct a NODE, as seen from inside the derive? — and where to say so.
///
/// [`node_attr_span`] alone is a NAME match, and a name is exactly the thing a
/// user can spell away:
///
/// ```text
/// use cerulion_core::prelude::cerulion_node as my_node;
///
/// #[derive(CerulionState)]   // expands FIRST
/// #[my_node(period_ms = 10)] // ... and this is not called `cerulion_node`
/// struct Camera { #[output] frame: Image, count: u64 }
/// ```
///
/// A proc macro resolves no names, so neither macro can catch that ALONE:
/// expanding the derive REMOVES the `derive` attribute, so `#[cerulion_node]`
/// runs against a struct that no longer carries it and its own
/// [`redundant_state_derive`] finds nothing. MEASURED before this fallback
/// existed: `E0119` *plus* an `E0063` for the `__cer_rt` the derive cannot
/// name, and a `note:` blaming `Vector3` — three wrong leads.
///
/// So the second signal is not a name at all, it is SHAPE. A
/// `#[cerulion_node]` must declare at least one `#[input]`/`#[output]` field
/// (`tests/ui/no_input_or_output_attrs_rejected.rs` pins that), and in this
/// order those attributes are still on the struct, because the attribute macro
/// that strips them has not run yet. A struct being derived that declares a
/// PORT is therefore a node whatever its attribute is called — and the same
/// predicate [`is_port_field`] the fold-in uses decides it, so the two cannot
/// drift apart.
///
/// Deliberately NOT a general fix, because there is no general fix: the OTHER
/// order (`#[my_node]` above the derive) is already caught by
/// [`redundant_state_derive`] matching the DERIVE's name, and a user who
/// renames BOTH still lands on `E0119` — loud, and rustc's. What this closes
/// is the one shape that is both reachable by an ordinary `use ... as ...` and
/// invisible to every name match.
///
/// Residual, stated because it is the cost: a struct that is NOT a node but
/// carries an `input`/`output` field attribute registered by some OTHER
/// derive's helper list, alongside this one, is refused. Nothing in this
/// workspace or its dependencies has that shape, and the refusal is a loud
/// error naming a concrete fix rather than a silent miscapture.
fn node_evidence_span(input: &DeriveInput) -> Option<proc_macro2::Span> {
    if let Some(span) = node_attr_span(&input.attrs) {
        return Some(span);
    }
    let Data::Struct(data) = &input.data else {
        return None;
    };
    // The struct NAME, not the port attribute: "remove the derive" reads as a
    // statement about this type, and an arrow at someone's `#[output]` line
    // would look like the port were the thing at fault.
    data.fields
        .iter()
        .any(is_port_field)
        .then(|| input.ident.span())
}

/// Entry point for `#[derive(CerulionState)]`.
pub fn derive(input: &DeriveInput) -> TokenStream {
    // The derive-ABOVE-the-attribute order, whatever that attribute is CALLED
    // (see `node_evidence_span`). Emitting ONLY the error (no impl) is what
    // keeps this to one diagnostic: `#[cerulion_node]` still runs afterwards
    // and still emits the fold-in impl, so there is nothing left to conflict
    // with, and the E0063 the derive would otherwise raise (its `cer_read`
    // cannot name the macro's injected `__cer_rt`) never appears.
    if let Some(span) = node_evidence_span(input) {
        return syn::Error::new(span, REDUNDANT_DERIVE_MSG).to_compile_error();
    }
    match expand(input) {
        Ok(tokens) => tokens,
        Err(err) => err.to_compile_error(),
    }
}

fn expand(input: &DeriveInput) -> Result<TokenStream, syn::Error> {
    match &input.data {
        Data::Struct(data) => expand_struct(input, &data.fields),
        Data::Enum(data) => expand_enum(input, data),
        Data::Union(_) => Err(syn::Error::new_spanned(
            &input.ident,
            "`CerulionState` cannot be derived for a union: a union has no \
             single declared value to capture. Wrap the reading you actually \
             want in a struct, or mark the field `#[cerulion(reconstruct)]`.",
        )),
    }
}

/// Which struct is being expanded — the derive's, or `#[cerulion_node]`'s.
///
/// The two differ in exactly one place, `cer_read`, and the difference is
/// forced rather than chosen. See [`is_port_field`] for the other half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Target {
    /// `#[derive(CerulionState)]` on an ordinary struct: every field is here,
    /// so `cer_read` can construct `Self` unless a field is escaped.
    Derived,
    /// The `#[cerulion_node]` fold-in: PORT fields and the macro's injected
    /// `__cer_rt` are not in the walk at all, so `cer_read` could never name
    /// them and always reports `Unrestorable`.
    Node,
}

/// Does this field declare a PORT?
///
/// The same predicate `strip_field_attrs_and_inject_hidden` uses to decide
/// what to strip, so the struct the user gets and the state impl emitted
/// beside it cannot disagree about which fields are ports. A field carrying
/// BOTH `#[input]` and `#[output]` lands in both of `FieldAttrs`' vectors
/// (`parse.rs`), so asking the FIELD rather than those vectors also excludes
/// it exactly once.
fn is_port_field(field: &Field) -> bool {
    field
        .attrs
        .iter()
        .any(|a| a.path().is_ident("input") || a.path().is_ident("output"))
}

/// Emit `impl CerulionState` for a `#[cerulion_node]` struct — the fold-in.
///
/// Called from `codegen::generate`, so an ordinary node costs the user ZERO
/// new lines, by design. What it walks is the **pre-injection**
/// field list minus ports, and both exclusions are load-bearing:
///
/// - A **port**'s declared type is a zero-sized SHM marker the impl macro
///   accesses through a per-tick local, never through the struct field. It
///   carries no `CerulionState` impl and never will, so walking it would make
///   EVERY node in the system a compile error.
/// - **`__cer_rt`** (`CerNodeRuntimeFields`) is appended by
///   `strip_field_attrs_and_inject_hidden` AFTER this runs, and holds an
///   `Arc<dyn Clock>`. It is in the resource inventory as well, so it would
///   classify `reconstruct` even if it were reached — but it is not reached,
///   because this walks `input` before the injection.
///
/// A prototype measured what a naive whole-struct walk costs: three
/// phantom lossy fields on a node whose real state is fully capturable.
pub(crate) fn expand_node(input: &DeriveInput) -> Result<TokenStream, syn::Error> {
    let Data::Struct(data) = &input.data else {
        // `#[cerulion_node]`'s own guard rejects non-structs first, with a
        // better message; this arm exists so the function is total.
        return Ok(TokenStream::new());
    };
    let named: Vec<&Field> = match &data.fields {
        Fields::Named(named) => named.named.iter().filter(|f| !is_port_field(f)).collect(),
        // A unit node struct is promoted to named-with-no-fields by the
        // macro; either way it captures nothing.
        Fields::Unit => Vec::new(),
        // Unreachable: `codegen::generate` rejects tuple structs before this.
        Fields::Unnamed(_) => return Ok(TokenStream::new()),
    };
    expand_fields(input, &named, Target::Node)
}

/// Assemble the `impl` block shared by the struct and enum paths.
#[allow(clippy::too_many_arguments)]
fn emit_impl(
    input: &DeriveInput,
    extra_predicates: Vec<TokenStream>,
    shape: TokenStream,
    inline_safe: TokenStream,
    min_bytes: TokenStream,
    probe_body: TokenStream,
    capture_body: TokenStream,
    read_body: TokenStream,
    restore_body: Option<TokenStream>,
) -> TokenStream {
    let __cer_root = crate_root::root();
    let name = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    // serde's own rule: one bound per type parameter, so resolution happens at
    // the INSTANTIATION where `T` is known rather than inside `impl<T>` where
    // nothing is (a prototype measured that as silent state loss).
    let param_predicates = input.generics.type_params().map(|tp| {
        let id = &tp.ident;
        quote! { #id: #__cer_root::state::CerulionState }
    });

    let existing = where_clause.map(|w| {
        let preds = &w.predicates;
        quote! { #preds }
    });

    let restore = restore_body.map(|body| {
        quote! {
            fn cer_restore(
                &mut self,
                src: &mut #__cer_root::state::StateCursor<'_>,
            ) -> ::std::result::Result<(), #__cer_root::state::StateError> {
                #body
            }
        }
    });

    quote! {
        #[automatically_derived]
        impl #impl_generics #__cer_root::state::CerulionState for #name #ty_generics
        where
            #(#extra_predicates,)*
            #(#param_predicates,)*
            #existing
        {
            const STATE_SHAPE: u64 = #shape;
            const INLINE_SAFE: bool = #inline_safe;
            const MIN_ENCODED_BYTES: usize = #min_bytes;

            fn cer_probe(&self) -> bool {
                #probe_body
            }

            fn cer_capture(
                &self,
                out: &mut dyn #__cer_root::state::StateSink,
            ) -> ::std::result::Result<(), #__cer_root::state::StateError> {
                #capture_body
            }

            fn cer_read(
                src: &mut #__cer_root::state::StateCursor<'_>,
            ) -> ::std::result::Result<Self, #__cer_root::state::StateError> {
                #read_body
            }

            #restore
        }
    }
}

fn expand_struct(input: &DeriveInput, fields: &Fields) -> Result<TokenStream, syn::Error> {
    let named = match fields {
        Fields::Named(named) => named.named.iter().collect::<Vec<_>>(),
        Fields::Unit => Vec::new(),
        Fields::Unnamed(_) => {
            return Err(syn::Error::new_spanned(
                &input.ident,
                "`CerulionState` cannot be derived for a tuple struct yet: the \
                 shape is keyed by FIELD NAME, which closes the transposition \
                 trap (swapping two same-typed fields must change the shape), \
                 and a tuple struct has none. Use a named-field struct.",
            ));
        }
    };
    expand_fields(input, &named, Target::Derived)
}

/// The ONE struct emission, shared by the derive and the `#[cerulion_node]`
/// fold-in so a node and a hand-derived helper cannot disagree about what a
/// field means.
fn expand_fields(
    input: &DeriveInput,
    named: &[&Field],
    target: Target,
) -> Result<TokenStream, syn::Error> {
    let __cer_root = crate_root::root();
    let named = named.iter().copied();
    let name_str = input.ident.to_string();
    let mut predicates = Vec::new();
    let mut shape_terms = Vec::new();
    let mut inline_terms = Vec::new();
    let mut min_terms = Vec::new();
    let mut probe_terms = Vec::new();
    let mut capture_stmts = Vec::new();
    let mut restore_stmts = Vec::new();
    let mut read_inits = Vec::new();
    let mut has_reconstruct = false;

    for field in named {
        let ident = field
            .ident
            .as_ref()
            .expect("named fields always carry an ident");
        let c = classify(field)?;
        let ty = c.ty;
        let fname = ident.to_string();

        match c.escape {
            Escape::Captured => {
                predicates.push(quote_spanned! {ty.span()=>
                    #ty: #__cer_root::state::CerulionState
                });
                shape_terms.push(quote! {
                    .field(#fname, <#ty as #__cer_root::state::CerulionState>::STATE_SHAPE)
                });
                inline_terms
                    .push(quote! { <#ty as #__cer_root::state::CerulionState>::INLINE_SAFE });
                min_terms
                    .push(quote! { <#ty as #__cer_root::state::CerulionState>::MIN_ENCODED_BYTES });
                probe_terms
                    .push(quote! { #__cer_root::state::CerulionState::cer_probe(&self.#ident) });
                capture_stmts.push(quote! {
                    #__cer_root::state::CerulionState::cer_capture(&self.#ident, out)?;
                });
                restore_stmts.push(quote! {
                    #__cer_root::state::CerulionState::cer_restore(&mut self.#ident, src)?;
                });
                read_inits.push(quote! {
                    #ident: <#ty as #__cer_root::state::CerulionState>::cer_read(src)?
                });
            }
            Escape::Reconstruct => {
                has_reconstruct = true;
                let marker = escape_shape(Escape::Reconstruct);
                shape_terms.push(quote! { .field(#fname, #marker) });
                // Captures nothing, restores nothing, contributes no floor —
                // and nothing that can block, so it does not disturb
                // INLINE_SAFE.
            }
            Escape::Serde => {
                predicates.push(quote_spanned! {ty.span()=>
                    #ty: #__cer_root::serde::Serialize
                });
                predicates.push(quote_spanned! {ty.span()=>
                    #ty: #__cer_root::serde::de::DeserializeOwned
                });
                let marker = escape_shape(Escape::Serde);
                shape_terms.push(quote! { .field(#fname, #marker) });
                // A user-written encoder can block, allocate, or run arbitrary
                // control flow, so the byte bound stops being a time bound.
                // FALSE, unconditionally.
                inline_terms.push(quote! { false });
                min_terms.push(quote! { 4usize });
                capture_stmts.push(quote! {
                    #__cer_root::state::capture_serde_field(&self.#ident, #fname, out)?;
                });
                restore_stmts.push(quote! {
                    self.#ident = #__cer_root::state::read_serde_field(src, #fname)?;
                });
                read_inits.push(quote! {
                    #ident: #__cer_root::state::read_serde_field(src, #fname)?
                });
            }
            Escape::Unordered => {
                let plan = unordered_plan(ty)?;
                let UnorderedPlan {
                    kind,
                    args,
                    container,
                } = &plan;
                for arg in args {
                    predicates.push(quote_spanned! {arg.span()=>
                        #arg: #__cer_root::state::CerulionState
                    });
                }
                let element_shapes = args.iter().map(|a| {
                    quote! { .element(<#a as #__cer_root::state::CerulionState>::STATE_SHAPE) }
                });
                shape_terms.push(quote! {
                    .field(
                        #fname,
                        #__cer_root::state::StateShape::of("cerulion::unordered")
                            #(#element_shapes)*
                            .finish(),
                    )
                });
                for arg in args {
                    inline_terms
                        .push(quote! { <#arg as #__cer_root::state::CerulionState>::INLINE_SAFE });
                }
                min_terms.push(quote! { 4usize });
                probe_terms.push(match kind {
                    UnorderedKind::Map => {
                        quote! { #__cer_root::state::probe_unordered_map(self.#ident.iter()) }
                    }
                    UnorderedKind::Set => {
                        quote! { #__cer_root::state::probe_unordered_set(self.#ident.iter()) }
                    }
                });
                match kind {
                    UnorderedKind::Map => {
                        capture_stmts.push(quote! {
                            #__cer_root::state::capture_unordered_map(
                                self.#ident.len(), self.#ident.iter(), out)?;
                        });
                        let build = quote! {{
                            let __cer_entries =
                                #__cer_root::state::read_unordered_entries(src)?;
                            let mut __cer_out = <#ty as ::std::default::Default>::default();
                            for (__cer_k, __cer_v) in __cer_entries {
                                if __cer_out.insert(__cer_k, __cer_v).is_some() {
                                    return ::std::result::Result::Err(
                                        #__cer_root::state::StateError::DuplicateEntry {
                                            type_name: #container,
                                        },
                                    );
                                }
                            }
                            __cer_out
                        }};
                        restore_stmts.push(quote! { self.#ident = #build; });
                        read_inits.push(quote! { #ident: #build });
                    }
                    UnorderedKind::Set => {
                        capture_stmts.push(quote! {
                            #__cer_root::state::capture_unordered_set(
                                self.#ident.len(), self.#ident.iter(), out)?;
                        });
                        let build = quote! {{
                            let __cer_elements =
                                #__cer_root::state::read_unordered_elements(src)?;
                            let mut __cer_out = <#ty as ::std::default::Default>::default();
                            for __cer_e in __cer_elements {
                                if !__cer_out.insert(__cer_e) {
                                    return ::std::result::Result::Err(
                                        #__cer_root::state::StateError::DuplicateEntry {
                                            type_name: #container,
                                        },
                                    );
                                }
                            }
                            __cer_out
                        }};
                        restore_stmts.push(quote! { self.#ident = #build; });
                        read_inits.push(quote! { #ident: #build });
                    }
                }
                predicates.push(quote_spanned! {ty.span()=>
                    #ty: ::std::default::Default
                });
            }
        }
    }

    let shape = quote! {
        #__cer_root::state::StateShape::of(#name_str)
            #(#shape_terms)*
            .finish()
    };
    let inline_safe = quote! { true #( && #inline_terms )* };
    let min_bytes = quote! { 0usize #( + #min_terms )* };

    // The inventory's own short circuit: `INLINE_SAFE` is `false` for every
    // lock, and it folds with AND, so `true` means no lock is reachable and
    // the walk can be skipped. That keeps the probe O(1) on the overwhelmingly
    // common lock-free node instead of O(state size) on the node thread once
    // per cadence — which would be worse than the cost the design exists to
    // avoid.
    let probe_body = quote! {
        if <Self as #__cer_root::state::CerulionState>::INLINE_SAFE {
            return true;
        }
        true #( && #probe_terms )*
    };

    // `let _ = out;` is not tidiness: a struct whose every field is escaped
    // writes nothing, and this workspace (and every scaffolded node crate)
    // sets `unused_variables = "deny"`, so an unused parameter in GENERATED
    // code is a hard error in the USER's build.
    let capture_unused = capture_stmts.is_empty().then(|| quote! { let _ = out; });
    let capture_body = quote! {
        #capture_unused
        #(#capture_stmts)*
        ::std::result::Result::Ok(())
    };

    // A struct with a `reconstruct` field cannot be CONSTRUCTED from a
    // recording — the recording deliberately carries nothing for that field —
    // so `cer_read` reports it rather than inventing a value. It is not the
    // path the runtime takes: `cer_restore` goes field by field and leaves the
    // escaped field alone. Requiring `Default` instead was rejected because a
    // handle type (`Arc<TransportManager>`) does not have one, and the
    // resulting `E0277` would name `Default` rather than the required diagnostic.
    //
    // A NODE takes that same arm UNCONDITIONALLY, and for a stronger reason
    // than a policy choice: its walk excludes ports and `__cer_rt`, so the
    // `Self { .. }` literal would be missing fields rustc requires
    // (`E0063`) — the code simply cannot be written. Falling back to
    // `..Default::default()` was rejected: it would silently mint a node with
    // DEFAULT ports and a default runtime context whenever anybody called
    // `cer_read`, and the restore path the runtime actually takes is
    // `init() -> cer_restore -> restored()`, which never calls it. Reporting
    // is the correct answer, and it is the one the derive already gives.
    let read_body = if has_reconstruct || target == Target::Node {
        quote! {
            let _ = src;
            ::std::result::Result::Err(
                #__cer_root::state::StateError::Unrestorable {
                    type_name: #name_str,
                },
            )
        }
    } else {
        let read_unused = read_inits.is_empty().then(|| quote! { let _ = src; });
        quote! {
            #read_unused
            ::std::result::Result::Ok(Self { #(#read_inits,)* })
        }
    };

    // Field by field, so an escaped field is left UNTOUCHED — the trait's own
    // doc calls that the decisive reason to override the default.
    let restore_unused = restore_stmts.is_empty().then(|| quote! { let _ = src; });
    let restore_body = quote! {
        #restore_unused
        #(#restore_stmts)*
        ::std::result::Result::Ok(())
    };

    Ok(emit_impl(
        input,
        predicates,
        shape,
        inline_safe,
        min_bytes,
        probe_body,
        capture_body,
        read_body,
        Some(restore_body),
    ))
}

#[derive(Debug)]
enum UnorderedKind {
    Map,
    Set,
}

#[derive(Debug)]
struct UnorderedPlan<'a> {
    kind: UnorderedKind,
    args: Vec<&'a Type>,
    container: String,
}

/// Work out what `#[cerulion(unordered)]` was put on.
///
/// Deliberately keyed on the container's LAST PATH SEGMENT, which is what the
/// user wrote at the field: the escape exists so a `HashMap` keyed by
/// something un-`Ord` (a float grid cell, a pose) can still be captured, and
/// the alternative — asking the type system — is exactly the `K: Ord`
/// obligation being escaped.
///
/// # `BTreeMap`/`BTreeSet` are REFUSED, and that is the point
///
/// The escape's advertised property is "no `Ord` on the key". A BTree
/// container cannot deliver it: the restore rebuilds the container with
/// `insert`, whose bound IS `K: Ord`, so the admissible key set is exactly the
/// plain impl's and the escape escapes nothing. MEASURED before the refusal
/// was written — `#[cerulion(unordered)] BTreeMap<f64, u8>` failed with
/// ``the trait bound `f64: Ord` is not satisfied ... required by a bound in
/// `BTreeMap::<K, V, A>::insert` ``, an error pointing into `std` for a field
/// the attribute had just promised to accept.
///
/// Nothing is lost by refusing them. A BTree container is ALREADY captured in
/// a deterministic order (it sorts nothing and pays no index), and its plain
/// impl already refuses duplicates on decode, so the escape's other half is
/// there too. What it would have bought is a DIFFERENT `STATE_SHAPE` for no
/// behavioural gain.
///
/// `IndexMap`/`IndexSet` stay: `insert` there needs only `Hash + Eq`, so the
/// no-`Ord` claim holds even though their plain impl already carries no `Ord`
/// bound either — the attribute is redundant on them, never false.
///
/// # A same-named container of the user's own lands in that refusal too
///
/// Keying on the name means the refusal cannot see through it, and this is
/// the one arm where that costs something REAL rather than theoretical:
/// MEASURED both directions, a `foo::BTreeMap<u32, u8>` whose `insert` needs
/// no `Ord` derived cleanly AND round-tripped under the pre-refusal arm, and
/// is refused today by a message that explains itself entirely in terms of
/// `std`'s `insert` — false of their type — while offering remedies that are
/// wrong for them (dropping the attribute leaves a container with no
/// `CerulionState`).
///
/// The refusal stays name-keyed anyway, for reasons that are not inertia. A
/// full-path rule cannot separate the two: `use std::collections::BTreeMap;`
/// is the overwhelmingly common spelling (it is what the compile-fail fixture
/// writes) and renders bare at the field, exactly like `use foo::BTreeMap;`,
/// so refusing only a `std::`-qualified path would miss the real `std` case
/// the refusal exists for while still catching a bare-imported custom one —
/// closing none of the class and adding an inconsistency, since every other
/// arm here (and `resource::is_resource`, and `STATE_SHAPE` itself) is keyed
/// the same way. It also fails CLOSED: a loud compile error at the attribute,
/// never a silent miscapture.
///
/// So the fix is the MESSAGE, and it is unconditional rather than gated on
/// the path having a foreign prefix — bare `BTreeMap` is itself ambiguous, so
/// there is no form of the refusal that can know which type it is looking at.
/// It names what is really matched (the name) and a remedy that EXISTS: both
/// `#[cerulion(serde)]` and a hand `CerulionState` impl were driven against a
/// custom `BTreeMap`-named container before being named here.
///
/// The remedies are BRANCHED rather than listed, and the order is part of the
/// contract. "Drop the attribute" is correct only for std's container; for a
/// same-named custom one it is a DEAD END (`E0277`: the type has no
/// `CerulionState`). Led with unscoped, it is the first instruction a custom
/// container's owner reads and follows, and they hit that dead end before ever
/// reaching the clause written for them. So the message states the KEYING
/// first, then offers each remedy inside the branch it serves, with the
/// custom-container remedies reachable without passing through the std one.
/// Pinned by `the_btree_refusal_tells_a_same_named_custom_container_what_to_do_instead`.
fn unordered_plan(ty: &Type) -> Result<UnorderedPlan<'_>, syn::Error> {
    let unsupported = |found: &str| {
        syn::Error::new(
            ty.span(),
            format!(
                "`#[cerulion(unordered)]` applies to a hash-like container, but this \
                 field is `{found}`. Supported: `HashMap`, `IndexMap`, `HashSet`, \
                 `IndexSet`. Every other field is already captured in a \
                 deterministic order and needs no escape."
            ),
        )
    };

    let Type::Path(path) = ty else {
        return Err(unsupported("not a path type"));
    };
    let segment = path
        .path
        .segments
        .last()
        .ok_or_else(|| unsupported("an empty path"))?;
    let name = segment.ident.to_string();
    let kind = match name.as_str() {
        "HashMap" | "IndexMap" => UnorderedKind::Map,
        "HashSet" | "IndexSet" => UnorderedKind::Set,
        // Named apart from the generic list so the message can say why this
        // ONE looks like it should work and cannot: refusing it inside
        // `unsupported` would print "Supported: ..." beside a container the
        // user can plainly see is a map.
        btree @ ("BTreeMap" | "BTreeSet") => {
            return Err(syn::Error::new(
                ty.span(),
                format!(
                    "`#[cerulion(unordered)]` cannot apply to `{btree}`. This is \
                     matched on the NAME you wrote — a macro sees tokens, not \
                     types — so which remedy is yours depends on which `{btree}` \
                     this is. If this is std's `{btree}`: the escape exists to lift \
                     the `Ord` obligation off the key, and rebuilding a `{btree}` \
                     needs `insert`, whose own bound is `Ord` — so the escape cannot \
                     lift anything here, and an un-`Ord` key would fail inside `std` \
                     instead. Drop the attribute: a `{btree}` is already captured in \
                     its own deterministic order and already refuses duplicates on \
                     decode; if the key really is un-`Ord`, the container has to be a \
                     `HashMap`/`HashSet` or an `IndexMap`/`IndexSet`. If instead this \
                     is a container of your OWN that merely shares the name: dropping \
                     the attribute would leave a field whose type has no \
                     `CerulionState`, so capture it with `#[cerulion(serde)]`, or \
                     implement `CerulionState` for it and leave the attribute off."
                ),
            ));
        }
        other => return Err(unsupported(other)),
    };
    let wanted = match kind {
        UnorderedKind::Map => 2,
        UnorderedKind::Set => 1,
    };

    let syn::PathArguments::AngleBracketed(generics) = &segment.arguments else {
        return Err(syn::Error::new(
            ty.span(),
            format!(
                "`#[cerulion(unordered)]` needs `{name}`'s element types spelled out at \
                 the field so the encoder can name them; write them explicitly \
                 rather than through a type alias."
            ),
        ));
    };
    let args: Vec<&Type> = generics
        .args
        .iter()
        .filter_map(|a| match a {
            syn::GenericArgument::Type(t) => Some(t),
            _ => None,
        })
        .take(wanted)
        .collect();
    if args.len() != wanted {
        return Err(syn::Error::new(
            ty.span(),
            format!("`{name}` needs {wanted} element type(s) spelled out at the field"),
        ));
    }

    Ok(UnorderedPlan {
        kind,
        args,
        container: name,
    })
}

/// The message for a `#[cerulion(...)]` key written on an enum variant's field.
///
/// BRANCHED on the key, and that is the fix rather than a flourish. The single
/// message this replaced ended "Move the field into a struct that carries the
/// escape, and hold that struct in the variant" for all three keys, and that
/// instruction is only true for two of them.
///
/// For `serde` and `unordered` the wrap works completely: the wrapper's
/// `cer_read` reconstructs those fields, so the variant round-trips.
///
/// For `reconstruct` it is a DEAD END, in the same way the `BTreeMap`
/// refusal's "drop the attribute" is a dead end for a same-named custom
/// container. A wrapper holding a `reconstruct` field reports
/// `StateError::Unrestorable` from `cer_read`, and an enum restores by calling
/// `cer_read`, so the variant can be captured and can never be restored.
/// MEASURED on the shape the old message asks for — 5 bytes captured, then
/// ``restore -> `ConnState` can be captured but not reconstructed from a
/// recording``. It fails LOUDLY, which is why it is still offered, but it is
/// offered second and with that cost stated, behind the remedy that actually
/// restores: hoist the handle OUT of the enum, so the enum carries only the
/// capturable part.
///
/// Pinned by `a_reconstruct_in_a_variant_leads_with_the_remedy_that_restores`.
fn variant_escape_refusal(escape: Escape) -> String {
    let key = match escape {
        Escape::Reconstruct => "reconstruct",
        Escape::Serde => "serde",
        Escape::Unordered => "unordered",
        Escape::Captured => unreachable!("parse never yields Captured"),
    };
    let head = format!(
        "`#[cerulion({key})]` is not supported on an enum variant's field. An enum \
         restores by replacing the whole value (its variant can change), so there is \
         no in-place field to leave untouched. "
    );
    let remedy = match escape {
        Escape::Reconstruct => {
            "A handle cannot be minted from bytes, so this variant has to stop holding \
             one. Hold the handle OUTSIDE the enum — a `#[cerulion(reconstruct)]` field \
             on the struct that owns the enum, re-opened in `fn restored(&mut self)` — \
             and leave the enum carrying only the capturable part. Wrapping the handle \
             in a struct that carries the escape and holding THAT in the variant does \
             compile, but the wrapper cannot be read back, so restoring this variant \
             then fails with `Unrestorable` every time — loudly, never silently, but it \
             never succeeds either."
        }
        Escape::Serde | Escape::Unordered | Escape::Captured => {
            "Move the field into a struct that carries the escape, and hold that struct \
             in the variant: that wrapper reads back, so the variant still round-trips."
        }
    };
    head + remedy
}

fn expand_enum(input: &DeriveInput, data: &syn::DataEnum) -> Result<TokenStream, syn::Error> {
    let __cer_root = crate_root::root();
    let name = &input.ident;
    let name_str = name.to_string();

    if data.variants.len() > MAX_ENUM_VARIANTS {
        return Err(syn::Error::new_spanned(
            name,
            format!(
                "`CerulionState` supports at most {MAX_ENUM_VARIANTS} variants (the \
                 recorded tag is one byte, matching `Option`/`Result`); this enum \
                 declares {}. Widening the tag would change the bytes of every enum \
                 already recorded, so it is refused rather than done silently.",
                data.variants.len()
            ),
        ));
    }

    let mut predicates = Vec::new();
    let mut variant_shapes = Vec::new();
    let mut inline_terms = Vec::new();
    let mut probe_arms = Vec::new();
    let mut capture_arms = Vec::new();
    let mut read_arms = Vec::new();

    for (index, variant) in data.variants.iter().enumerate() {
        let tag = u8::try_from(index).expect("variant count is bounded above");
        let vident = &variant.ident;
        let vname = vident.to_string();

        // A field escape inside a variant is refused rather than half-honoured:
        // an enum restores by REPLACING the value (its variant may change), so
        // there is no in-place field to leave untouched, which is the whole
        // meaning of `reconstruct`.
        for field in &variant.fields {
            if let Some(escape) = parse_field_escape(field)? {
                return Err(syn::Error::new(
                    field.span(),
                    variant_escape_refusal(escape),
                ));
            }
        }

        let tys: Vec<&Type> = variant.fields.iter().map(|f| &f.ty).collect();
        for ty in &tys {
            // `CerulionVariantMember`, not `CerulionState`: the obligation is
            // identical in force (blanket impl, every item forwarding) and its
            // `on_unimplemented` is the one that is TRUE inside an enum. The
            // real trait's own message names `#[cerulion(reconstruct)]`, which
            // the loop above refuses — a closed diagnostic loop until this
            // predicate changed. Every USE below names the same trait, because
            // a single `CerulionState` use anywhere in the emission raises its
            // own obligation and prints the misleading note as a second block
            // (MEASURED: switching all uses is what takes the count to one).
            predicates.push(quote_spanned! {ty.span()=>
                #ty: #__cer_root::state::CerulionVariantMember
            });
            inline_terms.push(
                quote! { <#ty as #__cer_root::state::CerulionVariantMember>::VARIANT_INLINE_SAFE },
            );
        }

        // A variant folds its NAME, its members and its ARITY, so adding a
        // field to a variant, reordering variants, or renaming one all change
        // the shape.
        //
        // A NAMED variant's members fold through `.field(name, ..)`, exactly
        // as a struct's do, because they carry the same transposition trap:
        // `V { a: u32, b: u32 }` and `V { b: u32, a: u32 }` capture and read in
        // DECLARATION order, so under a purely positional fold the two share a
        // shape and an old recording restores with the values swapped — the
        // trap `StateShape::field` exists to close and the reason a tuple
        // STRUCT is refused outright.
        //
        // A TUPLE variant keeps `.element(..)`: it has no names, so there is
        // nothing to fold that would separate `V(f64, f64)` from itself with
        // the two slots swapped. That residual is real and is the same one the
        // tuple-struct refusal names — it is left rather than closed by
        // refusing tuple variants too, which would reject shapes as ordinary
        // as `enum Msg { Point(f64, f64) }`. Slots of DIFFERENT types are
        // separated already, since their shapes fold in order.
        let member_shapes: Vec<TokenStream> = match &variant.fields {
            Fields::Named(named) => named
                .named
                .iter()
                .map(|f| {
                    let fname = f
                        .ident
                        .as_ref()
                        .expect("named fields carry an ident")
                        .to_string();
                    let ty = &f.ty;
                    quote! {
                        .field(
                            #fname,
                            <#ty as #__cer_root::state::CerulionVariantMember>::VARIANT_STATE_SHAPE,
                        )
                    }
                })
                .collect(),
            Fields::Unnamed(_) | Fields::Unit => tys
                .iter()
                .map(|ty| {
                    quote! {
                        .element(
                            <#ty as #__cer_root::state::CerulionVariantMember>::VARIANT_STATE_SHAPE,
                        )
                    }
                })
                .collect(),
        };
        let arity = tys.len();
        variant_shapes.push(quote! {
            .field(
                #vname,
                #__cer_root::state::StateShape::of(#vname)
                    #(#member_shapes)*
                    .count(#arity)
                    .finish(),
            )
        });

        match &variant.fields {
            Fields::Unit => {
                capture_arms.push(quote! {
                    Self::#vident => { out.write(&[#tag])?; }
                });
                probe_arms.push(quote! { Self::#vident => true });
                read_arms.push(quote! { #tag => ::std::result::Result::Ok(Self::#vident) });
            }
            Fields::Unnamed(unnamed) => {
                let binds: Vec<Ident> = (0..unnamed.unnamed.len())
                    .map(|i| format_ident!("__cer_{}", i))
                    .collect();
                let reads = unnamed.unnamed.iter().map(|f| {
                    let ty = &f.ty;
                    quote! {
                        <#ty as #__cer_root::state::CerulionVariantMember>::variant_cer_read(src)?
                    }
                });
                capture_arms.push(quote! {
                    Self::#vident( #(#binds),* ) => {
                        out.write(&[#tag])?;
                        #(
                            #__cer_root::state::CerulionVariantMember::variant_cer_capture(
                                #binds, out,
                            )?;
                        )*
                    }
                });
                probe_arms.push(quote! {
                    Self::#vident( #(#binds),* ) => {
                        true #(
                            && #__cer_root::state::CerulionVariantMember::variant_cer_probe(
                                #binds,
                            )
                        )*
                    }
                });
                read_arms.push(quote! {
                    #tag => ::std::result::Result::Ok(Self::#vident( #(#reads),* ))
                });
            }
            Fields::Named(named) => {
                let idents: Vec<&Ident> = named
                    .named
                    .iter()
                    .map(|f| f.ident.as_ref().expect("named"))
                    .collect();
                let reads = named.named.iter().map(|f| {
                    let id = f.ident.as_ref().expect("named");
                    let ty = &f.ty;
                    quote! {
                        #id: <#ty as #__cer_root::state::CerulionVariantMember>::variant_cer_read(src)?
                    }
                });
                capture_arms.push(quote! {
                    Self::#vident { #(#idents),* } => {
                        out.write(&[#tag])?;
                        #(
                            #__cer_root::state::CerulionVariantMember::variant_cer_capture(
                                #idents, out,
                            )?;
                        )*
                    }
                });
                probe_arms.push(quote! {
                    Self::#vident { #(#idents),* } => {
                        true #(
                            && #__cer_root::state::CerulionVariantMember::variant_cer_probe(
                                #idents,
                            )
                        )*
                    }
                });
                read_arms.push(quote! {
                    #tag => ::std::result::Result::Ok(Self::#vident { #(#reads),* })
                });
            }
        }
    }

    let variant_count = data.variants.len();
    let shape = quote! {
        #__cer_root::state::StateShape::of(#name_str)
            #(#variant_shapes)*
            .count(#variant_count)
            .finish()
    };
    let inline_safe = quote! { true #( && #inline_terms )* };
    // A tag byte is the floor: the cheapest variant is a unit one.
    let min_bytes = quote! { 1usize };

    let probe_body = if data.variants.is_empty() {
        // An uninhabited enum has no value, so no arm can run.
        quote! { true }
    } else {
        quote! {
            if <Self as #__cer_root::state::CerulionState>::INLINE_SAFE {
                return true;
            }
            match self { #(#probe_arms,)* }
        }
    };

    let capture_body = if data.variants.is_empty() {
        quote! {
            let _ = out;
            ::std::result::Result::Err(
                #__cer_root::state::StateError::Unrestorable { type_name: #name_str },
            )
        }
    } else {
        quote! {
            match self { #(#capture_arms)* }
            ::std::result::Result::Ok(())
        }
    };

    let read_body = quote! {
        let __cer_tag = src.take_array::<1>()?[0];
        match __cer_tag {
            #(#read_arms,)*
            __cer_other => ::std::result::Result::Err(
                #__cer_root::state::StateError::InvalidTag {
                    type_name: #name_str,
                    tag: __cer_other,
                },
            ),
        }
    };

    // No `cer_restore` override: the trait's default replaces the value, which
    // is the only correct thing for an enum whose VARIANT can change between
    // the recording and the restore. A field-wise in-place restore would have
    // to assume the running value is already in the recorded variant.
    Ok(emit_impl(
        input,
        predicates,
        shape,
        inline_safe,
        min_bytes,
        probe_body,
        capture_body,
        read_body,
        None,
    ))
}

#[cfg(test)]
mod tests;
