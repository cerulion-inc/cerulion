// SPDX-License-Identifier: AGPL-3.0-only
//! `#[cerulion_node_impl]` — AST-rewriting attribute macro for true zero-copy
//! tick bodies.
//!
//! Annotates an `impl <Name> { ... }` block adjacent to a `#[cerulion_node]`
//! struct. Parses the user's `tick` method body and rewrites every
//! `self.<port>` access to dispatch through a per-tick stack frame holding
//! either a lazy-loan slot (`LazyOutput<T>`) that vends an
//! `OutputProxy<T>` on the port's FIRST write (for `#[output]` ports) or an
//! SHM-backed [`InputView<T>`] reader handle (for `#[input]` ports). After
//! the rewrite, direct field assignments like `self.cmd_vel.linear.x = 0.3`
//! compile to a single `mov` of an `f64` straight into iceoryx2 SHM (the
//! loan on first write is a constant-cost `mov` too).
//!
//! # Why two macros?
//!
//! `#[cerulion_node]` annotates the struct definition; `#[cerulion_node_impl]`
//! annotates the impl block containing `tick`. They cooperate via a fixed
//! method name (`__cer_zero_copy_tick`) the impl macro emits and the struct
//! macro's generated `<Name>Entry::tick` calls. Now,
//! declarative-mode (any field carrying `#[input]`/`#[output]`) auto-routes
//! through this dispatch — there is no opt-in flag and no snapshot-buffered
//! fallback.
//!
//! # Discovery mechanism
//!
//! `#[cerulion_node_impl]` takes NO arguments. Every port name + type is
//! read from a per-build registry that `#[cerulion_node]` populates from the
//! sibling struct's `#[input]` / `#[output]` field attributes:
//!
//! ```text
//! #[cerulion_node]
//! struct Ctl {
//!     #[input(trigger)] scan: LaserScan,
//!     #[output] cmd: Twist,
//! }
//!
//! #[cerulion_node_impl]
//! impl Ctl {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.cmd.linear.x = 0.3; // direct SHM write
//!         Ok(())
//!     }
//! }
//! ```
//!
//! Source order matters: the `#[cerulion_node]` struct must appear before
//! its `#[cerulion_node_impl]` block so the registry lookup succeeds. The
//! impl macro emits a clear error when no registered struct matches.
//!
//! # Coverage
//!
//! - **Variable-schema outputs.** Variable-field setters (`set_<field>`,
//!   `loan_<field>`, `push_<field>`) and direct fixed-field assignment fall
//!   out of the `self.<port>` rewrite for free: the rewriter replaces
//!   `self.image` with the lazy get-or-loan receiver
//!   `__cer_image.__cer_loan()?` (`&mut OutputProxy<'_, Image>`), and
//!   `OutputProxy::DerefMut` lets `__cer_image.__cer_loan()?.set_data(&px)?`
//!   resolve to the codegen-emitted `ImageShm::set_data`. The user's `?`
//!   returns `TransportError`, which the user's `Result<(), NodeError>`
//!   accepts via the `#[from]` impl on `NodeError::Transport`.
//! - **Helper-method rewriting.** Every method on the
//!   impl block (not just `tick`) gets `self.<port>` rewritten. For helpers
//!   that reference ports, the macro injects per-port parameters (`&mut
//!   LazyOutput<'_, T>` for outputs, or `&InputView<'_, T>`) at the end of the helper
//!   signature and rewrites every `self.<helper>(args)` call inside `tick`
//!   (and inside other helpers) to pass the per-tick locals as those
//!   trailing arguments. The original (non-rewritten) helper is gone — this
//!   is correct because port fields on the user's struct are unit markers
//!   that have no usable accessors outside the per-tick proxy/view scope.
//! - **Multi-input zero-copy tick.** All input subscribers are taken
//!   from `ctx` upfront via the disjoint-borrow split
//!   `NodeContext::split_publishers_subscribers_mut` followed by
//!   `IndexMap::get_disjoint_mut` on each map, then wrapped in nested
//!   `try_view` closures (one per input) so every input view is in scope
//!   simultaneously inside the user body. The same disjoint split is used
//!   for outputs so multiple output proxies can coexist with input views
//!   without re-borrowing `&mut ctx`.
//!
//! # Coverage (out of scope still)
//!
//! - Helper methods that take generic / impl-trait receivers other than
//!   `&mut self`. The walker only rewrites methods whose first parameter is
//!   `&mut self` or `&self` — methods with explicit `Self` types or other
//!   generics are left untouched (and any port reference inside them will
//!   fail to compile, surfacing the misuse).

use proc_macro2::{Span, TokenStream as TokenStream2};
use quote::{format_ident, quote};
use std::collections::{BTreeSet, HashMap, HashSet};
use syn::spanned::Spanned;
use syn::visit::{self, Visit};
use syn::visit_mut::{self, visit_block_mut, VisitMut};
use syn::{
    parse_macro_input, parse_quote, parse_quote_spanned, Block, Expr, FnArg, Ident, ImplItem,
    ImplItemFn, ItemImpl, Member, Type,
};

use crate::crate_root;
use crate::determinism;
use crate::registry::{self, NodePortEntry};

/// Resolve the impl block's self-type to a single struct identifier.
///
/// Supports `impl Foo` and `impl path::To::Foo`. Returns `None` for
/// non-path types (`impl Box<Foo>` etc.) — those can't sensibly carry the
/// rewriter's per-tick locals and produce a clear error to the user.
fn impl_block_struct_ident(item_impl: &ItemImpl) -> Option<Ident> {
    let Type::Path(path) = item_impl.self_ty.as_ref() else {
        return None;
    };
    path.path.segments.last().map(|seg| seg.ident.clone())
}

/// Convert a registry entry into the `ImplAttr` shape the rewriter
/// consumes. Re-parses each port's stored type tokens — registry tokens
/// were produced by `to_token_stream` on a previously-parsed `syn::Type`,
/// so re-parse failures are programmer errors rather than user errors.
fn registry_to_impl_attr(entry: &NodePortEntry, item_impl: &ItemImpl) -> syn::Result<ImplAttr> {
    let mut inputs = Vec::with_capacity(entry.inputs.len());
    for p in &entry.inputs {
        let name = Ident::new(&p.name, item_impl.span());
        let ty = registry::parse_type_tokens(&p.type_tokens)
            .map_err(|e| syn::Error::new(item_impl.span(), e.to_string()))?;
        inputs.push(TypedPort { name, ty });
    }
    let mut outputs = Vec::with_capacity(entry.outputs.len());
    for p in &entry.outputs {
        let name = Ident::new(&p.name, item_impl.span());
        let ty = registry::parse_type_tokens(&p.type_tokens)
            .map_err(|e| syn::Error::new(item_impl.span(), e.to_string()))?;
        outputs.push(TypedPort { name, ty });
    }
    Ok(ImplAttr { inputs, outputs })
}

// ---------------------------------------------------------------------------
// Port info — auto-discovered from the registry written by `#[cerulion_node]`
// ---------------------------------------------------------------------------

/// One typed port declaration: `name + type`.
///
/// Always populated from the cross-macro registry (see `crate::registry`).
/// There is no `inputs(...)` / `outputs(...)`
/// attribute syntax: the user does not redeclare ports on both
/// macros; the impl macro reads them from the sibling
/// `#[cerulion_node]` directly.
struct TypedPort {
    name: Ident,
    ty: Type,
}

/// Resolved port set for the impl block — discovered from the registry.
///
/// The per-port variable-field tables are gone — the rewriter is
/// schema-blind. Every simple `self.<port>.<field> = expr` on a declared
/// OUTPUT port routes uniformly through the codegen-emitted
/// `__cer_assign_<field>` shim, which resolves fixed vs variable at compile
/// time inside the generated schema types.
struct ImplAttr {
    inputs: Vec<TypedPort>,
    outputs: Vec<TypedPort>,
}

// ---------------------------------------------------------------------------
// Visitor: collect port references in an expression tree
// ---------------------------------------------------------------------------

/// Walk an expression tree and collect every `self.<port>` reference where
/// `<port>` is a declared port. Used to determine which per-tick locals a
/// helper method needs as trailing parameters (helper-method rewriting).
struct PortRefCollector<'a> {
    port_idents: &'a HashSet<String>,
    referenced: BTreeSet<String>,
}

impl<'a, 'ast> Visit<'ast> for PortRefCollector<'a> {
    fn visit_expr(&mut self, expr: &'ast Expr) {
        if let Expr::Field(field_expr) = expr {
            if let Expr::Path(path_expr) = &*field_expr.base {
                if path_expr.path.is_ident("self") {
                    if let Member::Named(member) = &field_expr.member {
                        let name = member.to_string();
                        if self.port_idents.contains(&name) {
                            self.referenced.insert(name);
                        }
                    }
                }
            }
        }
        visit::visit_expr(self, expr);
    }
}

fn ports_referenced(block: &Block, port_idents: &HashSet<String>) -> BTreeSet<String> {
    let mut collector = PortRefCollector {
        port_idents,
        referenced: BTreeSet::new(),
    };
    collector.visit_block(block);
    collector.referenced
}

/// Walk an expression tree and collect every `self.<name>(args)` call
/// where `<name>` is one of `candidates`. Used to build the helper-helper
/// call graph for the transitive-port-set fixed point.
struct SelfCallCollector<'a> {
    candidates: &'a BTreeSet<String>,
    called: BTreeSet<String>,
}

impl<'a, 'ast> Visit<'ast> for SelfCallCollector<'a> {
    fn visit_expr(&mut self, expr: &'ast Expr) {
        if let Expr::MethodCall(call) = expr {
            if let Expr::Path(path) = &*call.receiver {
                if path.path.is_ident("self") {
                    let mname = call.method.to_string();
                    if self.candidates.contains(&mname) {
                        self.called.insert(mname);
                    }
                }
            }
        }
        visit::visit_expr(self, expr);
    }
}

fn self_method_calls(block: &Block, candidates: &BTreeSet<String>) -> BTreeSet<String> {
    let mut collector = SelfCallCollector {
        candidates,
        called: BTreeSet::new(),
    };
    collector.visit_block(block);
    collector.called
}

// ---------------------------------------------------------------------------
// Node-level determinism lint visitor
// ---------------------------------------------------------------------------

/// A single banned-symbol hit found while walking a method body: the matched
/// row plus the span to anchor the diagnostic at.
struct DeterminismHit {
    sym: &'static determinism::BannedSymbol,
    span: Span,
}

/// Walks a method body (read-only) and records every banned-non-deterministic
/// **DENY** symbol it can statically see. The macro half emits deny
/// errors only — WARN-class detection + surfacing is the deferred core half
/// (it re-walks node source at the CLI; there is no cross-process channel from
/// the proc-macro). So this visitor matches `match_deny` rows only; warn rows
/// (`env::var`, `fs::read_dir`, `thread::current().id()`, ...) are
/// deliberately not detected here. See `determinism.rs`'s "Warn-surfacing is
/// deferred" module note.
///
/// What it matches (and the false-positive trade):
///
/// - **Path calls** — `Expr::Call` whose callee is an `Expr::Path`. The
///   callee's LAST TWO path segments are matched strict against the DENY rows
///   (`Instant::now`, `std::time::Instant::now`, `SystemTime::now`,
///   `thread::spawn`). This is what makes both qualified and unqualified call
///   forms hit.
/// - **Calls inside macro arguments** — `tracing::info!("{:?}",
///   Instant::now())`, `format!`, `assert!`, etc. `syn::visit::Visit`
///   treats a macro's token stream as opaque, so the default walk never sees
///   inside it. `visit_macro` best-effort re-parses the macro tokens as a
///   comma-separated expression list and visits each — catching the common
///   "log a banned timestamp" form. Macros whose tokens do NOT parse as an
///   expression list (`asm!`, custom token DSLs) fall through silently — a
///   known, documented limitation (the same limitation the port
///   rewriter guard `try_emit_compile_error_for_macro_args` accepts).
///
/// What it deliberately does NOT match (no false positive):
///
/// - `Expr::MethodCall` on a value receiver — `self.timer.now()`,
///   `clock.now()`, `rng.thread_rng()` etc. Those are methods on user values,
///   not the banned free/associated functions, so they never consult the
///   path-tail table.
struct DeterminismLintVisitor {
    hits: Vec<DeterminismHit>,
}

impl DeterminismLintVisitor {
    fn new() -> Self {
        Self { hits: Vec::new() }
    }
}

/// Extract the last `n` path segment idents of `path` as a `Vec<String>` in
/// source order (e.g. `std::time::Instant::now` → `["Instant", "now"]` for
/// `n = 2`). Returns fewer than `n` only when the path is shorter.
fn last_path_segments(path: &syn::Path, n: usize) -> Vec<String> {
    let total = path.segments.len();
    let start = total.saturating_sub(n);
    path.segments
        .iter()
        .skip(start)
        .map(|seg| seg.ident.to_string())
        .collect()
}

impl<'ast> Visit<'ast> for DeterminismLintVisitor {
    fn visit_expr(&mut self, expr: &'ast Expr) {
        // ----- Path call: `Instant::now()`, `thread::spawn(...)`, etc. -----
        // DENY-only — the callee's LAST TWO path segments are matched strict
        // against `match_deny`. Warn rows are not consulted (warn detection is
        // the deferred core half; see the visitor doc).
        if let Expr::Call(call) = expr {
            if let Expr::Path(p) = call.func.as_ref() {
                let tail = last_path_segments(&p.path, 2);
                if let Some(sym) = determinism::match_deny(&tail) {
                    self.hits.push(DeterminismHit {
                        sym,
                        span: p
                            .path
                            .segments
                            .last()
                            .map(|s| s.ident.span())
                            .unwrap_or_else(|| call.func.span()),
                    });
                }
            }
        }
        // Recurse into children so nested calls (args, blocks, closures) are
        // also inspected. (Macro-argument calls are handled separately in
        // `visit_macro` — the default walk treats macro tokens as opaque.)
        visit::visit_expr(self, expr);
    }

    /// `syn::visit::Visit` treats a macro's token stream as
    /// opaque, so banned symbols inside macro arguments — the most common form,
    /// `tracing::info!("{:?}", Instant::now())` / `format!` / `assert!` —
    /// otherwise slip through clean. Best-effort re-parse the macro's tokens as
    /// a comma-separated expression list (the same shape the port
    /// rewriter guard `try_emit_compile_error_for_macro_args` uses) and visit each
    /// parsed expr through the normal `visit_expr` path so nested banned calls
    /// are caught.
    ///
    /// Known limitation (matches the existing note in
    /// `try_emit_compile_error_for_macro_args`): macros whose tokens are NOT a
    /// comma-separated expression list — `asm!`, custom token-tree DSLs — do
    /// not parse here and fall through silently. That is acceptable: those are
    /// rare in node bodies and a banned symbol would have to be embedded in a
    /// non-expression macro grammar to evade detection.
    ///
    /// We do NOT call the default `visit::visit_macro` (it would only re-walk
    /// the opaque token stream, finding nothing) — so each parsed expr is
    /// visited exactly once, no double-visit.
    fn visit_macro(&mut self, mac: &'ast syn::Macro) {
        let parser = syn::punctuated::Punctuated::<Expr, syn::Token![,]>::parse_terminated;
        let Ok(parsed_exprs) = syn::parse::Parser::parse2(parser, mac.tokens.clone()) else {
            return;
        };
        // The parsed exprs are OWNED locals (not tied to `'ast`), so we walk
        // them with a fresh sub-visitor and merge its hits — `visit_expr`
        // requires `&'ast Expr` and would not accept these short-lived
        // references. Spans on the parsed exprs still point back into the
        // original macro token stream (proc-macro2 preserves spans through
        // re-parse), so diagnostics anchor correctly at the user's source.
        let mut sub = DeterminismLintVisitor::new();
        for arg_expr in parsed_exprs.iter() {
            sub.visit_expr(arg_expr);
        }
        self.hits.append(&mut sub.hits);
    }
}

/// Run the determinism lint over a method body, returning every
/// DENY-class banned-symbol hit in source-walk order (including those hidden
/// inside macro arguments — see `DeterminismLintVisitor::visit_macro`). The
/// macro half is deny-only; warn detection is the deferred core half.
fn lint_determinism(block: &Block) -> Vec<DeterminismHit> {
    let mut visitor = DeterminismLintVisitor::new();
    visitor.visit_block(block);
    visitor.hits
}

// ---------------------------------------------------------------------------
// AST rewriter: self.<port> -> __cer_<port> + helper-call argument injection
// ---------------------------------------------------------------------------

/// Walks an expression tree and rewrites every `self.<port>` access to a
/// per-tick local. Also rewrites `self.<helper>(args)` call sites to pass
/// the appropriate `__cer_<port>` references when `<helper>` is a method
/// known to reference one or more ports (helper-method rewriting).
///
/// `port_idents` is the set of port names that should be rewritten when
/// they appear in `self.<port>` field-access position. Other `self.<x>`
/// accesses (regular struct fields like `self.min_safe_distance`) are left
/// untouched.
///
/// `helper_port_args` maps a helper method name to the (canonically
/// ordered) list of `(port_name, kind)` references the helper expects as
/// trailing parameters. Calls to those helpers get rewritten to pass the
/// per-tick locals as additional trailing arguments.
struct SelfPortRewriter<'a> {
    port_idents: &'a HashSet<String>,
    helper_port_args: &'a HashMap<String, Vec<(String, PortKind)>>,
    /// The declared OUTPUT port names. `self.<port>.<field> =
    /// expr` where `<port>` is in this set rewrites uniformly to
    /// `__cer_<port>.__cer_assign_<field>(&expr)?` (the codegen-emitted
    /// shim resolves fixed vs variable at compile time), and
    /// `self.<port>.<field>.fill_from(src)` to
    /// `__cer_<port>.__cer_fill_from_<field>(src)`. Replaces the earlier
    /// per-port variable-field tables — the rewriter is schema-blind.
    output_idents: &'a HashSet<String>,
    /// Number of simple port-field ASSIGNMENTS rewritten to a
    /// fallible `__cer_assign_<field>(…)?` call during this walk, plus the
    /// first offender. Consumed by `rewrite_helper_method`'s unit-return
    /// diagnostic (a `()` method can never use the `?` every rewritten
    /// assignment carries). Reads and user-written `fill_from` calls are
    /// deliberately NOT counted — the user sees those Results themselves.
    assign_rewrites: usize,
    first_assign: Option<(String, String)>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PortKind {
    Input,
    Output,
}

/// Peel `Expr::Paren` / `Expr::Group` wrappers so a structural match sees the
/// inner expression. Used to classify the RHS of a port-field assignment
/// (e.g. `(some_expr)` or a macro-inserted invisible group) by its real shape.
fn peel_groups(expr: &Expr) -> &Expr {
    let mut peeled = expr;
    loop {
        match peeled {
            Expr::Paren(p) => peeled = &p.expr,
            Expr::Group(g) => peeled = &g.expr,
            _ => return peeled,
        }
    }
}

impl<'a> SelfPortRewriter<'a> {
    /// Inspect a macro's raw token
    /// stream for `self.<port>.<field>` shapes on ANY declared port —
    /// fixed AND variable fields, INPUT and OUTPUT ports (the motivating
    /// case is a fixed-field READ of an `#[input]` port inside
    /// `tracing::debug!`). If found, rewrite the macro in place to emit a
    /// friendly `compile_error!` instead. Returns `true` iff the macro
    /// was rewritten.
    ///
    /// Without this, the access would reach rustc as a raw field access
    /// on the user struct's zero-sized port MARKER (the rewriter does not
    /// descend into macro token streams, so the `self.<port>` →
    /// `__cer_<port>` leaf rewrite never fires inside them) — a baffling
    /// E0609 pointing at macro internals. The fix the message prescribes
    /// is hoisting: bind the field to a local BEFORE the macro and pass
    /// the local.
    ///
    /// Coverage is partial — parses macro tokens as `Punctuated<Expr,
    /// Token![,]>`, which works for the majority of format / log
    /// macros (println!, eprintln!, format!, format_args!, dbg!,
    /// info!, debug!, warn!, error!, trace!, assert!, assert_eq!,
    /// tracing's `key = value` field syntax — an `Expr::Assign` —
    /// etc.) but NOT for macros with non-expression token shapes
    /// (asm!, sql!, quote!, custom DSLs). The latter silently fall
    /// through; the user gets whatever rustc produces for those.
    fn try_emit_compile_error_for_macro_args(&self, mac: &mut syn::Macro) -> bool {
        let tokens = mac.tokens.clone();
        let parser = syn::punctuated::Punctuated::<Expr, syn::Token![,]>::parse_terminated;
        let Ok(parsed_exprs) = syn::parse::Parser::parse2(parser, tokens) else {
            return false;
        };
        for arg_expr in parsed_exprs.iter() {
            let mut finder = PortFieldFinder {
                rewriter: self,
                found: None,
            };
            finder.visit_expr(arg_expr);
            if let Some((port_name, field_name)) = finder.found {
                // The ACTUAL macro path, rendered in full (`tracing::debug`,
                // not just `debug`) so the hoist suggestion is paste-ready.
                let macro_path = mac
                    .path
                    .segments
                    .iter()
                    .map(|s| s.ident.to_string())
                    .collect::<Vec<_>>()
                    .join("::");
                let macro_path = if macro_path.is_empty() {
                    "<macro>".to_string()
                } else {
                    macro_path
                };
                let msg = format!(
                    "self.{port_name}.{field_name} cannot appear inside a macro argument \
                     (here: `{macro_path}!`) — port field access is rewritten at expansion \
                     time and the rewriter does not descend into macro token streams. Hoist \
                     it to a local first: `let v = …; {macro_path}!(…, v);`."
                );
                // Replace the macro's path + tokens in place so the
                // resulting invocation is `::std::compile_error!("...")`
                // — works at both expression and statement positions.
                mac.path = syn::parse_quote!(::std::compile_error);
                mac.tokens = quote! { #msg };
                return true;
            }
        }
        false
    }

    /// Classify an arbitrary-depth `self.<port>.<f1>…<fN>.<leaf>`
    /// write path by PORT NAME alone — schema-blind (the earlier
    /// variable-field tables are gone). This is the sole port-WRITE classifier
    /// (the assign + fill_from arms); the macro-arg finder uses the
    /// 2-segment [`classify_any_port_field`](Self::classify_any_port_field)
    /// instead (it catches deep paths via recursive descent to their 2-segment
    /// prefix).
    ///
    /// Returns `Some((port, intermediates, leaf))` iff `expr` is a pure
    /// named-field access chain rooted at `self` (each level peeled of
    /// `Paren`/`Group`) with `<port>` a declared OUTPUT port. `intermediates`
    /// is `[f1, …, fN]`:
    ///   * EMPTY for the 2-segment `self.<port>.<leaf>` shape — the assign /
    ///     fill_from arms then take the UNCHANGED single-shim path (the
    ///     2-segment emission stays byte-identical);
    ///   * NON-empty for `self.<port>.<f1>…<fN>.<leaf>` — the arms emit the
    ///     recursive `__cer_with_nested_*` chain (the 2-segment rewrite alone falls through to the
    ///     leaf `self.<port>` rewrite + Deref; the nested rewrite owns the deeper write).
    ///
    /// Segment names keep their `r#` spelling (raw idents render as `r#type`);
    /// callers strip it before building the derived `__cer_*` idents, exactly
    /// like the 2-segment arms (`format_ident!` panics on a `r#…` string).
    /// Input ports and non-`self` bases never match.
    fn classify_port_path(&self, expr: &Expr) -> Option<(String, Vec<String>, String)> {
        let mut members = match_self_port_path_members(expr)?;
        // `members` is port-first: `[port, f1, …, fN, leaf]`, len >= 2.
        let port_name = members.remove(0);
        if !self.output_idents.contains(&port_name) {
            return None;
        }
        // After removing the port, `len >= 1`, so `pop` yields the leaf and
        // whatever remains (possibly empty) are the intermediates.
        let leaf = members
            .pop()
            .expect("match_self_port_path_members guarantees len >= 2");
        Some((port_name, members, leaf))
    }

    /// The 2-segment `self.<port>.<field>` classifier that matches
    /// ANY declared port — inputs AND outputs. Used ONLY by the
    /// macro-argument guard ([`PortFieldFinder`]): inside a foreign macro's
    /// tokens NO port access can work (the leaf rewrite never fires there),
    /// so input-port READS — legal everywhere else — are just as broken as
    /// output writes and deserve the same hoist-it diagnostic. The rewrite
    /// steps (assign / fill_from) use the OUTPUT-only, arbitrary-depth
    /// [`classify_port_path`](Self::classify_port_path); the finder catches
    /// deep paths via recursive descent to their 2-segment prefix.
    fn classify_any_port_field(&self, expr: &Expr) -> Option<(String, String)> {
        let (port_name, field_name) = match_self_port_field_shape(expr)?;
        if !self.port_idents.contains(&port_name) {
            return None;
        }
        Some((port_name, field_name))
    }
}

/// Structural half of the 2-segment port-field classifier: `Some((port,
/// field))` iff `expr` is EXACTLY `self.<port>.<field>` (single-segment field
/// path, with `Expr::Paren` / `Expr::Group` peeled) — membership in a port set
/// is the caller's business
/// ([`SelfPortRewriter::classify_any_port_field`] checks all ports; the
/// arbitrary-depth write classifier is
/// [`SelfPortRewriter::classify_port_path`]).
fn match_self_port_field_shape(expr: &Expr) -> Option<(String, String)> {
    let peeled = peel_groups(expr);
    let outer = match peeled {
        Expr::Field(f) => f,
        _ => return None,
    };
    let inner = match outer.base.as_ref() {
        Expr::Field(f) => f,
        _ => return None,
    };
    let self_path = match inner.base.as_ref() {
        Expr::Path(p) => p,
        _ => return None,
    };
    if !self_path.path.is_ident("self") {
        return None;
    }
    let port_member = match &inner.member {
        Member::Named(m) => m,
        _ => return None,
    };
    let field_member = match &outer.member {
        Member::Named(m) => m,
        _ => return None,
    };
    Some((port_member.to_string(), field_member.to_string()))
}

/// Structural half of
/// [`SelfPortRewriter::classify_port_path`]: peel the `Expr::Field` chain
/// rooted at `self`, returning the member names PORT-FIRST
/// (`[port, f1, …, fN, leaf]`) iff the whole expression is a pure
/// named-field access path on `self` carrying at least a port + one field.
/// `None` for any non-`self` base, a tuple-index member (`self.p.0`), or a
/// non-field node mid-chain (`self.p.foo()[i].bar`). Membership of `<port>`
/// in a port set is the caller's business.
fn match_self_port_path_members(expr: &Expr) -> Option<Vec<String>> {
    // Walk from the outermost field inward, collecting member names. The
    // chain `self.port.f1.f2.leaf` nests as
    //   Field{ base: Field{ base: … Field{ base: Path(self), member: port },
    //          member: f1 } …, member: leaf }
    // so descending yields members in OUTERMOST-first order
    // (`[leaf, fN, …, f1, port]`); we reverse to port-first below.
    let mut members: Vec<String> = Vec::new();
    let mut cur = peel_groups(expr);
    loop {
        match cur {
            Expr::Field(f) => {
                let name = match &f.member {
                    Member::Named(m) => m.to_string(),
                    Member::Unnamed(_) => return None,
                };
                members.push(name);
                cur = peel_groups(f.base.as_ref());
            }
            Expr::Path(p) => {
                if !p.path.is_ident("self") {
                    return None;
                }
                break;
            }
            _ => return None,
        }
    }
    // Require at least `[port, leaf]` (a bare `self` or `self.x` is not a
    // port-field path).
    if members.len() < 2 {
        return None;
    }
    members.reverse();
    Some(members)
}

/// The RECEIVER expression for an OUTPUT-port write site — the lazy
/// get-or-loan `__cer_<port>.__cer_loan()?`, yielding `&mut OutputProxy`. Every
/// uniform write shim (`__cer_assign_*`, `__cer_fill_from_*`,
/// `__cer_with_nested_*`) and every Deref method call (`set_*`, `loan_*`,
/// `push_*`, `with_*`) chains off it, so a port the tick never writes never
/// loans, and so never floods discard errors. `__cer_<port>` is a `&mut LazyOutput` in both
/// the tick frame (the preamble reborrow) and helper bodies (the injected
/// param), so `.__cer_loan()` resolves uniformly; the `?` propagates a loan
/// failure exactly as an eager preamble `loan_proxy()?` would.
fn output_write_receiver(port_name: &str, span: Span) -> Expr {
    let local = format_ident!("__cer_{}", port_name, span = span);
    parse_quote_spanned! { span => #local.__cer_loan()? }
}

/// Build the recursive `__cer_with_nested_*` closure chain for
/// a nested port-field write. `intermediates` (`[f1, …, fN]`, NON-empty, with
/// `r#` spelling preserved) nests as
///
/// ```text
/// <port_recv>.__cer_with_nested_f1(|__cer_v|
///     __cer_v.__cer_with_nested_f2(|__cer_v| … |__cer_v| <leaf_body>))
/// ```
///
/// where `port_recv` is the lazy-loan receiver
/// (`__cer_<port>.__cer_loan()?`, see [`output_write_receiver`]) and
/// `leaf_body` is the innermost call on `__cer_v` (the assign shim
/// `__cer_v.__cer_assign_<leaf>(…)` or the fill_from shim
/// `__cer_v.__cer_fill_from_<leaf>(…)`). The ONE closure-param ident
/// `__cer_v` is reused at every level: shadowing across nesting levels is
/// legal and keeps codegen uniform — each level only ever touches its own
/// `__cer_v`, and a `__cer_v` bound by an outer closure is never read inside
/// an inner one. Each `__cer_with_nested_<seg>` ident is `r#`-stripped (the
/// codegen names it with the RAW schema name; `format_ident!` panics on a
/// `r#…` string). Returns the chain WITHOUT a trailing `?` — the assign arm
/// wraps it in a hoisting block + `?`, the fill_from arm rides the user's
/// own `?` (both legs return `Result<(), TransportError>`).
fn build_nested_chain(
    port_recv: &Expr,
    intermediates: &[String],
    leaf_body: Expr,
    span: Span,
) -> Expr {
    debug_assert!(
        !intermediates.is_empty(),
        "build_nested_chain requires >= 1 intermediate (the 2-segment shape uses the single shim)"
    );
    let v = format_ident!("__cer_v", span = span);
    // Innermost closure — for the LAST intermediate `fN` — wraps the leaf.
    let mut closure: Expr = parse_quote_spanned! { span => |#v| #leaf_body };
    // Wrap each earlier intermediate's `__cer_with_nested_<seg>` outward,
    // from `f(N-1)` down to `f2` (skip `f1` — it applies to the port local).
    for seg in intermediates.iter().skip(1).rev() {
        let stripped = seg.strip_prefix("r#").unwrap_or(seg);
        let nested = format_ident!("__cer_with_nested_{}", stripped, span = span);
        closure = parse_quote_spanned! { span => |#v| #v.#nested(#closure) };
    }
    // Apply `f1`'s `__cer_with_nested_` method to the port receiver itself
    // (the lazy-loan `__cer_<port>.__cer_loan()?`).
    let first = &intermediates[0];
    let first_stripped = first.strip_prefix("r#").unwrap_or(first);
    let first_nested = format_ident!("__cer_with_nested_{}", first_stripped, span = span);
    parse_quote_spanned! { span => #port_recv.#first_nested(#closure) }
}

/// Walks an expression looking for `self.<port>.<field>` shapes
/// on ANY declared port (inputs and outputs — see
/// [`SelfPortRewriter::classify_any_port_field`]). `Visit` (not
/// `VisitMut`) — read-only, used to detect port-field usage inside
/// macro argument tokens that can't be modified safely by VisitMut.
struct PortFieldFinder<'a, 'r> {
    rewriter: &'a SelfPortRewriter<'r>,
    found: Option<(String, String)>,
}

impl<'a, 'r, 'ast> Visit<'ast> for PortFieldFinder<'a, 'r> {
    fn visit_expr(&mut self, expr: &'ast Expr) {
        if self.found.is_some() {
            return; // short-circuit after first hit
        }
        if let Some((port, field)) = self.rewriter.classify_any_port_field(expr) {
            self.found = Some((port, field));
            return;
        }
        visit::visit_expr(self, expr);
    }
}

impl<'a> VisitMut for SelfPortRewriter<'a> {
    fn visit_expr_mut(&mut self, expr: &mut Expr) {
        // Classify the two rewritten shapes BEFORE recursion so
        // the `self.<port>` → `__cer_<port>` leaf rewrite doesn't tear up
        // the nested pattern. Detection order:
        // 0. `Expr::Macro` with a port-field arg (
        //    ANY declared port, input or output).
        // 1. `self.<port>.<field>.fill_from(...)` → the uniform
        //    `__cer_fill_from_<field>` shim.
        // 2. `self.<port>.<field> = expr` → the uniform
        //    `__cer_assign_<field>` shim.
        // Every other shape (compound assign, borrow, non-fill_from method
        // call, bare read) falls through to the leaf rewrite: fixed fields
        // keep resolving through the Deref chain, and variable fields land
        // on the codegen-emitted write-only proxy whose gated operator
        // impls render the friendly `[i]` / `+=` diagnostics.

        // ----- 0. Port-field inside an Expr::Macro argument -----
        // Stmt::Macro (e.g., `println!(...);` as a standalone stmt) is
        // handled in `visit_stmt_mut` below — VisitMut wouldn't reach
        // Expr::Macro there because Stmt::Macro is a peer variant.
        if let Expr::Macro(em) = expr {
            if self.try_emit_compile_error_for_macro_args(&mut em.mac) {
                return;
            }
        }

        // ----- 1. fill_from method call -----
        if let Expr::MethodCall(call) = expr {
            if call.method == "fill_from" {
                if let Some((port_name, intermediates, leaf)) =
                    self.classify_port_path(&call.receiver)
                {
                    let span = call.method.span();
                    // The lazy get-or-loan receiver
                    // (`__cer_<port>.__cer_loan()?`) — the fill_from shim
                    // chains off it, so the loan happens on this write.
                    let recv = output_write_receiver(&port_name, span);
                    // Uniform shim — codegen resolves the simple-vs-complex
                    // target (`fill_from_<f>` vs `fill_from_<f>_bytes`)
                    // inside the generated `__cer_fill_from_<f>`.
                    //
                    // Keyword-named LEAF (`self.<port>.r#type`): the matched
                    // name renders as `r#type`, but codegen names the shim
                    // with the RAW schema name (the suffixed ident
                    // `__cer_fill_from_type` is never a keyword) — strip
                    // `r#`, or `format_ident!` panics on the invalid ident
                    // `__cer_fill_from_r#type`.
                    // Intermediates are stripped inside `build_nested_chain`.
                    let shim_field = leaf.strip_prefix("r#").unwrap_or(&leaf);
                    let method_ident = format_ident!("__cer_fill_from_{}", shim_field, span = span);
                    // Recurse into args first so any inner `self.<port>`
                    // references get rewritten. Args are deliberately NOT
                    // hoisted (unlike the `=` RHS): `fill_from` args are
                    // typically closures, and a same-port arg becomes a loud
                    // borrow error — acceptable, and better than silently
                    // reordering the producer's side effects vs the write.
                    let mut args = call.args.clone();
                    for arg in args.iter_mut() {
                        visit_mut::visit_expr_mut(self, arg);
                    }
                    // No trailing `?` on EITHER leg — the user's own `?` (if
                    // present) wraps this expression naturally. Adding `?`
                    // here would produce `(...?)?` (double `?`) when the user
                    // writes `fill_from(...)?`, and the outer `?` would apply
                    // to `()`. (This differs from the `=` rewrite which DOES
                    // add `?` because `Assign` has no user-visible result to
                    // attach `?` to.) The nested chain likewise returns
                    // `Result<(), TransportError>`, so it composes the same.
                    if intermediates.is_empty() {
                        // 2-segment `self.<port>.<leaf>.fill_from(args)` —
                        // the single-shim emission off the lazy-loan
                        // receiver.
                        *expr = parse_quote_spanned! {
                            span => #recv.#method_ident(#args)
                        };
                    } else {
                        // Nested
                        // `self.<port>.<f1>…<fN>.<leaf>.fill_from(args)` →
                        // the `__cer_with_nested_*` chain (rooted at the
                        // lazy-loan receiver) with the fill_from shim as the
                        // innermost leaf body.
                        let leaf_body: Expr = parse_quote_spanned! {
                            span => __cer_v.#method_ident(#args)
                        };
                        *expr = build_nested_chain(&recv, &intermediates, leaf_body, span);
                    }
                    return;
                }
            }
        }

        // ----- 2. Assign with a port-field LHS (the uniform shim) -----
        if let Expr::Assign(assign) = expr {
            if let Some((port_name, intermediates, leaf)) = self.classify_port_path(&assign.left) {
                let span = assign.eq_token.span;
                // Bookkeeping: the emitted call carries `?`, so a
                // surrounding method that cannot use `?` gets a targeted
                // diagnostic (see `rewrite_helper_method`). Nested writes
                // count too, and `first_assign` records the FULL dotted
                // path (`r#` spelling per segment) so the diagnostic names
                // exactly what the user wrote (`header.stamp.sec`).
                self.assign_rewrites += 1;
                if self.first_assign.is_none() {
                    let dotted = if intermediates.is_empty() {
                        leaf.clone()
                    } else {
                        let mut s = intermediates.join(".");
                        s.push('.');
                        s.push_str(&leaf);
                        s
                    };
                    self.first_assign = Some((port_name.clone(), dotted));
                }
                // The lazy get-or-loan receiver
                // (`__cer_<port>.__cer_loan()?`) — the assign shim (or the
                // nested `__cer_with_nested_*` chain) hangs off it, loaning the
                // proxy on this write.
                let recv = output_write_receiver(&port_name, span);
                // Keyword-named LEAF: strip the `r#` prefix — codegen names
                // the shim with the RAW schema name (`__cer_assign_type`),
                // and `format_ident!` panics on the invalid ident
                // `__cer_assign_r#type` (validate-pass HIGH; the
                // guard/diagnostic MESSAGES keep the user's `r#` spelling
                // via `first_assign`/the finder, deliberately). Intermediate
                // segments are stripped inside `build_nested_chain` (same
                // reason, for `__cer_with_nested_*`).
                let shim_field = leaf.strip_prefix("r#").unwrap_or(&leaf);
                let setter_name = format_ident!("__cer_assign_{}", shim_field, span = span);
                // Recurse into RHS so any self.<port> references inside
                // it get rewritten too.
                let mut rhs_box = assign.right.clone();
                visit_mut::visit_expr_mut(self, &mut rhs_box);
                // Pass the RHS to the shim, whose param type is `&T`
                // (fixed) / `&str` / `&[ElemTy]` (variable) — the same
                // by-reference shapes the old `set_*` targets took. Two
                // shapes, two forms — chosen so we keep the
                // OLD deref-coercion strictness (a `String`/`&str` RHS into a
                // `&[u8]` field stays a COMPILE ERROR, not a silent UTF-8
                // write) while staying clippy-clean:
                //   * RHS already a reference (`&payload[..]`), a string
                //     literal (`"g"`, type `&str`), or a byte-string literal
                //     (`b"..."`, type `&[u8; N]`) → pass it through unborrowed.
                //     All three are already references. Adding `&` here would
                //     double-borrow and trip `needless_borrow` (deny-level in
                //     CI); deref coercion still adapts `&[u8; N]`/`&String` to
                //     `&[u8]`/`&str`. (Byte-string literals matter because they
                //     parse as `syn::Lit::ByteStr`, NOT `Lit::Str` — a node
                //     writing `self.<bytefield> = b"..."` would otherwise fall
                //     to the else-branch and emit `__cer_assign_<f>(&(b"..."))`
                //     = `&&[u8; N]`, a needless borrow.)
                //   * any other RHS (owned `String`, `Vec<u8>`, `format!(..)`,
                //     a field/path) → borrow it; deref coercion adapts the
                //     owned value to the borrowed param.
                // NB: a uniform `AsRef::as_ref(&rhs)` would also silence the
                // lint but WIDENS the contract — `String: AsRef<[u8]>` makes
                // `byte_field = some_string` compile silently. The codebase's
                // "loud over silent / illegal states unrepresentable" rule
                // forbids that, hence the AST split.
                let already_borrowed = matches!(
                    peel_groups(&rhs_box),
                    Expr::Reference(_)
                        | Expr::Lit(syn::ExprLit {
                            lit: syn::Lit::Str(_) | syn::Lit::ByteStr(_),
                            ..
                        })
                );
                // HOIST the RHS into `__cer_rhs`
                // FIRST for BOTH the 2-segment and nested arms (MANDATORY, not
                // an optimization). Without the hoist a same-port read-back —
                // `self.out.y = self.out.x * 2.0` (2-segment) or
                // `self.imu.orientation.x = self.imu.orientation.w * 2.0`
                // (nested) — emits TWO overlapping mutable loans of the same
                // `__cer_<port>`: the write receiver's
                // `__cer_<port>.__cer_loan()?` AND the RHS's own
                // `__cer_<port>.__cer_loan()?` (reading an OUTPUT field loans
                // it, once). Both are EXPLICIT `&mut` `.__cer_loan()`
                // calls, NOT receiver autorefs, so two-phase borrows do NOT
                // rescue them:
                //   * 2-segment → E0499 (two `&mut` borrows of `__cer_<port>`
                //     live at once inside the single call expression);
                //   * nested    → E0502 (the RHS read is captured by the
                //     `__cer_with_nested_*` closure while `__cer_<port>` is
                //     already mutably borrowed by the chain receiver).
                // Hoisting the RHS into a `let __cer_rhs = …;` binding SEQUENCES
                // the two loans (the RHS loan is taken + released before the
                // write loan is taken) and matches Rust's rhs-first assignment
                // evaluation order exactly, so it is source-faithful. Because
                // rhs is evaluated BEFORE the `let` binds, a user `__cer_rhs`
                // inside rhs is unaffected (moot anyway — `__cer` is a reserved
                // prefix). Borrow adaptation applies to the temp, reusing
                // `already_borrowed` above.
                let rhs_ident = format_ident!("__cer_rhs", span = span);
                let adapted: Expr = if already_borrowed {
                    parse_quote_spanned! { span => #rhs_ident }
                } else {
                    parse_quote_spanned! { span => &(#rhs_ident) }
                };
                if intermediates.is_empty() {
                    // 2-segment `self.<port>.<leaf> = rhs` — the single shim off
                    // the lazy-loan receiver, wrapped in the SAME
                    // hoisting block as the nested arm below.
                    *expr = parse_quote_spanned! {
                        span => {
                            let #rhs_ident = #rhs_box;
                            #recv.#setter_name(#adapted)?
                        }
                    };
                } else {
                    // Nested `self.<port>.<f1>…<fN>.<leaf> = rhs` — the
                    // `__cer_with_nested_*` closure chain (rooted at the
                    // lazy-loan receiver) with the assign shim as the innermost
                    // leaf body, over the same hoisted RHS temp.
                    let leaf_body: Expr = parse_quote_spanned! {
                        span => __cer_v.#setter_name(#adapted)
                    };
                    let chain = build_nested_chain(&recv, &intermediates, leaf_body, span);
                    *expr = parse_quote_spanned! {
                        span => {
                            let #rhs_ident = #rhs_box;
                            #chain?
                        }
                    };
                }
                return;
            }
        }

        // Recurse first so nested patterns get rewritten before we inspect
        // the current node: `self.helper(self.cmd.linear.x)` needs the
        // inner `self.cmd` rewrite to land before we touch the outer call.
        visit_mut::visit_expr_mut(self, expr);

        // 1. Method call on `self`: `self.helper(args...)` — inject the
        //    per-tick locals as trailing args when `helper` is known to
        //    transitively reference ports.
        //
        // `__cer_<port>` is a reference type at every call site (
        // `&mut LazyOutput<'_, T>` for outputs — the lazy slot, NOT a
        // pre-loaned proxy — or `&InputView<'_, T>` for inputs), so we pass
        // the bare local. Rust auto-reborrows `&mut` references for function
        // calls, which is the magic that lets the same call expression work
        // from tick (where the local is the original reborrow) and from
        // another helper (where the local is itself a parameter of reference
        // type). A helper that never writes a port it was handed never loans
        // it — the write shims inside the helper route through
        // `__cer_<port>.__cer_loan()?` identically to the tick.
        if let Expr::MethodCall(call) = expr {
            if let Expr::Path(path) = &*call.receiver {
                if path.path.is_ident("self") {
                    let helper_name = call.method.to_string();
                    if let Some(extra) = self.helper_port_args.get(&helper_name) {
                        for (port, _kind) in extra {
                            let local = format_ident!("__cer_{}", port, span = call.method.span());
                            let arg: Expr = parse_quote_spanned! {
                                call.method.span() => #local
                            };
                            call.args.push(arg);
                        }
                    }
                }
            }
        }

        // 2. Field access on self: `self.<port>` — rewrite to the per-tick
        //    local. INPUT ports become the bare `__cer_<port>` (`&InputView`);
        //    OUTPUT ports become the lazy get-or-loan receiver
        //    `__cer_<port>.__cer_loan()?` (`&mut OutputProxy`) so a method-call
        //    receiver (`self.cmd.set_data(...)`) or a rare bare use loans on
        //    demand — an output never reached here never loans.
        if let Expr::Field(field_expr) = expr {
            if let Expr::Path(path_expr) = &*field_expr.base {
                if path_expr.path.is_ident("self") {
                    if let Member::Named(member) = &field_expr.member {
                        let name = member.to_string();
                        if self.output_idents.contains(&name) {
                            *expr = output_write_receiver(&name, member.span());
                        } else if self.port_idents.contains(&name) {
                            let local = format_ident!("__cer_{}", name, span = member.span());
                            *expr = parse_quote_spanned! { member.span() => #local };
                        }
                    }
                }
            }
        }
    }

    /// Handle `Stmt::Macro` (e.g., `println!(...);`
    /// as a standalone stmt) — `VisitMut::visit_expr_mut` only sees
    /// `Expr::Macro`, which is the in-expression-position case. Macros
    /// used as statements take this path.
    fn visit_stmt_mut(&mut self, stmt: &mut syn::Stmt) {
        if let syn::Stmt::Macro(stmt_macro) = stmt {
            if self.try_emit_compile_error_for_macro_args(&mut stmt_macro.mac) {
                return;
            }
        }
        visit_mut::visit_stmt_mut(self, stmt);
    }
}

// ---------------------------------------------------------------------------
// `#[on_event(input|output = "...")]` handler discovery
// ---------------------------------------------------------------------------

/// The reactable event kinds a `#[on_event]` handler may bind to, routed from
/// the handler method's event PARAMETER TYPE. Each kind fixes (a)
/// the required filter scope — input vs output — and (b) the
/// `NodeContext::take_*` accessor the generated dispatch calls.
///
/// `TickWithin` is deliberately NOT covered: it emits no event (counter-only
/// until the clock model). `Liveliness` IS covered:
/// input-scoped, routed from a `LivelinessEvent` parameter.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum EventKind {
    /// `BackpressureEvent` — input-scoped. Accessor:
    /// `ctx.take_backpressure_event(name)`.
    Backpressure,
    /// `ExpectWithinEvent` — input-scoped. Accessor:
    /// `ctx.take_expect_within_event(name)`.
    ExpectWithin,
    /// `PromiseWithinEvent` — output-scoped. Accessor:
    /// `ctx.take_promise_within_event(name)`.
    PromiseWithin,
    /// `LivelinessEvent` — input-scoped. Accessor:
    /// `ctx.take_liveliness_event(name)`.
    Liveliness,
}

impl EventKind {
    /// Map the event parameter's last type-path segment ident to a kind.
    /// `None` for any unrecognized type — the collector turns that into a
    /// compile error naming the valid types.
    fn from_type_ident(ident: &str) -> Option<Self> {
        match ident {
            "BackpressureEvent" => Some(Self::Backpressure),
            "ExpectWithinEvent" => Some(Self::ExpectWithin),
            "PromiseWithinEvent" => Some(Self::PromiseWithin),
            "LivelinessEvent" => Some(Self::Liveliness),
            _ => None,
        }
    }

    /// Human-readable event type name (for diagnostics).
    fn type_name(self) -> &'static str {
        match self {
            Self::Backpressure => "BackpressureEvent",
            Self::ExpectWithin => "ExpectWithinEvent",
            Self::PromiseWithin => "PromiseWithinEvent",
            Self::Liveliness => "LivelinessEvent",
        }
    }

    /// `true` if this event is scoped to an `#[input]` port (so the handler
    /// filter must be `input = "..."`); `false` for output-scoped events
    /// (filter must be `output = "..."`).
    fn is_input_scoped(self) -> bool {
        match self {
            Self::Backpressure | Self::ExpectWithin | Self::Liveliness => true,
            Self::PromiseWithin => false,
        }
    }

    /// The `NodeContext` accessor ident the dispatch calls for this kind.
    fn accessor_ident(self) -> Ident {
        let name = match self {
            Self::Backpressure => "take_backpressure_event",
            Self::ExpectWithin => "take_expect_within_event",
            Self::PromiseWithin => "take_promise_within_event",
            Self::Liveliness => "take_liveliness_event",
        };
        Ident::new(name, Span::call_site())
    }
}

/// One discovered `#[on_event(input|output = "X")]` handler: the port name it
/// reacts to, the routed [`EventKind`], and the user's handler method ident.
struct EventHandler {
    port: String,
    kind: EventKind,
    method: Ident,
}

/// Scan the impl block's methods for `#[on_event(input|output = "...")]`,
/// route each by its handler method's event PARAMETER TYPE, parse + validate
/// each, and STRIP the attribute off the method (so rustc doesn't error with
/// "cannot find attribute `on_event`" on the emitted impl — there is no real
/// proc-macro attribute by that name).
///
/// **The handler may not fire if the underlying QoS is not configured.** The
/// macro cannot see the graph topology at expansion time (the cross-crate /
/// cross-macro registry gap): a `BackpressureEvent` handler attached to an
/// input declared `drop_oldest`/`block` (rather than `sample(N)`), or an
/// `ExpectWithin`/`PromiseWithin` handler on a port with no
/// `expect_within_ms`/`promise_within_ms` window, validates and compiles here
/// but the runtime never queues an event, so the handler stays silent. Attach
/// handlers to ports whose QoS actually produces the event.
///
/// Validation (all surfaced as compile errors anchored at the offending
/// token; diagnostics ACCUMULATE so multiple problems on one method report in
/// one compile pass):
/// - the handler signature MUST be exactly `(&mut self, ev: <EventType>)` —
///   wrong arity or a missing/non-receiver first arg is rejected, and the
///   event type MUST be one of `BackpressureEvent` / `ExpectWithinEvent` /
///   `PromiseWithinEvent` / `LivelinessEvent`;
/// - the attribute MUST carry exactly one of `input = "<lit>"` /
///   `output = "<lit>"` — neither, both, an unknown key, or a non-string
///   literal is rejected;
/// - the filter scope must MATCH the event kind: input-scoped events
///   (`Backpressure`, `ExpectWithin`, `Liveliness`) require `input = "..."`;
///   output-scoped (`PromiseWithin`) requires `output = "..."`;
/// - the referenced port must be declared on the sibling `#[cerulion_node]`
///   struct — input-scoped ports in `#[input]` fields, output-scoped in
///   `#[output]` fields;
/// - a method may carry at most one `#[on_event]` attribute;
/// - the dedup key is `(port, event-kind)`: two handlers on the SAME port
///   with DIFFERENT event kinds are allowed (e.g. a `BackpressureEvent` and
///   an `ExpectWithinEvent` both on input `"imu"`); two with the SAME
///   `(port, kind)` is rejected (the dispatch drains a single pending event
///   per accessor per tick).
///
/// Returns the validated handler list in SOURCE order (the determinism
/// guarantee — dispatch order = declaration order). The handler methods
/// themselves are left in the impl (minus the stripped attribute) and emitted
/// as ordinary `&mut self` methods.
fn collect_event_handlers(
    item_impl: &mut ItemImpl,
    input_idents: &HashSet<String>,
    output_idents: &HashSet<String>,
) -> syn::Result<Vec<EventHandler>> {
    let mut handlers: Vec<EventHandler> = Vec::new();
    // Dedup key is (port, event-kind): different kinds on the same port are
    // allowed, the same (port, kind) is not.
    let mut seen: HashMap<(String, EventKind), Span> = HashMap::new();

    // Accumulate diagnostics so several malformed / duplicate `#[on_event]`
    // attributes on one method all report in a single compile pass, instead
    // of first-error-wins-drop (which forces the user to
    // fix-recompile-discover the next one).
    fn accumulate(slot: &mut Option<syn::Error>, e: syn::Error) {
        match slot {
            Some(acc) => acc.combine(e),
            None => *slot = Some(e),
        }
    }

    for item in &mut item_impl.items {
        let ImplItem::Fn(method) = item else {
            continue;
        };

        // Skip methods carrying no `#[on_event]` at all (cheap pre-check so we
        // don't touch handler-signature inference / attribute stripping on
        // every ordinary helper or `tick`).
        if !method
            .attrs
            .iter()
            .any(|attr| attr.path().is_ident("on_event"))
        {
            continue;
        }

        // A handler attached to a RESERVED lifecycle method (`tick` /
        // `init` / `shutdown`) or one in the `__cer_*` macro namespace is also
        // rewritten by the lifecycle/shim branch of the top-level rewrite loop
        // (see the loop near `mname == "tick"`). The generated dispatch would
        // then call `self.tick(ev)` — a method the rewrite already consumed —
        // producing a confusing post-expansion E0599. Reject it here with a
        // clean, actionable error anchored at the offending method name.
        let mname = method.sig.ident.to_string();
        if matches!(mname.as_str(), "tick" | "init" | "shutdown") {
            return Err(syn::Error::new(
                method.sig.ident.span(),
                format!(
                    "#[on_event] cannot be attached to the reserved `{mname}` method; \
                     move the handler to a separate method"
                ),
            ));
        }
        if mname.starts_with("__cer_") {
            return Err(syn::Error::new(
                method.sig.ident.span(),
                "#[on_event] cannot be attached to a method in the reserved \
                 `__cer_` namespace (used by `#[cerulion_node_impl]` for \
                 macro-injected shims); move the handler to a separate method",
            ));
        }

        let attr_span = method.sig.ident.span();
        let mut parse_error: Option<syn::Error> = None;

        // ---- Parse the filter (`input` / `output`) from each attribute ----
        // Track each separately: at most one filter overall, exactly one of
        // input/output, and count attribute occurrences so the "at most one
        // per method" rule fires even when an EARLIER attribute is malformed
        // (matches the on_event_duplicate_malformed_first contract).
        #[derive(Clone, Copy)]
        enum FilterScope {
            Input,
            Output,
        }
        let mut parsed_filter: Option<(FilterScope, String, Span)> = None;
        let mut on_event_count = 0usize;
        method.attrs.retain(|attr| {
            if !attr.path().is_ident("on_event") {
                return true;
            }
            on_event_count += 1;
            // Parse exactly one of `input = "<lit>"` / `output = "<lit>"`.
            // Reject anything else.
            let mut found_input: Option<String> = None;
            let mut found_output: Option<String> = None;
            let res = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("input") {
                    // A repeated key on ONE attribute (`input = "a",
                    // input = "b"`) silently last-wins without this guard — the
                    // second parse overwrites the first. Reject it so the user
                    // picks exactly one port instead of getting the (arbitrary)
                    // last value.
                    if found_input.is_some() {
                        return Err(meta.error(
                            "#[on_event] sets `input` more than once; \
                             specify exactly one port",
                        ));
                    }
                    let value = meta.value()?;
                    let lit: syn::LitStr = value.parse().map_err(|_| {
                        meta.error(
                            "#[on_event] `input` must be a string literal, \
                             e.g. #[on_event(input = \"imu\")]",
                        )
                    })?;
                    found_input = Some(lit.value());
                    Ok(())
                } else if meta.path.is_ident("output") {
                    if found_output.is_some() {
                        return Err(meta.error(
                            "#[on_event] sets `output` more than once; \
                             specify exactly one port",
                        ));
                    }
                    let value = meta.value()?;
                    let lit: syn::LitStr = value.parse().map_err(|_| {
                        meta.error(
                            "#[on_event] `output` must be a string literal, \
                             e.g. #[on_event(output = \"cmd\")]",
                        )
                    })?;
                    found_output = Some(lit.value());
                    Ok(())
                } else {
                    Err(meta.error(
                        "#[on_event] accepts exactly one of `input = \"<port>\"` \
                         or `output = \"<port>\"`; unexpected key",
                    ))
                }
            });
            if let Err(e) = res {
                accumulate(&mut parse_error, e);
                return false; // strip regardless
            }
            match (found_input, found_output) {
                (Some(_), Some(_)) => {
                    accumulate(
                        &mut parse_error,
                        syn::Error::new(
                            attr_span,
                            "#[on_event] accepts exactly one of `input = \"<port>\"` \
                             or `output = \"<port>\"`, not both",
                        ),
                    );
                }
                (Some(name), None) => {
                    // Keep the FIRST valid filter for the checks below; the
                    // at-most-one rule is enforced post-loop via the count.
                    if parsed_filter.is_none() {
                        parsed_filter = Some((FilterScope::Input, name, attr_span));
                    }
                }
                (None, Some(name)) => {
                    if parsed_filter.is_none() {
                        parsed_filter = Some((FilterScope::Output, name, attr_span));
                    }
                }
                (None, None) => {
                    accumulate(
                        &mut parse_error,
                        syn::Error::new(
                            attr_span,
                            "#[on_event] requires exactly one of `input = \"<port>\"` \
                             or `output = \"<port>\"`, \
                             e.g. #[on_event(input = \"imu\")]",
                        ),
                    );
                }
            }
            false // strip the attribute from the emitted method
        });

        if on_event_count > 1 {
            accumulate(
                &mut parse_error,
                syn::Error::new(
                    attr_span,
                    "a method may carry at most one #[on_event] \
                     attribute; split the handlers across separate methods",
                ),
            );
        }

        // ---- Route the event kind from the handler's parameter type ----
        // Handler must be exactly `(&mut self, ev: <EventType>)`: first arg a
        // receiver, exactly one further typed arg, type's last path segment
        // one of the three event idents. Surfaced as an accumulating error so
        // a signature problem reports alongside any filter problem.
        let routed_kind: Option<EventKind> = match resolve_event_kind(method) {
            Ok(k) => Some(k),
            Err(e) => {
                accumulate(&mut parse_error, e);
                None
            }
        };

        if let Some(e) = parse_error {
            return Err(e);
        }
        // Both must have succeeded to continue (errors returned above).
        let (Some((scope, port, span)), Some(kind)) = (parsed_filter, routed_kind) else {
            continue;
        };

        // ---- Filter scope must MATCH the routed event kind ----
        match (scope, kind.is_input_scoped()) {
            (FilterScope::Input, true) | (FilterScope::Output, false) => {}
            (FilterScope::Output, true) => {
                return Err(syn::Error::new(
                    span,
                    format!(
                        "{ty} is an input-scoped event; use `input = \"...\"`, \
                         not `output = ...`",
                        ty = kind.type_name()
                    ),
                ));
            }
            (FilterScope::Input, false) => {
                return Err(syn::Error::new(
                    span,
                    format!(
                        "{ty} is an output-scoped event; use `output = \"...\"`, \
                         not `input = ...`",
                        ty = kind.type_name()
                    ),
                ));
            }
        }

        // ---- The referenced port must be declared on the sibling struct ----
        if kind.is_input_scoped() {
            if !input_idents.contains(&port) {
                return Err(syn::Error::new(
                    span,
                    format!(
                        "#[on_event(input = \"{port}\")] references an unknown input — \
                         `{port}` is not declared as an `#[input]` field on the sibling \
                         `#[cerulion_node]` struct"
                    ),
                ));
            }
        } else if !output_idents.contains(&port) {
            return Err(syn::Error::new(
                span,
                format!(
                    "#[on_event(output = \"{port}\")] references an unknown output — \
                     `{port}` is not declared as an `#[output]` field on the sibling \
                     `#[cerulion_node]` struct"
                ),
            ));
        }

        // ---- Dedup on (port, kind) ----
        let key = (port.clone(), kind);
        if let Some(prev) = seen.get(&key) {
            let mut err = syn::Error::new(
                span,
                format!(
                    "duplicate #[on_event] handler for {ty} on `{port}` — each \
                     (port, event-kind) pair may have at most one handler (the dispatch \
                     drains a single pending event per accessor per tick). Handlers for \
                     DIFFERENT event kinds on the same port are allowed.",
                    ty = kind.type_name()
                ),
            );
            err.combine(syn::Error::new(
                *prev,
                format!(
                    "first {ty} handler for `{port}` declared here",
                    ty = kind.type_name()
                ),
            ));
            return Err(err);
        }
        seen.insert(key, span);

        handlers.push(EventHandler {
            port,
            kind,
            method: method.sig.ident.clone(),
        });
    }

    Ok(handlers)
}

/// Route an `#[on_event]` handler's [`EventKind`] from its signature.
///
/// The handler must be exactly `fn h(&mut self, ev: <EventType>)`: the first
/// parameter a `self` receiver, exactly one further typed parameter, and that
/// parameter's type's last path segment one of the reactable event type idents.
/// Any deviation is a compile error anchored at the signature (or the
/// offending parameter), naming the valid types.
fn resolve_event_kind(method: &ImplItemFn) -> syn::Result<EventKind> {
    let valid = "valid event types are `BackpressureEvent`, `ExpectWithinEvent`, \
                 `PromiseWithinEvent`, and `LivelinessEvent`";
    let mut args = method.sig.inputs.iter();
    // First arg must be a `&mut self` (or `&self`) receiver.
    match args.next() {
        Some(FnArg::Receiver(_)) => {}
        _ => {
            return Err(syn::Error::new_spanned(
                &method.sig,
                format!(
                    "#[on_event] handler must take `&mut self` plus exactly one \
                     event parameter, e.g. `fn on_evt(&mut self, ev: BackpressureEvent)`; {valid}"
                ),
            ));
        }
    }
    // Exactly one further typed parameter.
    let event_arg = match (args.next(), args.next()) {
        (Some(FnArg::Typed(pt)), None) => pt,
        _ => {
            return Err(syn::Error::new_spanned(
                &method.sig,
                format!(
                    "#[on_event] handler must take exactly one event parameter after \
                     `&mut self`, e.g. `fn on_evt(&mut self, ev: BackpressureEvent)`; {valid}"
                ),
            ));
        }
    };
    // The parameter type's last path segment must be a known event ident.
    let Type::Path(type_path) = event_arg.ty.as_ref() else {
        return Err(syn::Error::new_spanned(
            &event_arg.ty,
            format!("#[on_event] handler parameter has an unrecognized event type; {valid}"),
        ));
    };
    let last = type_path.path.segments.last().ok_or_else(|| {
        syn::Error::new_spanned(
            &event_arg.ty,
            format!("#[on_event] handler parameter has an unrecognized event type; {valid}"),
        )
    })?;
    match EventKind::from_type_ident(&last.ident.to_string()) {
        Some(k) => Ok(k),
        None => Err(syn::Error::new_spanned(
            &event_arg.ty,
            format!(
                "#[on_event] handler parameter type `{}` is not a reactable event; {valid}",
                last.ident
            ),
        )),
    }
}

// ---------------------------------------------------------------------------
// Top-level entry: rewrite the impl block
// ---------------------------------------------------------------------------

pub fn cerulion_node_impl(
    attr: proc_macro::TokenStream,
    item: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    let __cer_root = crate_root::root();
    // `#[cerulion_node_impl]` takes NO arguments. The
    // port list is discovered from the registry that `#[cerulion_node]`
    // populates from its sibling struct's `#[input]`/`#[output]` field
    // attrs. Reject any args with an actionable error.
    if !attr.is_empty() {
        return syn::Error::new(
            proc_macro2::TokenStream::from(attr).span(),
            "#[cerulion_node_impl] takes no arguments — input/output ports are auto-discovered \
             from the sibling `#[cerulion_node]` struct's `#[input]`/`#[output]` field attributes",
        )
        .to_compile_error()
        .into();
    }

    let mut item_impl = parse_macro_input!(item as ItemImpl);

    // Resolve the impl block's `Self` type to a struct identifier and look
    // it up in the cross-macro registry.
    let struct_ident = match impl_block_struct_ident(&item_impl) {
        Some(id) => id,
        None => {
            return syn::Error::new_spanned(
                &item_impl.self_ty,
                "#[cerulion_node_impl] requires `impl <StructName> { ... }` over a path type so \
                 the macro can look up the sibling `#[cerulion_node]` struct's port declarations",
            )
            .to_compile_error()
            .into();
        }
    };

    let entry = match crate::registry::lookup(&struct_ident.to_string()) {
        Some(e) => e,
        None => {
            return syn::Error::new_spanned(
                &item_impl.self_ty,
                format!(
                    "#[cerulion_node_impl] could not find a registered #[cerulion_node] for \
                     `{struct_ident}`. The struct's `#[cerulion_node]` attribute must appear \
                     before this impl block in source order, and at least one field must carry \
                     a `#[input]` or `#[output]` attribute."
                ),
            )
            .to_compile_error()
            .into();
        }
    };

    let impl_attr = match registry_to_impl_attr(&entry, &item_impl) {
        Ok(a) => a,
        Err(e) => return e.to_compile_error().into(),
    };

    let port_idents: HashSet<String> = impl_attr
        .inputs
        .iter()
        .chain(impl_attr.outputs.iter())
        .map(|p| p.name.to_string())
        .collect();

    // Lookup map for port type / kind by name — used when injecting
    // helper parameters and per-tick locals.
    let mut port_type_by_name: HashMap<String, (Type, PortKind)> = HashMap::new();
    for p in &impl_attr.inputs {
        port_type_by_name.insert(p.name.to_string(), (p.ty.clone(), PortKind::Input));
    }
    for p in &impl_attr.outputs {
        port_type_by_name.insert(p.name.to_string(), (p.ty.clone(), PortKind::Output));
    }

    // Discover (and strip) `#[on_event(input|output =
    // "...")]` handlers BEFORE the helper-port pre-walk / method-rewrite loop
    // runs. Stripping early keeps rustc from choking on the unknown attribute
    // and keeps the handler methods out of the port-rewriter's helper
    // machinery (they take an event type, not per-tick proxies). Dispatch for
    // each is emitted at the tail of the generated `__cer_zero_copy_tick`,
    // type-routed to the matching `ctx.take_*_event` accessor.
    let input_idents: HashSet<String> = impl_attr
        .inputs
        .iter()
        .map(|p| p.name.to_string())
        .collect();
    let output_idents: HashSet<String> = impl_attr
        .outputs
        .iter()
        .map(|p| p.name.to_string())
        .collect();
    let event_handlers = match collect_event_handlers(&mut item_impl, &input_idents, &output_idents)
    {
        Ok(h) => h,
        Err(e) => return e.to_compile_error().into(),
    };
    // Names of the `#[on_event]` handler methods. They are invoked directly
    // from the tick tail as `self.<handler>(event)` — NOT through the
    // helper-call-rewrite path — so they must be EXCLUDED from the
    // helper-port machinery (otherwise the rewriter would inject per-tick
    // proxy params they never receive at the dispatch call site, producing
    // an arity mismatch). They are emitted verbatim as ordinary methods.
    let handler_method_names: HashSet<String> = event_handlers
        .iter()
        .map(|h| h.method.to_string())
        .collect();

    // -----------------------------------------------------------------
    // Node-level determinism lint.
    //
    // Walk every method body (tick, init, shutdown, helpers, AND the
    // `#[on_event]` handlers — they all run user code) for banned
    // non-deterministic symbols. This MUST run BEFORE any body rewrite so
    // the diagnostic spans point at the user's source, not at
    // macro-injected code. `#[on_event]` attributes were already stripped
    // by `collect_event_handlers`, but bodies are untouched at this point.
    //
    // DENY hits become a combined `compile_error!` PREPENDED to the
    // still-emitted expansion (mirroring `validate.rs` /
    // `cerulion_node`'s error-emission pattern) so downstream "cannot find
    // type / method" errors don't cascade on top of the real diagnostic.
    //
    // Suppression (the locked determinism contract), read from the registry entry
    // the sibling `#[cerulion_node]` wrote:
    // - `allow_non_deterministic` → suppress EVERYTHING (every deny + every
    //   warn, regardless of IO-class).
    // - `uses_live_io` → suppress ONLY IO-class rows (`io: true`, e.g. the
    //   `fs::read_dir` warn); non-IO denies (time/thread) and non-IO warns
    //   still apply. (Severity and IO-class are orthogonal — today the only
    //   IO-class row is a warn, so `uses_live_io` suppresses a warn here, but
    //   the gate is `io`, not severity.)
    //
    // WARN-class symbols are NOT detected or surfaced by the macro half:
    // stable Rust has no proc-macro warn API, and there is no cross-process
    // channel from the proc-macro to the separate graph-load CLI (the
    // proc-macro registry is process-local — anything recorded there is gone
    // by graph load). So the macro half emits DENY errors only; warn detection
    // + surfacing is the deferred core half (it re-walks node source at the
    // CLI). The `lint_determinism` visitor collects deny hits only — see
    // `determinism.rs`'s "Warn-surfacing is deferred" module note.
    //
    // The combined DENY error is NOT early-returned here — it is carried in
    // `determinism_deny_error` and PREPENDED to the FINAL (fully rewritten)
    // expansion at the bottom of this function. Letting the normal rewrite
    // still run means the generated `<Name>Entry` wrapper's `__cer_*`
    // shim calls resolve and `self.<port>` Deref accessors exist, so the
    // user sees ONLY the actionable determinism `compile_error!` — not an
    // E0599/E0609 cascade about macro-internal method/field names.
    let determinism_deny_error: Option<syn::Error> = {
        // `allow_non_deterministic` suppresses every row. `uses_live_io`
        // suppresses ONLY IO-class rows (`sym.io`). (Today every deny row is
        // non-IO, so `uses_live_io` never suppresses a deny — but the gate is
        // `io`, not severity, so it stays correct if an IO-class deny is ever
        // added... which the `determinism.rs` const contract forbids. Kept
        // defensive + symmetric with the deferred warn path.)
        let suppressed = |sym: &determinism::BannedSymbol| -> bool {
            if entry.allow_non_deterministic {
                return true;
            }
            entry.uses_live_io && sym.io
        };

        let mut deny_error: Option<syn::Error> = None;
        for item in &item_impl.items {
            let ImplItem::Fn(method) = item else {
                continue;
            };
            for hit in lint_determinism(&method.block) {
                if suppressed(hit.sym) {
                    continue;
                }
                // `lint_determinism` returns deny hits only; assert it so a
                // future visitor change that starts emitting warn hits is
                // caught here rather than silently dropping them.
                debug_assert!(
                    hit.sym.class.is_deny(),
                    "lint_determinism must return deny hits only (warn detection \
                     is the deferred core half)"
                );
                let err = syn::Error::new(hit.span, hit.sym.message);
                match &mut deny_error {
                    Some(acc) => acc.combine(err),
                    None => deny_error = Some(err),
                }
            }
        }

        deny_error
    };

    // -----------------------------------------------------------------
    // Pre-walk every method to figure out which ports each helper
    // *transitively* uses. Direct references collected via the
    // `PortRefCollector` visitor; transitive references propagated
    // through helper-helper calls (`self.<other_helper>(args)`) via
    // a fixed-point iteration over the call graph.
    // -----------------------------------------------------------------
    //
    // Step 1: per helper, collect the set of ports referenced *directly*
    // and the set of helpers it *calls* via `self.<callee>(...)`. We
    // restrict to methods that take `&self` or `&mut self` — methods
    // with a non-receiver `Self` type can't sensibly hold per-tick
    // proxy references, so we don't pull them into the call graph.
    let mut direct_refs: HashMap<String, BTreeSet<String>> = HashMap::new();
    let mut helper_callees: HashMap<String, BTreeSet<String>> = HashMap::new();
    let mut all_helpers: BTreeSet<String> = BTreeSet::new();
    for item in &item_impl.items {
        if let ImplItem::Fn(method) = item {
            let name = method.sig.ident.to_string();
            if name == "tick" {
                continue;
            }
            // `#[on_event]` handlers are dispatched directly
            // and are not part of the per-tick proxy call graph.
            if handler_method_names.contains(&name) {
                continue;
            }
            if !first_arg_is_self_receiver(method) {
                continue;
            }
            all_helpers.insert(name.clone());
            direct_refs.insert(name.clone(), ports_referenced(&method.block, &port_idents));
            helper_callees.insert(name.clone(), self_method_calls(&method.block, &all_helpers));
        }
    }
    // The visitor pass above sees `all_helpers` only as it's being
    // populated, so a method calling a helper declared *later* in the
    // impl block would miss the edge. Re-collect callees with the full
    // helper set to fix that.
    helper_callees.clear();
    for item in &item_impl.items {
        if let ImplItem::Fn(method) = item {
            let name = method.sig.ident.to_string();
            if name == "tick" {
                continue;
            }
            if handler_method_names.contains(&name) {
                continue;
            }
            if !first_arg_is_self_receiver(method) {
                continue;
            }
            helper_callees.insert(name.clone(), self_method_calls(&method.block, &all_helpers));
        }
    }

    // Step 2: fixed-point — port set per helper is direct refs ∪ port
    // sets of every helper it calls. Iterate until stable.
    let mut transitive_refs: HashMap<String, BTreeSet<String>> = direct_refs.clone();
    loop {
        let mut changed = false;
        for h in all_helpers.iter() {
            let callees = helper_callees.get(h).cloned().unwrap_or_default();
            let mut new_set = transitive_refs.get(h).cloned().unwrap_or_default();
            for callee in callees {
                if let Some(callee_set) = transitive_refs.get(&callee) {
                    for p in callee_set {
                        if new_set.insert(p.clone()) {
                            changed = true;
                        }
                    }
                }
            }
            transitive_refs.insert(h.clone(), new_set);
        }
        if !changed {
            break;
        }
    }

    // Step 3: build `helper_port_args` — for each helper that
    // transitively references at least one port, materialize the
    // canonically ordered (port, kind) list to use both for parameter
    // injection and for call-site argument injection.
    let mut helper_port_args: HashMap<String, Vec<(String, PortKind)>> = HashMap::new();
    for h in &all_helpers {
        let refs = transitive_refs.get(h).cloned().unwrap_or_default();
        if refs.is_empty() {
            continue;
        }
        // Canonical ordering: outputs in declaration order, then inputs
        // in declaration order. Keeps generated signatures stable.
        let mut ordered = Vec::new();
        for p in impl_attr.outputs.iter().chain(impl_attr.inputs.iter()) {
            let pname = p.name.to_string();
            if refs.contains(&pname) {
                if let Some((_, kind)) = port_type_by_name.get(&pname) {
                    ordered.push((pname, *kind));
                }
            }
        }
        helper_port_args.insert(h.clone(), ordered);
    }

    // -----------------------------------------------------------------
    // Find `tick` (required), `init` and `shutdown` (optional), and
    // rewrite every method (helpers + tick + lifecycle).
    //
    // User-written `init(&mut self, ctx: &mut NodeContext)
    // -> Result<(), NodeError>` and `shutdown(&mut self) -> Result<(),
    // NodeError>` methods are renamed to `__cer_user_init` /
    // `__cer_user_shutdown`. The struct-macro-generated `<Name>Entry`
    // wrapper always calls those shim names — when the user didn't
    // write either, this code appends no-op stubs so the wrapper's call
    // resolves regardless. Keeps `init` and `shutdown` truly optional
    // as the node API promises.
    // -----------------------------------------------------------------
    let mut found_tick = false;
    let mut found_user_init = false;
    let mut found_user_shutdown = false;
    // An `#[cerulion_node(external)]` node must define an
    // `external_source` method (and a non-external node must NOT). Track whether
    // it was seen + successfully renamed to `__cer_user_external_source`;
    // `external_source_error` collects the actionable diagnostic (policy
    // mismatch / bad signature / missing) prepended to the final expansion, and
    // `external_source_renamed` gates the stub injection below so the
    // codegen-emitted `external_source()` override's call still resolves in the
    // error case (no E0599 cascade, the clean-diagnostic pattern).
    let mut found_external_source = false;
    let mut external_source_renamed = false;
    let mut external_source_error: Option<syn::Error> = None;
    for item in &mut item_impl.items {
        if let ImplItem::Fn(method) = item {
            let mname = method.sig.ident.to_string();
            // Reserve the `__cer_*` namespace
            // so a user-written method whose name happens to collide
            // with a macro-injected one (e.g. `__cer_user_init`,
            // `__cer_zero_copy_tick`) gets a clear error pointing at
            // their method instead of a confusing "duplicate definition"
            // cascade after macro expansion.
            if mname.starts_with("__cer_") {
                return syn::Error::new_spanned(
                    &method.sig.ident,
                    "method names starting with `__cer_` are reserved by \
                     `#[cerulion_node_impl]` for macro-injected shims. \
                     Rename this method to something else.",
                )
                .to_compile_error()
                .into();
            }
            if mname == "tick" {
                found_tick = true;
                rewrite_tick_method(
                    method,
                    &impl_attr,
                    &port_idents,
                    &output_idents,
                    &helper_port_args,
                    &event_handlers,
                );
            } else if mname == "init" {
                found_user_init = true;
                rewrite_user_init_method(method);
            } else if mname == "shutdown" {
                found_user_shutdown = true;
                rewrite_user_shutdown_method(method);
            } else if mname == "external_source" {
                // The user's external-ingress source method.
                found_external_source = true;
                if !entry.external {
                    // Policy mismatch: `external_source` on a non-external node.
                    // The node has no `external_source()` override generated, so
                    // there is no `__cer_user_external_source` call to resolve —
                    // leave the method in place (harmless) and surface only our
                    // diagnostic.
                    external_source_error.get_or_insert_with(|| {
                        syn::Error::new_spanned(
                            &method.sig.ident,
                            format!(
                                "`external_source` is only valid on an \
                                 `#[cerulion_node(external)]` node, but `{struct_ident}` has no \
                                 `external` attribute. Remove this method, or add `external` to \
                                 the struct's `#[cerulion_node(...)]`."
                            ),
                        )
                    });
                } else if let Err(e) = validate_external_source_sig(method) {
                    // Wrong signature: collect the error and DON'T rename — the
                    // post-loop stub injection provides a correctly-typed
                    // `__cer_user_external_source` so the wrapper's call resolves.
                    external_source_error.get_or_insert(e);
                } else {
                    rewrite_user_external_source_method(method);
                    external_source_renamed = true;
                }
            } else if handler_method_names.contains(&mname) {
                // `#[on_event]` handler — emitted verbatim.
                // It is dispatched by name from the tick tail with the routed
                // event; it holds no per-tick proxies, so it must NOT be
                // rewritten.
            } else if helper_port_args.contains_key(&mname) {
                rewrite_helper_method(
                    method,
                    &port_idents,
                    &output_idents,
                    &helper_port_args,
                    &port_type_by_name,
                );
            } else if first_arg_is_self_receiver(method) {
                // Helpers that don't reference ports still need their
                // call sites updated if they call OTHER helpers that do —
                // re-walk for call-site rewrites only (the SelfPortRewriter
                // is a no-op on bodies with no `self.<port>` references).
                let mut rewriter = SelfPortRewriter {
                    port_idents: &port_idents,
                    helper_port_args: &helper_port_args,
                    output_idents: &output_idents,
                    assign_rewrites: 0,
                    first_assign: None,
                };
                visit_block_mut(&mut rewriter, &mut method.block);
            }
        }
    }

    if !found_tick {
        return syn::Error::new_spanned(
            &item_impl,
            "#[cerulion_node_impl] requires a `tick` method on the impl block",
        )
        .to_compile_error()
        .into();
    }

    if !found_user_init {
        let stub: ImplItem = parse_quote! {
            #[doc(hidden)]
            #[inline]
            fn __cer_user_init(
                &mut self,
                _ctx: &mut #__cer_root::graph::node::NodeContext,
            ) -> #__cer_root::error::TransportResult<()> {
                ::std::result::Result::Ok(())
            }
        };
        item_impl.items.push(stub);
    }
    if !found_user_shutdown {
        let stub: ImplItem = parse_quote! {
            #[doc(hidden)]
            #[inline]
            fn __cer_user_shutdown(
                &mut self,
            ) -> #__cer_root::error::TransportResult<()> {
                ::std::result::Result::Ok(())
            }
        };
        item_impl.items.push(stub);
    }

    // An `#[cerulion_node(external)]` node MUST define an
    // `external_source` method. Missing it is our own clear diagnostic (stable
    // across toolchains — it goes in the trybuild BLOCKING group). We span the
    // whole impl block, mirroring the missing-`tick` diagnostic.
    if entry.external && !found_external_source {
        external_source_error.get_or_insert_with(|| {
            syn::Error::new_spanned(
                &item_impl,
                format!(
                    "#[cerulion_node(external)] node `{struct_ident}` must define an \
                     `external_source` method on its `#[cerulion_node_impl]` block: \
                     `fn external_source(&mut self) -> ExternalSource`. Return \
                     `ExternalSource::HostDriven` if the node fires only via a host \
                     `trigger_external`, or `ExternalSource::Fd(fd)` / \
                     `ExternalSource::Blocking(..)` to self-trigger from a device fd or a \
                     blocking SDK."
                ),
            )
        });
    }
    // Inject a `__cer_user_external_source` stub whenever an external node's user
    // method is absent OR had a wrong signature (so it wasn't renamed). The
    // codegen-emitted `external_source()` override calls
    // `self.inner.__cer_user_external_source()`; without this stub that call
    // would E0599-cascade over the actionable `external_source_error` we prepend
    // below (the clean-diagnostic pattern). NOTE: this only covers the
    // impl-block-PRESENT case — a `#[cerulion_node(external)]` struct with NO
    // `#[cerulion_node_impl]` block at all never reaches this macro, so it falls
    // back to rustc's cryptic E0599 for both `__cer_zero_copy_tick` and
    // `__cer_user_external_source` (an accepted backstop — such a node also has
    // no `tick` and cannot compile regardless).
    if entry.external && !external_source_renamed {
        let stub: ImplItem = parse_quote! {
            #[doc(hidden)]
            #[inline]
            fn __cer_user_external_source(
                &mut self,
            ) -> #__cer_root::graph::node::ExternalSource {
                #__cer_root::graph::node::ExternalSource::HostDriven
            }
        };
        item_impl.items.push(stub);
    }

    // Prepend the combined `compile_error!`s (if any)
    // to the FULLY-rewritten expansion. Because the rewrite + stub injection
    // already ran, the generated shims/accessors all exist — the user sees ONLY
    // the actionable diagnostic(s), with no macro-internal E0599/E0609 cascade.
    let mut error_tokens = TokenStream2::new();
    if let Some(err) = determinism_deny_error {
        error_tokens.extend(err.to_compile_error());
    }
    if let Some(err) = external_source_error {
        error_tokens.extend(err.to_compile_error());
    }
    quote! {
        #error_tokens
        #item_impl
    }
    .into()
}

/// Validate the user's `external_source` signature. It must be
/// `fn external_source(&mut self) -> ExternalSource` — the runtime calls it once
/// (no args beyond the receiver) and binds the returned source. The return type
/// is matched on its LAST path segment, so a qualified path
/// (`cerulion_core::graph::node::ExternalSource`, `node::ExternalSource`, …) is
/// accepted as well as the bare `ExternalSource`. Every signature modifier
/// (`async`, `const`, `unsafe`, `extern "C"`, generic/lifetime/const params, a
/// `where`-clause, or a `...` variadic) is rejected — the runtime binds a plain
/// `fn`, so any modifier would only fail downstream with a confusing error.
/// Returns a clear, span-anchored `compile_error!` on any mismatch (our own
/// diagnostic — blocking trybuild group).
fn validate_external_source_sig(method: &ImplItemFn) -> syn::Result<()> {
    let sig = &method.sig;
    let bad = |span_src: &dyn quote::ToTokens| {
        syn::Error::new_spanned(
            span_src,
            "`external_source` must be declared `fn external_source(&mut self) -> \
             ExternalSource` — it is queried once at run_live entry to obtain this node's \
             ExternalSource (no parameters beyond the receiver; returns `ExternalSource` by \
             value — a qualified path like `cerulion_core::graph::node::ExternalSource` is \
             accepted, matched on the last path segment).",
        )
    };
    // Reject every signature modifier — the runtime binds a plain
    // `fn external_source(&mut self) -> ExternalSource`. Any modifier changes the
    // callee shape (e.g. `async` makes it future-returning, `extern "C"` an
    // FFI-ABI fn) and would otherwise slip past this validator only to fail
    // downstream with a confusing error, so reject each here with the same
    // diagnostic, spanning the offending token where cheap.
    if let Some(constness) = &sig.constness {
        return Err(bad(constness));
    }
    if let Some(asyncness) = &sig.asyncness {
        return Err(bad(asyncness));
    }
    if let Some(unsafety) = &sig.unsafety {
        return Err(bad(unsafety));
    }
    if let Some(abi) = &sig.abi {
        return Err(bad(abi));
    }
    if !sig.generics.params.is_empty() {
        return Err(bad(&sig.generics));
    }
    if let Some(where_clause) = &sig.generics.where_clause {
        return Err(bad(where_clause));
    }
    if let Some(variadic) = &sig.variadic {
        return Err(bad(variadic));
    }
    // Receiver must be `&mut self` (by reference, mutable, untyped).
    let mut inputs = sig.inputs.iter();
    match inputs.next() {
        Some(FnArg::Receiver(r))
            if r.reference.is_some() && r.mutability.is_some() && r.colon_token.is_none() => {}
        Some(other) => return Err(bad(other)),
        None => return Err(bad(&sig.ident)),
    }
    // No parameters beyond the receiver.
    if let Some(extra) = inputs.next() {
        return Err(bad(extra));
    }
    // Return type must be a path ending in `ExternalSource`.
    let returns_external_source = match &sig.output {
        syn::ReturnType::Type(_, ty) => match ty.as_ref() {
            Type::Path(p) => p
                .path
                .segments
                .last()
                .is_some_and(|seg| seg.ident == "ExternalSource"),
            _ => false,
        },
        syn::ReturnType::Default => false,
    };
    if !returns_external_source {
        return Err(bad(&sig.output));
    }
    Ok(())
}

/// Rename the user's validated `external_source` to
/// `__cer_user_external_source` (mirrors `rewrite_user_init_method`). Unlike
/// init/shutdown there is no `Result` boundary to widen — the method returns
/// `ExternalSource` by value — so the body is left untouched (it runs OUTSIDE
/// tick and must not touch ports). The codegen wrapper's `external_source()`
/// override calls this shim.
fn rewrite_user_external_source_method(method: &mut ImplItemFn) {
    method.sig.ident = format_ident!("__cer_user_external_source");
    method.attrs.push(parse_quote!(#[doc(hidden)]));
}

/// Rewrite the user's `fn init(&mut self, ctx:
/// &mut NodeContext) -> Result<(), NodeError>` to `fn __cer_user_init`
/// with the same signature except the return type is widened to
/// `TransportResult<()>` (matches the wrapper's expected boundary).
/// The body's `Result<(), NodeError>` is converted at the boundary via
/// the same closure-then-map_err pattern `tick` uses.
fn rewrite_user_init_method(method: &mut ImplItemFn) {
    let __cer_root = crate_root::root();
    let user_body = method.block.clone();
    let new_block: Block = parse_quote! {
        {
            let __cer_user_result: ::std::result::Result<
                (),
                #__cer_root::error::NodeError,
            > = (|| #user_body)();
            __cer_user_result.map_err(|e| #__cer_root::error::TransportError::NodeError {
                node_id: ::std::string::String::from("user_init"),
                reason: ::std::string::ToString::to_string(&e),
            })
        }
    };
    method.block = new_block;
    method.sig.ident = format_ident!("__cer_user_init");
    method.sig.output = parse_quote! {
        -> #__cer_root::error::TransportResult<()>
    };
    // Hide from rustdoc — the user-facing name is `init`, the renamed
    // version is implementation detail.
    method.attrs.push(parse_quote!(#[doc(hidden)]));
}

/// Same as `rewrite_user_init_method` but for the
/// optional `fn shutdown(&mut self) -> Result<(), NodeError>` method.
fn rewrite_user_shutdown_method(method: &mut ImplItemFn) {
    let __cer_root = crate_root::root();
    let user_body = method.block.clone();
    let new_block: Block = parse_quote! {
        {
            let __cer_user_result: ::std::result::Result<
                (),
                #__cer_root::error::NodeError,
            > = (|| #user_body)();
            __cer_user_result.map_err(|e| #__cer_root::error::TransportError::NodeError {
                node_id: ::std::string::String::from("user_shutdown"),
                reason: ::std::string::ToString::to_string(&e),
            })
        }
    };
    method.block = new_block;
    method.sig.ident = format_ident!("__cer_user_shutdown");
    method.sig.output = parse_quote! {
        -> #__cer_root::error::TransportResult<()>
    };
    method.attrs.push(parse_quote!(#[doc(hidden)]));
}

/// Whether the first parameter of `method` is `&mut self` or `&self` — a
/// receiver, not a non-receiver `Self` type. Methods with no receiver, or
/// with `self` taken by value, are not eligible for the helper rewrite
/// because per-tick locals can't be smuggled into them safely.
fn first_arg_is_self_receiver(method: &ImplItemFn) -> bool {
    matches!(method.sig.inputs.first(), Some(FnArg::Receiver(_)))
}

/// `true` iff a method's return type SYNTACTICALLY guarantees it
/// cannot use `?` — no return arrow at all, or the literal empty tuple
/// `-> ()`. Deliberately does NOT try to judge aliases / qualified paths /
/// generics: anything else is assumed `?`-capable and left to rustc's
/// E0277 backstop, so the unit-return diagnostic has zero false positives
/// by construction.
fn returns_unit_syntactically(output: &syn::ReturnType) -> bool {
    match output {
        syn::ReturnType::Default => true,
        syn::ReturnType::Type(_, ty) => {
            matches!(ty.as_ref(), Type::Tuple(t) if t.elems.is_empty())
        }
    }
}

/// Rewrite a non-tick helper method:
///   1. Rewrite `self.<port>` references in the body to `__cer_<port>`
///      locals (which arrive as the trailing parameters added below).
///   2. Append `&mut __cer_<port>: &mut LazyOutput<'_, T>` (outputs — the
///      lazy slot) or `__cer_<port>: &InputView<'_, T>` (inputs) to the
///      method's parameter list, in the canonical order recorded in
///      `helper_port_args`.
///   3. Rewrite any `self.<other_helper>(args)` calls inside this method
///      so they pass through the per-tick locals (so a helper-helper chain
///      like `tick → fill_command → fill_subfield` works without losing
///      the proxy references).
fn rewrite_helper_method(
    method: &mut ImplItemFn,
    port_idents: &HashSet<String>,
    output_idents: &HashSet<String>,
    helper_port_args: &HashMap<String, Vec<(String, PortKind)>>,
    port_type_by_name: &HashMap<String, (Type, PortKind)>,
) {
    let __cer_root = crate_root::root();
    // Walk the body — same visitor as tick. The visitor handles both
    // `self.<port>` rewrites and `self.<other_helper>(args)` argument
    // injection.
    let mut rewriter = SelfPortRewriter {
        port_idents,
        helper_port_args,
        output_idents,
        assign_rewrites: 0,
        first_assign: None,
    };
    visit_block_mut(&mut rewriter, &mut method.block);

    // Decision: a helper whose signature syntactically
    // CANNOT use `?` (no return arrow, or literal `-> ()`) but whose body
    // had >= 1 port-field assignment rewritten to a fallible
    // `__cer_assign_<field>(…)?` call would otherwise die with rustc's
    // generic E0277 ("the `?` operator can only be used in a method that
    // returns `Result`… consider `Box<dyn Error>`"). Replace the whole
    // BODY with our targeted compile_error — once per offending method, no
    // E0277 cascade (the `?`s are gone with the body), zero false
    // positives by construction (a `()` method can never use `?`, and
    // every rewritten assignment carries one). Reads and user-written
    // `fill_from` calls never trip this (only step-2 assigns are counted).
    // Any OTHER return arrow (aliases, qualified paths, …) is assumed fine
    // — rustc's E0277 remains the backstop there.
    if rewriter.assign_rewrites > 0 && returns_unit_syntactically(&method.sig.output) {
        let (port, field) = rewriter
            .first_assign
            .clone()
            .expect("assign_rewrites > 0 implies a recorded first offender");
        let method_name = method.sig.ident.to_string();
        // `field` may be a dotted path (`header.stamp.sec`) for a nested
        // write, so it reads correctly in `self.<port>.<field>` but must NOT
        // be spliced into `__cer_assign_<field>` (that ident uses only the
        // leaf, and nested writes route through `__cer_with_nested_*`). The
        // second sentence names both shims generically instead.
        let msg = format!(
            "`{method_name}` writes port field(s) (e.g. `self.{port}.{field}`) — \
             port writes are fallible, so this method must return `Result` (e.g. \
             `Result<(), NodeError>`), and its callers must propagate with `?`. Each \
             rewritten port-field write expands to a fallible `?`-carrying \
             `__cer_assign_…` call (nested paths route through `__cer_with_nested_…`)."
        );
        let span = method.sig.ident.span();
        method.block = parse_quote_spanned! { span => {
            ::std::compile_error!(#msg);
        }};
    }

    // Append `__cer_<port>` parameters in the canonical order.
    let helper_name = method.sig.ident.to_string();
    let Some(extra) = helper_port_args.get(&helper_name) else {
        return;
    };
    for (port, _kind) in extra {
        let local = format_ident!("__cer_{}", port);
        let Some((ty, kind)) = port_type_by_name.get(port) else {
            continue;
        };
        let new_arg: FnArg = match kind {
            // Helpers receive the `&mut LazyOutput` slot (not a
            // pre-loaned `&mut OutputProxy`), so a helper that never writes a
            // port it was handed never loans it. The write shims inside the
            // helper body route through `__cer_<port>.__cer_loan()?`
            // identically to the tick frame (see `output_write_receiver`).
            PortKind::Output => parse_quote! {
                #local: &mut #__cer_root::graph::node::LazyOutput<'_, #ty>
            },
            PortKind::Input => parse_quote! {
                #local: &#__cer_root::transport::input_view::InputView<'_, #ty>
            },
        };
        method.sig.inputs.push(new_arg);
    }

    // Suppress "fn signature differs from prior declaration" style lints —
    // not applicable here, but keeps the helper from being flagged as
    // unused if user code only calls it via tick (which passes the
    // extra args). The compiler still enforces param-count match at the
    // call site, so the rewrite is sound even without this attribute.
    let allow_attr: syn::Attribute = parse_quote! {
        #[allow(clippy::needless_lifetimes, clippy::too_many_arguments, dead_code)]
    };
    method.attrs.push(allow_attr);
}

/// Build the per-tick context-split + output-slot setup.
///
/// Splits `ctx` into disjoint `&mut` views of the publishers and
/// subscribers maps so the rest of the wrapper can hold output slots
/// (borrowing publishers) and input views (borrowing subscribers)
/// concurrently. Then takes every required output publisher off the
/// publisher map via `get_disjoint_mut` (so multiple output slots don't
/// conflict either) and binds each as a `LazyOutput` (from
/// `cerulion_core::graph::node`) — the proxy is loaned lazily on the port's
/// FIRST write, so an untouched output never loans, never discards, never
/// floods.
///
/// Failure to find a publisher is a runtime `NodeError` rather than a
/// silent skip — declarative-mode users always declare every output they
/// touch, so a missing publisher is a wiring bug. (This check stays at the
/// preamble; only the LOAN moves to first write.)
fn build_ctx_split_and_output_loans(impl_attr: &ImplAttr) -> TokenStream2 {
    let __cer_root = crate_root::root();
    let split = quote! {
        let (__cer_pubs_map, __cer_subs_map) =
            ctx.split_publishers_subscribers_mut();
    };

    if impl_attr.outputs.is_empty() {
        // Suppress unused-binding warnings on the publisher map when
        // there are no outputs to take from it. Subscribers are always
        // taken below if any inputs are declared.
        return quote! {
            #split
            let _ = __cer_pubs_map;
        };
    }

    let n_out = impl_attr.outputs.len();
    let n_out_lit = syn::LitInt::new(&n_out.to_string(), Span::call_site());
    let out_names_lit: Vec<TokenStream2> = impl_attr
        .outputs
        .iter()
        .map(|p| {
            let n = p.name.to_string();
            quote! { #n }
        })
        .collect();

    // Lazy-loan: for each output, take the publisher off the disjoint
    // array and bind a per-port `LazyOutput` SLOT (NOT an eager loan) plus a
    // `&mut LazyOutput` reborrow to `__cer_<port>`. The proxy is loaned on the
    // FIRST write (via `LazyOutput::__cer_loan`, which the write shims
    // route through) — an output the tick never writes never loans, never
    // discards, never floods (see `LazyOutput` in `graph::node`). The reborrow
    // keeps the call-site rewriter uniform: passing `__cer_<port>` to a helper
    // auto-reborrows the `&mut LazyOutput` into the helper signature, exactly
    // as the earlier `&mut OutputProxy` reborrow did. The missing-publisher
    // check stays at the preamble (unchanged) — only the LOAN moves to first
    // write.
    let take_loans: Vec<TokenStream2> = impl_attr
        .outputs
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let port_name = p.name.to_string();
            let local = format_ident!("__cer_{}", p.name);
            let lazy_local = format_ident!("__cer_{}_lazy", p.name);
            let ty = &p.ty;
            let idx = syn::LitInt::new(&i.to_string(), Span::call_site());
            quote! {
                let mut #lazy_local: #__cer_root::graph::node::LazyOutput<'_, #ty> =
                    match __cer_out_pubs[#idx].take() {
                        Some(pub_port) =>
                            #__cer_root::graph::node::LazyOutput::new(pub_port),
                        None => {
                            return Err(#__cer_root::error::TransportError::NodeError {
                                node_id: #port_name.into(),
                                reason: concat!(
                                    "zero-copy tick: missing publisher for output `",
                                    #port_name,
                                    "`"
                                ).into(),
                            });
                        }
                    };
                // The `&mut LazyOutput` reborrow the write sites +
                // helper-arg passing reference as `__cer_<port>`. An output the
                // tick never writes references it nowhere — a legitimate
                // "declare an output, never write it" non-event; the `__cer_`
                // prefix (leading `_`) exempts it from `unused_variables`,
                // exactly as the earlier `&mut OutputProxy` reborrow relied
                // on. The owning `#lazy_local` is always used (the tail
                // arms/traces it).
                let #local: &mut #__cer_root::graph::node::LazyOutput<'_, #ty> =
                    &mut #lazy_local;
            }
        })
        .collect();

    quote! {
        #split
        let mut __cer_out_pubs: [::std::option::Option<
            &mut #__cer_root::graph::node::AnyPublisher,
        >; #n_out_lit] = __cer_pubs_map.get_disjoint_mut([#(#out_names_lit),*]);
        #(#take_loans)*
    }
}

/// Build the nested `try_view` chain for `inputs[start..]`.
///
/// `inner` is the leaf code that runs with every input view in scope. For
/// a single input we return `try_view::<T,_>(|view| inner)`; for two we
/// return `try_view::<T0,_>(|v0| try_view::<T1,_>(|v1| inner))`; etc.
///
/// All input subscribers must already be in scope as `__cer_in_sub_<i>`
/// locals (an `Option<&mut AnySubscriber>`); the caller is responsible
/// for taking them disjointly off `ctx` before invoking this. The leaf
/// closure returns `Result<bool, NodeError>`: `true` means the
/// user body genuinely ran (and returned `Ok(())`); `false` means the
/// WHOLE tick collapsed to a no-op because some input had nothing to
/// view. On the way back out we wrap each `try_view` so a `None` return
/// collapses to `Ok(Ok(false))` — NOT `Ok(Ok(()))` — so
/// `rewrite_tick_method`'s arm-on-Ok guard can tell "ran and succeeded"
/// apart from "collapsed". If both cases were the structurally
/// identical `Ok(Ok(()))`, a collapsed tick's already-loaned
/// fixed-schema outputs would be armed and publish a fabricated zero-init
/// frame.
fn build_nested_try_view(inputs: &[TypedPort], start: usize, leaf: TokenStream2) -> TokenStream2 {
    let __cer_root = crate_root::root();
    if start >= inputs.len() {
        return leaf;
    }
    let p = &inputs[start];
    let local = format_ident!("__cer_{}", p.name);
    let view_local = format_ident!("__cer_{}_view", p.name);
    let sub_local = format_ident!("__cer_in_sub_{}", start);
    let ty = &p.ty;
    let inner = build_nested_try_view(inputs, start + 1, leaf);
    // Bind the per-input local as `&InputView<'_, T>` (a shared
    // reference) so it has the same type as the helper-injected
    // parameter. The `view` parameter the closure receives is owned;
    // we keep the owned value alive in `<port>_view` and bind a
    // reference to it for the user body to consume.
    quote! {
        {
            let __cer_step: ::std::result::Result<
                ::std::result::Result<bool, #__cer_root::error::NodeError>,
                #__cer_root::error::TransportError,
            > = match #sub_local.take() {
                Some(__cer_sub_ref) => {
                    let __cer_view_outcome = __cer_sub_ref
                        .try_view::<#ty, _>(|view| {
                            let #view_local = view;
                            let #local: &#__cer_root::transport::input_view::InputView<
                                '_, #ty,
                            > = &#view_local;
                            let __cer_layer: ::std::result::Result<
                                bool,
                                #__cer_root::error::NodeError,
                            > = #inner;
                            __cer_layer
                        });
                    match __cer_view_outcome {
                        Ok(Some(layer_result)) => Ok(layer_result),
                        // No sample available for this input — the user
                        // body did not run because a view cannot be
                        // fabricated. `Ok(Ok(false))` marks this as
                        // COLLAPSED (the inner closure never ran, so
                        // nothing deeper could have set `true`) — the
                        // tick is still a successful no-op (matches the
                        // single-input semantics), but the arm
                        // gate in `rewrite_tick_method` must NOT publish
                        // on it.
                        Ok(None) => Ok(Ok(false)),
                        Err(e) => Err(e),
                    }
                }
                // Subscriber missing in ctx (wiring bug). Treat
                // as a COLLAPSED no-op (not "ran") for parity with the
                // single-input path — it never ran the inner closure
                // either.
                None => Ok(Ok(false)),
            };
            __cer_step?
        }
    }
}

/// Rewrite a single `fn tick` method:
///   1. Walk its body, replacing `self.<port>` with `__cer_<port>` locals
///      and `self.<helper>(args)` with the trailing-arg-augmented call.
///   2. Wrap the rewritten body in a generated outer scope that:
///      - For each output: binds a `LazyOutput` slot; the proxy is
///        loaned lazily on the port's FIRST write (an untouched output never
///        loans, so it cannot flood discard errors).
///      - For each input: takes the subscriber off the previously-split
///        `__cer_subs_map` via `IndexMap::get_disjoint_mut`, then opens
///        a nested `try_view` callback per input.
///      - The tail arms each LOANED output's publish on a fully-Ok outcome
///        and `trace!`s each skipped port; the `LazyOutput` slots drop at the
///        end of the scope (a loaned+armed proxy publishes, a loaned+unarmed
///        one discards, an untouched one is a no-op).
///   3. Rename the method to `__cer_zero_copy_tick` and inject a `ctx`
///      parameter. The struct macro's generated `<Name>Entry::tick` will
///      call this method.
fn rewrite_tick_method(
    method: &mut ImplItemFn,
    impl_attr: &ImplAttr,
    port_idents: &HashSet<String>,
    output_idents: &HashSet<String>,
    helper_port_args: &HashMap<String, Vec<(String, PortKind)>>,
    event_handlers: &[EventHandler],
) {
    let __cer_root = crate_root::root();
    let mut rewriter = SelfPortRewriter {
        port_idents,
        helper_port_args,
        output_idents,
        assign_rewrites: 0,
        first_assign: None,
    };
    visit_block_mut(&mut rewriter, &mut method.block);

    let user_body: &Block = &method.block;

    let split_and_loans = build_ctx_split_and_output_loans(impl_attr);

    // Per-port `#[on_event]` dispatch, emitted AFTER the
    // user tick body and on the Ok path only, in SOURCE order (the
    // determinism guarantee — dispatch order = declaration order). The
    // proxy/subscriber borrows of `ctx` live inside `#inner` (the loan/view
    // scope); we evaluate that inner block to a plain `Result<(), NodeError>`
    // first so every `&mut ctx` borrow is released by the time we re-borrow
    // `ctx` for the `take_*_event` accessors. Each handler drains its port's
    // single pending event (edge-triggered by the foundation), type-routed to
    // the accessor for its event kind, and, if present, calls the user method
    // with it.
    //
    // Ok-path-only: a failed tick does NOT fire event handlers. The tick
    // erred, so user state may be half-written; firing a recovery callback on
    // top of that is worse than deferring the event to the next successful
    // tick (the foundation keeps the event queued until drained).
    let dispatch: Vec<TokenStream2> = event_handlers
        .iter()
        .map(|h| {
            let port = &h.port;
            let handler = &h.method;
            let accessor = h.kind.accessor_ident();
            quote! {
                if let ::std::option::Option::Some(__cer_event) =
                    ctx.#accessor(#port)
                {
                    self.#handler(__cer_event);
                }
            }
        })
        .collect();

    // Publish-on-success inversion + lazy-loan: a
    // NON-COMPLETED tick publishes NOTHING, structurally. Under lazy-loan a
    // proxy exists ONLY for a port the tick actually WROTE (the write shim's
    // `__cer_<port>.__cer_loan()?` loaned it and DEFERRED it — see
    // `LazyOutput::__cer_loan`), so discard is the DEFAULT and the Err arms
    // here do nothing. The tail's only job is to ARM publishing on the
    // fully-Ok outcome, per LOANED port, while the `LazyOutput` slots are
    // still alive (they drop — publishing the armed / discarding the unarmed
    // — at the END of the `#inner` block). Illegal state unrepresentable:
    // publishing requires affirmative tick completion AND a real write.
    //
    // Legs covered by construction: (a) a SIBLING output's loan failure at a
    // LATER write (earlier written proxies are already deferred → discard);
    // (b) TRANSPORT Err from the try_view input chain (the WHOLE chain runs
    // inside the capture closure, so `__cer_step?` exits the CLOSURE, and the
    // un-armed proxies discard); (c) user-body `Err` / early `return
    // Err(...)` / `?`-carrying rewritten port writes (same capture); (d) any
    // future early exit — it cannot arm; (e) a COLLAPSED tick (some
    // non-trigger input had nothing to view, so the user body never ran, so
    // NO port ever loaned) — the two-layer outcome is `Ok(Ok(false))`,
    // structurally distinct from a genuine `Ok(Ok(true))` success, so it
    // falls through the guard exactly like an Err leg. `__cer_arm_if_loaned`
    // is a no-op on an untouched port. The `__cer_<port>` reborrows' last use
    // is inside the capture closure, so arming/tracing the owning
    // `__cer_<port>_lazy` locals here borrows cleanly (NLL).
    let arm_calls: TokenStream2 = {
        let lazy_locals: Vec<Ident> = impl_attr
            .outputs
            .iter()
            .map(|p| format_ident!("__cer_{}_lazy", p.name))
            .collect();
        quote! { #( #lazy_locals.__cer_arm_if_loaned(); )* }
    };
    // After the arm decision, emit a per-skipped-port `trace!` for
    // every output the tick did NOT write (a zero-traffic non-event — off by
    // default, `RUST_LOG=…=trace` surfaces it). Runs on EVERY path (Ok, Err,
    // collapsed) — a skipped port is skipped either way; `__cer_trace_if_unloaned`
    // self-gates on `proxy.is_none()`. Empty for output-less nodes.
    let trace_calls: TokenStream2 = {
        let lazy_locals: Vec<Ident> = impl_attr
            .outputs
            .iter()
            .map(|p| format_ident!("__cer_{}_lazy", p.name))
            .collect();
        quote! { #( #lazy_locals.__cer_trace_if_unloaned(); )* }
    };
    // 0-input arm: the only fallible step past the preamble is the user
    // body, so the guard keys on `__cer_user_result` directly.
    let arm_on_ok: TokenStream2 = if impl_attr.outputs.is_empty() {
        quote! {}
    } else {
        quote! {
            if __cer_user_result.is_ok() {
                #arm_calls
            }
        }
    };
    // Inputs arm: the capture is two layers (transport OUTER, user INNER)
    // — arm ONLY on `Ok(Ok(true))` (`true` = the user body
    // genuinely ran and returned `Ok(())`; `Ok(Ok(false))` is a COLLAPSED
    // no-op tick — some input was empty, the user body never ran, and
    // arming publish on it would fabricate a fixed-schema output's
    // zero-init loan as a real sample), then yield unchanged (error
    // identity preserved: a transport error re-propagates via `?` AFTER
    // the guard; the user error flows to the boundary conversion exactly
    // as before).
    let arm_on_ok_two_layer: TokenStream2 = if impl_attr.outputs.is_empty() {
        quote! {}
    } else {
        quote! {
            if ::std::matches!(
                __cer_tick_outcome,
                ::std::result::Result::Ok(::std::result::Result::Ok(true))
            ) {
                #arm_calls
            }
        }
    };

    // The user's tick method returns `Result<(), NodeError>` (declarative
    // contract) — but the entry's NodeEntry::tick returns
    // `TransportResult<()>` (`Result<(), TransportError>`). We map at the
    // boundary by running the user body via a closure into
    // `Result<(), NodeError>` and then converting any error to
    // `TransportError::NodeError { ... }` outside the closure.
    let inner: TokenStream2 = if impl_attr.inputs.is_empty() {
        // No inputs — flat scope: outputs only, then user body, then drop.
        // `__cer_subs_map` is bound but unused; suppress with `_`.
        quote! {
            {
                #split_and_loans
                let _ = __cer_subs_map;
                let __cer_user_result: ::std::result::Result<(), #__cer_root::error::NodeError> =
                    (|| #user_body)();
                #arm_on_ok
                #trace_calls
                __cer_user_result
            }
        }
    } else {
        // 1+ inputs (multi-input zero-copy tick): take all input subscribers off the disjoint
        // `__cer_subs_map` upfront, then nest `try_view` closures so
        // every input's view is in scope simultaneously.
        let input_names: Vec<String> = impl_attr
            .inputs
            .iter()
            .map(|p| p.name.to_string())
            .collect();
        let take_subs = build_input_subscriber_takes(&input_names);

        // Build the inner-most leaf: `(|| user_body)()` returns
        // `Result<(), NodeError>`; `.map(|()| true)` lifts it to
        // `Result<bool, NodeError>` so it matches the nested `try_view`
        // chain's uniform `Result<bool, NodeError>` layer, `true` marking
        // "the user body genuinely ran" (as opposed to the `Ok(Ok(false))`
        // a deeper collapse produces — see `build_nested_try_view`). The
        // closure argument is the irrefutable UNIT pattern `()`, not a
        // wildcard `_` — `_` would accept (and silently discard) ANY
        // Ok-payload type, quietly loosening the compile-time
        // contract that a tick body's success arm must produce exactly
        // `()` (a mistyped non-unit tail expression, e.g. a forgotten `;`,
        // would then compile with the value silently dropped instead of
        // failing with a type-mismatch error).
        let leaf: TokenStream2 = quote! {
            (|| #user_body)().map(|()| true)
        };
        let nested = build_nested_try_view(&impl_attr.inputs, 0, leaf);

        // Transport-Err leg: the try_view chain's `__cer_step?`
        // would otherwise early-return the TransportError OUT OF THE FN.
        // Wrap the WHOLE chain (user body included) in an immediately-
        // invoked capture closure returning
        // `TransportResult<Result<bool, NodeError>>` (`bool` =
        // ran-vs-collapsed, see `build_nested_try_view`): the proxies are
        // declared OUTSIDE it (`#split_and_loans`), every `?` inside exits
        // the closure, and the arm-on-Ok guard sees BOTH failure layers
        // before the transport error re-propagates (an escape can never
        // arm — the deferred default discards).
        quote! {
            {
                #split_and_loans
                #take_subs
                let __cer_tick_outcome: #__cer_root::error::TransportResult<
                    ::std::result::Result<bool, #__cer_root::error::NodeError>,
                > = (|| ::std::result::Result::Ok(#nested))();
                #arm_on_ok_two_layer
                // Trace skipped ports BEFORE the `?` below re-raises a
                // transport error (an escape must still surface which outputs
                // were untouched). `__cer_trace_if_unloaned` self-gates.
                #trace_calls
                // Discard the ran/collapsed discriminant AFTER
                // the arm guard has consumed it — everything downstream
                // (`dispatch_block`'s `is_ok()` gate, the final
                // `.map_err(...)` boundary conversion) only cares about
                // Ok-vs-Err, never ran-vs-collapsed.
                let __cer_user_result: ::std::result::Result<
                    (),
                    #__cer_root::error::NodeError,
                > = __cer_tick_outcome?.map(|_ran| ());
                __cer_user_result
            }
        }
    };

    // Error-context label: the first input port name when present (matches
    // the historical behavior), else the literal `"zero_copy_tick"`.
    let err_label: String = impl_attr
        .inputs
        .first()
        .map(|p| p.name.to_string())
        .unwrap_or_else(|| "zero_copy_tick".to_string());

    // Only emit the Ok-path dispatch guard when there is at least one
    // handler — an empty `if __cer_user_result.is_ok() {}` would trip
    // clippy and adds a needless branch.
    let dispatch_block: TokenStream2 = if dispatch.is_empty() {
        quote! {}
    } else {
        quote! {
            // Dispatch `#[on_event]` handlers on the Ok
            // path, after all proxy/view borrows of `ctx` are released by
            // `#inner`.
            if __cer_user_result.is_ok() {
                #(#dispatch)*
            }
        }
    };

    let new_block: Block = parse_quote! {
        {
            let __cer_user_result: ::std::result::Result<
                (),
                #__cer_root::error::NodeError,
            > = #inner;
            #dispatch_block
            __cer_user_result.map_err(|e| #__cer_root::error::TransportError::NodeError {
                node_id: ::std::string::String::from(#err_label),
                reason: ::std::string::ToString::to_string(&e),
            })
        }
    };

    method.block = new_block;

    // Rename the method to `__cer_zero_copy_tick` and inject a `ctx`
    // parameter so the struct macro's generated wrapper can call it.
    // Override the return type to `TransportResult<()>` — the user wrote
    // `Result<(), NodeError>`, but we converted at the boundary inside
    // the generated body.
    method.sig.ident = format_ident!("__cer_zero_copy_tick");
    let ctx_arg: FnArg = parse_quote! {
        ctx: &mut #__cer_root::graph::node::NodeContext
    };
    method.sig.inputs.push(ctx_arg);
    method.sig.output = parse_quote! {
        -> #__cer_root::error::TransportResult<()>
    };
}

/// Generate the disjoint-mut call that pulls every input's
/// `&mut AnySubscriber` off the previously-split `__cer_subs_map` in one
/// shot, binding each to a local `__cer_in_sub_<i>: Option<&mut
/// AnySubscriber>` for the nested `try_view` chain to consume via
/// `take()`.
///
/// The names list cannot contain duplicates because each input is
/// declared at most once on `#[cerulion_node_impl(inputs(...))]`, so
/// `IndexMap::get_disjoint_mut`'s panic-on-duplicate is unreachable in
/// generated code.
fn build_input_subscriber_takes(input_names: &[String]) -> TokenStream2 {
    let __cer_root = crate_root::root();
    let n = input_names.len();
    let names_lit: Vec<TokenStream2> = input_names.iter().map(|n| quote! { #n }).collect();
    let n_lit = syn::LitInt::new(&n.to_string(), Span::call_site());
    let takes: Vec<TokenStream2> = (0..n)
        .map(|i| {
            let local = format_ident!("__cer_in_sub_{}", i);
            let idx = syn::LitInt::new(&i.to_string(), Span::call_site());
            quote! {
                let mut #local: ::std::option::Option<
                    &mut #__cer_root::graph::node::AnySubscriber,
                > = __cer_subs_array[#idx].take();
            }
        })
        .collect();
    quote! {
        let mut __cer_subs_array: [::std::option::Option<
            &mut #__cer_root::graph::node::AnySubscriber,
        >; #n_lit] = __cer_subs_map.get_disjoint_mut([#(#names_lit),*]);
        #(#takes)*
    }
}

#[cfg(test)]
mod tests {
    //! Unit tests for
    //! [`validate_external_source_sig`]. The validator is file-private, so the
    //! tests live inline and parse candidate signatures with `syn::parse_quote!`.
    //! End-to-end rejection rendering is pinned by the trybuild fixtures in
    //! `crates/cerulion_core/tests/ui/external_source_wrong_signature.rs`; these pin the
    //! validator's decision surface directly (each malformed shape → `Err`, the
    //! one exact valid shape → `Ok`).
    use super::*;

    /// Oracle vectors for the unit-return predicate gating the
    /// helper port-write diagnostic. Zero-false-positive contract: ONLY
    /// no-arrow and literal `-> ()` count as unit; every other arrow
    /// (Result, aliases, qualified paths, non-empty tuples) is left to
    /// rustc's E0277 backstop.
    #[test]
    fn returns_unit_syntactically_oracle_vectors() {
        let unit_cases: Vec<ImplItemFn> = vec![
            parse_quote! { fn a(&mut self) {} },
            parse_quote! { fn b(&mut self) -> () {} },
        ];
        for m in &unit_cases {
            assert!(
                returns_unit_syntactically(&m.sig.output),
                "no-arrow / `-> ()` must classify as unit: {:?}",
                m.sig.ident
            );
        }
        let non_unit_cases: Vec<ImplItemFn> = vec![
            parse_quote! { fn c(&mut self) -> Result<(), NodeError> { Ok(()) } },
            parse_quote! { fn d(&mut self) -> f64 { 0.0 } },
            // An alias COULD be unit — but the predicate must not guess
            // (rustc's E0277 is the backstop for aliased returns).
            parse_quote! { fn e(&mut self) -> MyAlias { todo!() } },
            // A non-empty tuple is not unit.
            parse_quote! { fn f(&mut self) -> (u8,) { (0,) } },
        ];
        for m in &non_unit_cases {
            assert!(
                !returns_unit_syntactically(&m.sig.output),
                "arrowed non-`()` types must NOT classify as unit: {:?}",
                m.sig.ident
            );
        }
    }

    // ------------------------------------------------------------------
    // Token-level oracle vectors for the recursive
    // nested-write rewrite. Each pin runs the tick-body rewriter over a
    // single expression (or block) and compares its emitted token string
    // against a HAND-WRITTEN `parse_quote!` oracle — NOT a self-compare
    // (both sides format through the same proc-macro2 `to_string`, so the
    // comparison is spacing-robust while the RHS is independently authored).
    // ------------------------------------------------------------------

    /// Run the tick-body rewriter over ONE expression with `outputs` as the
    /// declared output ports (also the full port set — no inputs needed for
    /// these shapes). Returns the rewritten expr + the assign bookkeeping.
    fn rewrite_one_expr(
        mut expr: Expr,
        outputs: &[&str],
    ) -> (Expr, usize, Option<(String, String)>) {
        let output_idents: HashSet<String> = outputs.iter().map(|s| s.to_string()).collect();
        let port_idents = output_idents.clone();
        let helper_port_args: HashMap<String, Vec<(String, PortKind)>> = HashMap::new();
        let mut rewriter = SelfPortRewriter {
            port_idents: &port_idents,
            helper_port_args: &helper_port_args,
            output_idents: &output_idents,
            assign_rewrites: 0,
            first_assign: None,
        };
        rewriter.visit_expr_mut(&mut expr);
        (
            expr,
            rewriter.assign_rewrites,
            rewriter.first_assign.clone(),
        )
    }

    /// Same as [`rewrite_one_expr`] but over a whole block (for counting
    /// multiple rewrites in declaration order).
    fn rewrite_one_block(mut block: Block, outputs: &[&str]) -> (usize, Option<(String, String)>) {
        let output_idents: HashSet<String> = outputs.iter().map(|s| s.to_string()).collect();
        let port_idents = output_idents.clone();
        let helper_port_args: HashMap<String, Vec<(String, PortKind)>> = HashMap::new();
        let mut rewriter = SelfPortRewriter {
            port_idents: &port_idents,
            helper_port_args: &helper_port_args,
            output_idents: &output_idents,
            assign_rewrites: 0,
            first_assign: None,
        };
        visit_block_mut(&mut rewriter, &mut block);
        (rewriter.assign_rewrites, rewriter.first_assign.clone())
    }

    /// Token string of an expression via proc-macro2's formatter.
    fn tok(e: &Expr) -> String {
        quote!(#e).to_string()
    }

    // ------------------------------------------------------------------
    // Publish-on-success inversion: token-level oracles for the
    // tick PREAMBLE + TAIL — every output proxy is DEFERRED immediately
    // after its loan, and ONLY a fully-Ok outcome ARMS publishing before
    // the proxies drop. The rewrite oracles above are untouched by this
    // (the preamble/tail wrap the rewritten body; they do not alter any
    // rewrite emission).
    // ------------------------------------------------------------------

    /// Run `rewrite_tick_method` over a minimal tick with the given output
    /// ports (no inputs, no handlers) and return the whitespace-stripped
    /// token string of the rewritten method.
    fn rewritten_tick_flat(outputs: &[&str]) -> String {
        let mut method: ImplItemFn = parse_quote! {
            fn tick(&mut self) -> Result<(), NodeError> {
                Ok(())
            }
        };
        let impl_attr = ImplAttr {
            inputs: vec![],
            outputs: outputs
                .iter()
                .map(|n| TypedPort {
                    name: format_ident!("{}", n),
                    ty: parse_quote!(Vector3),
                })
                .collect(),
        };
        let port_idents: HashSet<String> = outputs.iter().map(|s| s.to_string()).collect();
        let output_idents = port_idents.clone();
        let helper_port_args: HashMap<String, Vec<(String, PortKind)>> = HashMap::new();
        rewrite_tick_method(
            &mut method,
            &impl_attr,
            &port_idents,
            &output_idents,
            &helper_port_args,
            &[],
        );
        quote!(#method)
            .to_string()
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect()
    }

    #[test]
    fn tick_preamble_binds_lazy_output_slots_no_eager_loan() {
        // The preamble binds a `LazyOutput` slot per output but does
        // NOT eagerly loan — no `loan_proxy` token and no `__cer_defer_publish`
        // token appears in the generated tick (both now live INSIDE
        // `LazyOutput::__cer_loan`, invoked only on the first write). Each port
        // gets its `LazyOutput::new(pub_port)` binding.
        let flat = rewritten_tick_flat(&["cmd", "aux"]);
        assert!(
            !flat.contains("loan_proxy"),
            "the preamble must NOT eagerly loan — the loan moved into \
             LazyOutput::__cer_loan (first-write); got: {flat}"
        );
        assert!(
            !flat.contains("__cer_defer_publish"),
            "the preamble must NOT defer eagerly — the defer moved into \
             LazyOutput::__cer_loan; got: {flat}"
        );
        assert!(
            flat.contains("LazyOutput::new(pub_port)"),
            "each output must bind a LazyOutput slot over its publisher; got: {flat}"
        );
        assert!(
            flat.contains("__cer_cmd_lazy") && flat.contains("__cer_aux_lazy"),
            "each output must bind its `__cer_<port>_lazy` slot local; got: {flat}"
        );
    }

    #[test]
    fn tick_tail_arms_loaned_publish_only_on_ok_then_traces_skips() {
        // Two outputs → BOTH slots arm-IF-LOANED inside ONE is_ok()
        // guard, in declaration order; then BOTH `__cer_trace_if_unloaned`
        // (per-skipped-port visibility), before the trailing
        // `__cer_user_result`. The Err arm does NOTHING — the deferred default
        // discards. An untouched port's arm-if-loaned is a runtime no-op.
        let flat = rewritten_tick_flat(&["cmd", "aux"]);
        let tail = "if__cer_user_result.is_ok(){__cer_cmd_lazy.__cer_arm_if_loaned();\
                    __cer_aux_lazy.__cer_arm_if_loaned();}\
                    __cer_cmd_lazy.__cer_trace_if_unloaned();\
                    __cer_aux_lazy.__cer_trace_if_unloaned();__cer_user_result";
        assert!(
            flat.contains(tail),
            "tick tail must arm every LOANED output ONLY on Ok, then trace every \
             skipped port, then yield the result; got: {flat}"
        );
        assert!(
            !flat.contains("is_err"),
            "no Err-keyed logic may remain — discard is the default; got: {flat}"
        );
        // The capture shape itself (user body via immediately-invoked
        // closure) is what routes early `return Err(...)` / `?` around
        // the arm guard (they can never arm).
        assert!(
            flat.contains("=(||{Ok(())})();"),
            "user body must be captured via an immediately-invoked closure; got: {flat}"
        );
    }

    #[test]
    fn tick_tail_emits_no_arm_or_trace_for_output_less_nodes() {
        let flat = rewritten_tick_flat(&[]);
        assert!(
            !flat.contains("__cer_arm_if_loaned")
                && !flat.contains("__cer_trace_if_unloaned")
                && !flat.contains("__cer_loan"),
            "an output-less tick has nothing to loan, arm, or trace; got: {flat}"
        );
    }

    /// Like [`rewritten_tick_flat`] but with declared INPUTS too (the
    /// try_view-chain arm of `rewrite_tick_method`).
    fn rewritten_tick_flat_with_inputs(inputs: &[&str], outputs: &[&str]) -> String {
        let mut method: ImplItemFn = parse_quote! {
            fn tick(&mut self) -> Result<(), NodeError> {
                Ok(())
            }
        };
        let to_ports = |names: &[&str]| -> Vec<TypedPort> {
            names
                .iter()
                .map(|n| TypedPort {
                    name: format_ident!("{}", n),
                    ty: parse_quote!(Vector3),
                })
                .collect()
        };
        let impl_attr = ImplAttr {
            inputs: to_ports(inputs),
            outputs: to_ports(outputs),
        };
        let port_idents: HashSet<String> = inputs
            .iter()
            .chain(outputs.iter())
            .map(|s| s.to_string())
            .collect();
        let output_idents: HashSet<String> = outputs.iter().map(|s| s.to_string()).collect();
        let helper_port_args: HashMap<String, Vec<(String, PortKind)>> = HashMap::new();
        rewrite_tick_method(
            &mut method,
            &impl_attr,
            &port_idents,
            &output_idents,
            &helper_port_args,
            &[],
        );
        quote!(#method)
            .to_string()
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect()
    }

    #[test]
    fn tick_input_chain_is_inside_the_transport_err_capture() {
        // Transport-Err leg: the try_view chain must sit INSIDE the
        // immediately-invoked capture closure, so its `__cer_step?` exits
        // the CLOSURE (routing through the discard guard) instead of
        // escaping the fn with the proxies live. A mutation reverting to
        // the escaping `?` (binding `#nested` directly, no closure wrapper)
        // deletes the capture-open token and FAILS this oracle.
        let flat = rewritten_tick_flat_with_inputs(&["scan"], &["cmd"]);

        let capture_open = "=(||::std::result::Result::Ok(";
        let open_at = flat
            .find(capture_open)
            .unwrap_or_else(|| panic!("capture closure must wrap the input chain; got: {flat}"));
        let try_view_at = flat
            .find("try_view")
            .unwrap_or_else(|| panic!("inputs arm must emit a try_view chain; got: {flat}"));
        assert!(
            open_at < try_view_at,
            "the try_view chain must be INSIDE the capture closure \
             (capture at {open_at}, try_view at {try_view_at}); got: {flat}"
        );

        // The arm guard keys on the TWO-LAYER outcome (transport OUTER,
        // user INNER) — ONLY Ok(Ok(true)) arms (`true` = the user
        // body genuinely ran; `Ok(Ok(false))` is a collapsed no-op tick,
        // e.g. an empty non-trigger input, and must NOT arm); every escape
        // leaves the loaned-and-deferred proxies to discard. The arm
        // is `__cer_arm_if_loaned` on the `LazyOutput` slot (a no-op for an
        // untouched port), followed by the per-skipped-port trace.
        let guard = "if::std::matches!(__cer_tick_outcome,::std::result::Result::Ok(::std::result::Result::Ok(true))){__cer_cmd_lazy.__cer_arm_if_loaned();}";
        let guard_at = flat
            .find(guard)
            .unwrap_or_else(|| panic!("two-layer arm-on-Ok guard missing; got: {flat}"));
        // …and only THEN does the transport error re-propagate (`?` AFTER
        // the guard — error identity preserved, the arm not skipped on Ok).
        let requestion_at = flat.find("__cer_tick_outcome?").unwrap_or_else(|| {
            panic!("transport error must re-propagate via `__cer_tick_outcome?`; got: {flat}")
        });
        assert!(
            guard_at < requestion_at,
            "the arm guard must run BEFORE the transport error re-propagates; got: {flat}"
        );
    }

    #[test]
    fn tick_leaf_and_collapse_arms_use_bool_discriminant() {
        // The leaf must lift the user body's `Result<(), NodeError>`
        // to `Result<bool, NodeError>` via `.map(|()| true)`, and BOTH
        // collapse arms in `build_nested_try_view` (the `Ok(None)` no-sample
        // arm and the missing-subscriber `None` arm) must emit
        // `Ok(Ok(false))` — never `Ok(Ok(()))`, which would be
        // structurally identical to a genuine success and let a collapsed
        // tick's fixed-schema output arm publish (fabricating a zero-init
        // frame — the bug). The closure argument is the irrefutable
        // UNIT pattern `()`, not a wildcard `_` — a wildcard would silently
        // accept (and discard) any non-`()` Ok-payload, loosening the
        // pre-existing compile-time contract that a tick body must produce
        // exactly `()` on success (a second bug of the same class).
        let flat = rewritten_tick_flat_with_inputs(&["scan"], &["cmd"]);
        assert!(
            flat.contains("(||{Ok(())})().map(|()|true)"),
            "leaf must map the user body's Result<(),NodeError> to \
             Result<bool,NodeError> via an irrefutable UNIT pattern (not a \
             wildcard, which would silently discard a mistyped non-unit \
             Ok-payload); got: {flat}"
        );
        assert!(
            !flat.contains("Ok(Ok(()))"),
            "no collapse arm may emit a nested Ok(Ok(())) — it is \
             structurally identical to a genuine success; got: {flat}"
        );
        let collapse_count = flat.matches("Ok(Ok(false))").count();
        assert_eq!(
            collapse_count, 2,
            "exactly two collapse arms (Ok(None) and missing-subscriber \
             None) must emit Ok(Ok(false)) for a single-input chain; got \
             {collapse_count} in: {flat}"
        );
    }

    #[test]
    fn tick_input_chain_capture_emits_no_arm_or_trace_for_output_less_nodes() {
        // Output-less data-trigger node: the capture wrapper still exists
        // (harmless) but there is nothing to loan, arm, or trace.
        let flat = rewritten_tick_flat_with_inputs(&["scan"], &[]);
        assert!(
            !flat.contains("__cer_arm_if_loaned")
                && !flat.contains("__cer_trace_if_unloaned")
                && !flat.contains("__cer_loan"),
            "an output-less tick has nothing to loan, arm, or trace; got: {flat}"
        );
    }

    #[test]
    fn nested_assign_three_segment_emits_chain_with_hoist_and_question() {
        // `self.imu.orientation.x = 1.5` (fixed nested, 1 intermediate).
        let (out, count, first) =
            rewrite_one_expr(parse_quote! { self.imu.orientation.x = 1.5 }, &["imu"]);
        // The chain roots at the lazy get-or-loan receiver
        // `__cer_imu.__cer_loan()?`.
        let expected: Expr = parse_quote! {
            {
                let __cer_rhs = 1.5;
                __cer_imu.__cer_loan()?.__cer_with_nested_orientation(|__cer_v| __cer_v.__cer_assign_x(&(__cer_rhs)))?
            }
        };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "3-segment nested chain + hoist + `?`"
        );
        assert_eq!(count, 1, "the nested assign is counted");
        assert_eq!(
            first,
            Some(("imu".to_string(), "orientation.x".to_string())),
            "first_assign carries the dotted path"
        );
    }

    #[test]
    fn nested_assign_depth_two_nests_two_with_nested_levels() {
        // `self.image.header.stamp.sec = 5` (2 intermediates: header, stamp).
        let (out, count, first) =
            rewrite_one_expr(parse_quote! { self.image.header.stamp.sec = 5 }, &["image"]);
        let expected: Expr = parse_quote! {
            {
                let __cer_rhs = 5;
                __cer_image.__cer_loan()?.__cer_with_nested_header(|__cer_v| __cer_v
                    .__cer_with_nested_stamp(|__cer_v| __cer_v.__cer_assign_sec(&(__cer_rhs))))?
            }
        };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "depth-2 (4-segment) nested chain"
        );
        assert_eq!(count, 1);
        assert_eq!(
            first,
            Some(("image".to_string(), "header.stamp.sec".to_string()))
        );
    }

    #[test]
    fn nested_fill_from_emits_chain_without_trailing_question() {
        // `self.image.header.frame_id.fill_from(src)` — fill_from leaf body,
        // NO trailing `?` (rides the user's own `?`), NOT counted as an
        // assign.
        let (out, count, first) = rewrite_one_expr(
            parse_quote! { self.image.header.frame_id.fill_from(src) },
            &["image"],
        );
        let expected: Expr = parse_quote! {
            __cer_image.__cer_loan()?.__cer_with_nested_header(|__cer_v| __cer_v.__cer_fill_from_frame_id(src))
        };
        assert_eq!(tok(&out), tok(&expected), "nested fill_from chain, no `?`");
        assert_eq!(count, 0, "fill_from never counts as an assign_rewrite");
        assert_eq!(first, None);
    }

    #[test]
    fn two_segment_assign_emission_roots_at_lazy_loan_receiver() {
        // The 2-segment (empty-intermediate) single-shim emission
        // roots at the lazy get-or-loan receiver `__cer_out.__cer_loan()?`.
        // It now HOISTS the RHS into `__cer_rhs`
        // first (matching the nested arm), so both the borrowed (str literal
        // pass-through) and the `&(rhs)` else-branch emit inside a
        // `{ let __cer_rhs = …; … }` block. The shim + borrow-adaptation are
        // otherwise unchanged.
        let (out_lit, count_lit, first_lit) =
            rewrite_one_expr(parse_quote! { self.out.data = "hello" }, &["out"]);
        let expected_lit: Expr = parse_quote! {
            {
                let __cer_rhs = "hello";
                __cer_out.__cer_loan()?.__cer_assign_data(__cer_rhs)?
            }
        };
        assert_eq!(
            tok(&out_lit),
            tok(&expected_lit),
            "str literal passes through unborrowed (hoisted into __cer_rhs)"
        );
        assert_eq!(count_lit, 1);
        assert_eq!(first_lit, Some(("out".to_string(), "data".to_string())));

        let (out_owned, _c, _f) =
            rewrite_one_expr(parse_quote! { self.out.data = value }, &["out"]);
        let expected_owned: Expr = parse_quote! {
            {
                let __cer_rhs = value;
                __cer_out.__cer_loan()?.__cer_assign_data(&(__cer_rhs))?
            }
        };
        assert_eq!(
            tok(&out_owned),
            tok(&expected_owned),
            "owned RHS gets the `&(rhs)` borrow adaptation (hoisted into __cer_rhs)"
        );
    }

    #[test]
    fn output_method_call_receiver_routes_through_lazy_loan() {
        // A bare method call on an OUTPUT port, the shape a
        // zero-field schema uses to publish, `self.pulse.emit()` — rewrites its
        // RECEIVER through the lazy get-or-loan `__cer_<port>.__cer_loan()?`, so
        // the loan (= the publish intent) happens on THIS call. `emit` is NOT
        // special-cased in the macro: there is no per-method allowlist. The
        // generic output-field receiver rewrite handles it uniformly, exactly
        // as it does the codegen Deref methods (`set_*`, `loan_*`, `push_*`,
        // `with_*`) — `emit` then resolves through `OutputProxy::DerefMut` to
        // `EmptyShm::emit`, which codegen emits ONLY for zero-field schemas.
        let (out, count, first) = rewrite_one_expr(parse_quote! { self.pulse.emit() }, &["pulse"]);
        let expected: Expr = parse_quote! { __cer_pulse.__cer_loan()?.emit() };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "an output method-call receiver must route through the lazy get-or-loan"
        );
        // A method call is neither an assign nor a fill_from, so it records no
        // assign-rewrite and no `first_assign`.
        assert_eq!(count, 0, "a method call is not an assign_rewrite");
        assert!(first.is_none(), "a method call records no first_assign");
    }

    #[test]
    fn keyword_intermediate_and_leaf_are_r_hash_stripped_in_idents() {
        // `self.out.r#type.r#match = 1`: the emitted `__cer_with_nested_*`
        // (intermediate) AND `__cer_assign_*` (leaf) idents strip the `r#`
        // (codegen names shims with the RAW schema name; `format_ident!`
        // panics on a `r#…` string), while `first_assign` KEEPS the user's
        // `r#` spelling in the dotted path.
        let (out, count, first) =
            rewrite_one_expr(parse_quote! { self.out.r#type.r#match = 1 }, &["out"]);
        let expected: Expr = parse_quote! {
            {
                let __cer_rhs = 1;
                __cer_out.__cer_loan()?.__cer_with_nested_type(|__cer_v| __cer_v.__cer_assign_match(&(__cer_rhs)))?
            }
        };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "r# stripped on both intermediate + leaf idents"
        );
        assert_eq!(count, 1);
        assert_eq!(
            first,
            Some(("out".to_string(), "r#type.r#match".to_string())),
            "first_assign keeps the user's r# spelling per segment"
        );
    }

    #[test]
    fn nested_assigns_count_and_first_records_declaration_order() {
        // TWO nested assigns in a block → count == 2, first_assign is the
        // FIRST in declaration order (with its dotted path).
        let (count, first) = rewrite_one_block(
            parse_quote! {{
                self.imu.orientation.x = 1.0;
                self.image.header.stamp.sec = 2;
            }},
            &["imu", "image"],
        );
        assert_eq!(count, 2, "both nested assigns are counted");
        assert_eq!(
            first,
            Some(("imu".to_string(), "orientation.x".to_string())),
            "first_assign is the earliest offender"
        );
    }

    #[test]
    fn nested_assign_hoists_same_port_rhs_before_the_chain() {
        // The RHS reads the SAME port being written — the hoist is what makes
        // this compile (the token oracle pins the ordering: the `let` binds
        // the recursed RHS `__cer_imu.__cer_with_nested_orientation(|__cer_v|
        // __cer_v.w)` FIRST, then the write chain borrows `__cer_imu`).
        let (out, _c, _f) = rewrite_one_expr(
            parse_quote! { self.imu.orientation.x = self.imu.orientation.w * 2.0 },
            &["imu"],
        );
        // The RHS is a bare READ, so it falls through to the leaf rewrite +
        // Deref, NOT a `__cer_with_nested_*` chain (only assigns / fill_from
        // rewrite to chains). An OUTPUT-port leaf reads through the
        // same lazy get-or-loan receiver (`__cer_imu.__cer_loan()?.orientation.w`
        // — reading an output field loans it, once; the write chain's second
        // `__cer_loan()` sees the proxy already present). The hoist is
        // load-bearing precisely because that read of `__cer_imu` cannot live
        // inside the closure that borrows `__cer_imu` mutably (E0502) — moving
        // it before the write chain fixes it (and sequences the two loans).
        let expected: Expr = parse_quote! {
            {
                let __cer_rhs = __cer_imu.__cer_loan()?.orientation.w * 2.0;
                __cer_imu.__cer_loan()?.__cer_with_nested_orientation(|__cer_v| __cer_v.__cer_assign_x(&(__cer_rhs)))?
            }
        };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "same-port RHS hoisted before the write chain"
        );
    }

    #[test]
    fn two_segment_assign_hoists_same_port_rhs_before_the_chain() {
        // The 2-SEGMENT arm now hoists the RHS the
        // same way the nested arm does. This is a COMPILE regression pin for
        // read-back-after-write within a tick: `self.out.y = self.out.x * 2.0`.
        // Without the hoist the write receiver `__cer_out.__cer_loan()?` and the
        // RHS's own OUTPUT-field read `__cer_out.__cer_loan()?.x` are two
        // overlapping EXPLICIT `&mut` loans of `__cer_out` inside one call
        // expression → E0499 (two-phase borrows rescue a receiver AUTOREF, not
        // an explicit `.__cer_loan()` call). The hoist sequences them: the RHS
        // loan is taken + released into `__cer_rhs` before the write loan.
        let (out, count, first) =
            rewrite_one_expr(parse_quote! { self.out.y = self.out.x * 2.0 }, &["out"]);
        // The RHS `self.out.x` is a bare READ of an output field, so — exactly
        // like the nested oracle above — it rewrites through the same lazy
        // get-or-loan receiver (`__cer_out.__cer_loan()?.x`) and is a
        // `Expr::Binary` (not a reference/literal), so the leaf takes the
        // `&(__cer_rhs)` borrow-adaptation branch.
        let expected: Expr = parse_quote! {
            {
                let __cer_rhs = __cer_out.__cer_loan()?.x * 2.0;
                __cer_out.__cer_loan()?.__cer_assign_y(&(__cer_rhs))?
            }
        };
        assert_eq!(
            tok(&out),
            tok(&expected),
            "same-port RHS hoisted before the 2-segment write chain"
        );
        assert_eq!(count, 1);
        assert_eq!(first, Some(("out".to_string(), "y".to_string())));
    }

    #[test]
    fn validate_external_source_sig_accepts_exact_valid_signature() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_ok(),
            "the exact `fn external_source(&mut self) -> ExternalSource` must validate"
        );
    }

    #[test]
    fn validate_external_source_sig_accepts_qualified_return_path() {
        // The validator keys on the LAST return-path segment, so a fully
        // qualified `ExternalSource` return is also valid.
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) -> ::cerulion_core::graph::node::ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_ok(),
            "a qualified `...::ExternalSource` return path must validate"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_extra_arg() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self, _extra: u8) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "an extra parameter beyond the receiver must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_wrong_return_type() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) -> u8 {
                0
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a non-ExternalSource return type must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_missing_return_type() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) {}
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "an absent return type (unit) must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_self_by_value() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a by-value `self` receiver must be rejected (must be `&mut self`)"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_shared_ref_receiver() {
        // `&self` (not `&mut self`) — the runtime needs mutable access to bind
        // the source, so a shared receiver is rejected.
        let method: ImplItemFn = parse_quote! {
            fn external_source(&self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a `&self` (non-mut) receiver must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_no_receiver() {
        let method: ImplItemFn = parse_quote! {
            fn external_source() -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a free-function shape with no receiver must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_async_fn() {
        // `async fn` returns a future, not `ExternalSource` — the runtime would
        // call a future-returning shim and fail downstream, so reject it here.
        let method: ImplItemFn = parse_quote! {
            async fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "an `async fn external_source` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_const_fn() {
        let method: ImplItemFn = parse_quote! {
            const fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a `const fn external_source` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_unsafe_fn() {
        let method: ImplItemFn = parse_quote! {
            unsafe fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "an `unsafe fn external_source` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_extern_abi_fn() {
        let method: ImplItemFn = parse_quote! {
            extern "C" fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "an `extern \"C\" fn external_source` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_generic_params() {
        let method: ImplItemFn = parse_quote! {
            fn external_source<T>(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a generic `external_source<T>` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_rejects_where_clause() {
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) -> ExternalSource
            where
                Self: Sized,
            {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_err(),
            "a `where`-clause on `external_source` must be rejected"
        );
    }

    #[test]
    fn validate_external_source_sig_still_accepts_exact_valid_signature_after_modifier_gates() {
        // Re-assert the exact valid shape stays `Ok` now that the modifier gates
        // run first (guards against an over-broad reject regressing the happy path).
        let method: ImplItemFn = parse_quote! {
            fn external_source(&mut self) -> ExternalSource {
                ExternalSource::HostDriven
            }
        };
        assert!(
            validate_external_source_sig(&method).is_ok(),
            "the exact `fn external_source(&mut self) -> ExternalSource` must still validate"
        );
    }
}
