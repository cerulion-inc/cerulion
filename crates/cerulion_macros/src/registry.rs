// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-macro port registry for `#[cerulion_node]` ⇄ `#[cerulion_node_impl]`.
//!
//! Avoids the historical footgun where `#[cerulion_node_impl(inputs(...),
//! outputs(...))]` had to re-declare every port that `#[cerulion_node]`
//! already named via field attributes (`#[input]` / `#[output]`).
//!
//! # Mechanism
//!
//! `#[cerulion_node]` writes the resolved port list (name + type) into a
//! process-wide `Mutex<HashMap>` keyed by the struct identifier as soon as
//! it expands. `#[cerulion_node_impl]` reads from the same map at expand
//! time, looking up by `impl <Name>`'s self type. Both macros run inside
//! the same proc-macro DLL invocation, so the static map is shared across
//! every `#[cerulion_node*]` expansion in a single `cargo build`.
//!
//! Source order matters: the struct's `#[cerulion_node]` MUST expand
//! before its impl block's `#[cerulion_node_impl]` so the impl macro's
//! lookup succeeds. This matches the natural source order users already
//! write (`struct Foo { ... }` declared above `impl Foo { ... }`), and
//! the impl macro emits a clear error if the struct entry is missing.
//!
//! # Limitations
//!
//! * Only the unqualified struct identifier is the lookup key, so two
//!   `#[cerulion_node]` structs named `Foo` in different modules of the
//!   same crate will collide. The current contract is "one node type per
//!   struct name per crate"; cross-crate collisions are impossible because
//!   each crate gets its own proc-macro instantiation.
//! * Port types are stored as token strings (re-parsed by the impl macro)
//!   because `syn::Type` is not `Send`/`Sync` and can't live in a `static`
//!   directly. The re-parse is infallible because the source string came
//!   from a previously-parsed `syn::Type`.

use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};

use quote::ToTokens;

/// One declared port: name (Rust ident) + the user-written type.
///
/// `type_tokens` is the `to_token_stream().to_string()` of the field's
/// `syn::Type`. Re-parse via `syn::parse_str::<syn::Type>` from the
/// consumer side.
///
/// The per-output variable-field name lists are gone —
/// codegen emits a uniform `__cer_assign_<field>` / `__cer_fill_from_<field>`
/// shim per schema field, so the `#[cerulion_node_impl]` rewriter is
/// schema-blind and classifies on the port name alone.
#[derive(Debug, Clone)]
pub struct RegisteredPort {
    pub name: String,
    pub type_tokens: String,
}

/// Per-struct registry entry recording every declared input/output.
///
/// It also carries the determinism suppression flags
/// `allow_non_deterministic` / `uses_live_io` — written by `#[cerulion_node]`
/// from its `#[cerulion_node(...)]` attrs, read back by
/// `#[cerulion_node_impl]` to gate the deny lint.
///
/// There is intentionally NO `determinism_warn_keys` field: warn-class
/// surfacing is the deferred core half's job and cannot flow through this
/// registry — the registry is process-local to the proc-macro DLL and is gone
/// by the time the graph-load CLI runs, so anything recorded here would be
/// unreadable (dead). The macro half emits deny errors only. See
/// `determinism.rs`'s "Warn-surfacing is deferred" module note.
#[derive(Debug, Clone, Default)]
pub struct NodePortEntry {
    pub inputs: Vec<RegisteredPort>,
    pub outputs: Vec<RegisteredPort>,
    /// Blanket determinism opt-out
    /// (`#[cerulion_node(allow_non_deterministic)]`).
    pub allow_non_deterministic: bool,
    /// IO-class determinism opt-out
    /// (`#[cerulion_node(uses_live_io)]`).
    pub uses_live_io: bool,
    /// The node carries the `external` trigger-policy attr
    /// (`#[cerulion_node(external)]`). Written by `#[cerulion_node]`, read back
    /// by `#[cerulion_node_impl]` so it can REQUIRE the user-written
    /// `external_source` method on external nodes (and REJECT it on non-external
    /// ones). The trigger policy itself flows to the runtime via
    /// `MacroPolicy::External` — this flag is only the macro-side coupling that
    /// gates the required-method check.
    pub external: bool,
}

fn registry() -> &'static Mutex<HashMap<String, NodePortEntry>> {
    static REGISTRY: OnceLock<Mutex<HashMap<String, NodePortEntry>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Convert a `syn::Type` into a stable string form for the registry.
pub fn type_to_string(ty: &syn::Type) -> String {
    ty.to_token_stream().to_string()
}

/// Re-parse a token-string from the registry back into a `syn::Type`.
pub fn parse_type_tokens(s: &str) -> syn::Result<syn::Type> {
    syn::parse_str::<syn::Type>(s)
}

/// Insert (or overwrite) the port entry for `struct_name`.
pub fn register(struct_name: &str, entry: NodePortEntry) {
    if let Ok(mut guard) = registry().lock() {
        guard.insert(struct_name.to_string(), entry);
    }
}

/// Look up the registered port entry for `struct_name`, if any.
pub fn lookup(struct_name: &str) -> Option<NodePortEntry> {
    registry()
        .lock()
        .ok()
        .and_then(|g| g.get(struct_name).cloned())
}
