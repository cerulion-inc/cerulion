// SPDX-License-Identifier: AGPL-3.0-only
//! Compile-fail tests whose diagnostics rustc renders, run on demand.
//!
//! The blocking suite, the `#[cerulion_node]` diagnostics the macros emit
//! themselves plus the compile-pass programs, lives in
//! `crates/test_fixtures/node_macro_ui` (`cargo test -p node_macro_ui`).
//! This file keeps the groups that pin a core type contract through an error
//! rustc writes, beside the types they pin.
//!
//! `tests/ui/const_eval/*.rs` and `tests/ui/type_error/*.rs` are the **ignored**
//! group: these fixtures assert diagnostics whose *rendering* is produced by
//! rustc, not by us, and is NOT stabilized across toolchains. Gating CI on that
//! rendering plants a time-bomb that breaks on the next compiler bump for purely
//! cosmetic reasons, so the whole group is `#[ignore]`d. Two flavors live here:
//!
//! - **`const_eval/`** — errors raised by `const { panic!(...) }` invariants
//!   (`MaxSliceLen::const_new`, `ShmMessage::_SHM_INVARIANTS`). The
//!   `error[E0080]: evaluation panicked: <msg>` line is stable, but rustc's
//!   surrounding const-eval diagnostic *rendering* (note layout, `$RUST/...` vs
//!   `src/...` spans, `---` vs `^^^` underlines) drifts between stable
//!   toolchains.
//! - **`type_error/`** — fixtures pinning a *type-level contract* whose error is
//!   a plain rustc type error, not one of our `compile_error!`s.
//!   `var_field_str_into_byte_field_rejected.rs` pins that assigning a `&str`
//!   RHS into a `uint8[]` (byte) variable field is a COMPILE ERROR (E0308), not
//!   a silent UTF-8 write — a future widening of the shim to `AsRef<[u8]>`
//!   would let it compile, so the *contract* is worth gating. Beside it sit the
//!   write-only-proxy diagnostics: `var_field_compound_assign_proxy_diagnostic`
//!   and `var_field_index_proxy_diagnostic` pin that `+=` / `[i]` on a variable
//!   field render E0277 with OUR `#[diagnostic::on_unimplemented]` hoist text,
//!   and `var_field_method_call_no_method_on_proxy` pins the E0599 naming the
//!   proxy ZST. `node_hand_written_state_impl` pins
//!   the E0119 a hand-rolled `impl CerulionState for <node>` gets — the one
//!   collision with the fold-in that no macro can catch earlier, because an
//!   attribute macro is handed only its own item and a sibling `impl` block is
//!   a separate one (the two REDUNDANT-DERIVE orders *are* caught in our own
//!   words, and live in the blocking group). All assert rustc's diagnostic
//!   *rendering* — toolchain-fragile, the same class as the const-eval cases —
//!   so they are `#[ignore]`d alongside them.
//!
//! Run the ignored group on demand and re-bless after a toolchain change:
//!
//! ```bash
//! cargo test -p cerulion_core --test macro_compile_fail_test -- --ignored
//! TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test -- --ignored
//! ```

/// Rustc-diagnostic compile-fail cases — `#[ignore]`d because they assert
/// rustc's unstable diagnostic rendering (const-eval panic rendering in
/// `const_eval/`, and rustc type-error E0308 rendering in `type_error/`; see
/// module docs). Kept in version control and runnable locally via
/// `-- --ignored`; not a CI gate.
#[test]
#[ignore = "asserts rustc's unstable diagnostic rendering (const-eval panic rendering + rustc type-error E0308); run with --ignored and re-bless after toolchain bumps"]
fn compile_fail_rustc_diagnostic_tests() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/const_eval/*.rs");
    t.compile_fail("tests/ui/type_error/*.rs");
}
