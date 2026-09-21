// SPDX-License-Identifier: AGPL-3.0-only
//! Compile-fail tests for `#[cerulion_node]`.
//!
//! Uses `trybuild` to verify that the macro produces clear errors
//! for invalid usage. Run with:
//!
//! ```bash
//! cargo test -p cerulion_core --test macro_compile_fail_test
//! ```
//!
//! To regenerate `.stderr` snapshots:
//!
//! ```bash
//! TRYBUILD=overwrite cargo test -p cerulion_core --test macro_compile_fail_test
//! ```
//!
//! # Do NOT split this into one `TestCases` per expectation — measured, a net loss
//!
//! `compile_fail_tests` mixes 80 compile-fail fixtures with 11 compile-pass ones, and that
//! is deliberate. Because the set contains a `pass` case, trybuild sets `has_pass`, which
//! costs the batch path (`run.rs`: `else if project.keep_going && !project.has_pass`) — so
//! the group pays a `cargo clean` + `cargo build --bin` per fixture. Splitting the groups to
//! recover batching looks obviously right and is a NET LOSS on CI; the measurement
//! is below.
//!
//! `has_pass` also pins the cargo PROFILE for the whole set (`cargo.rs` picks `build` vs
//! `check` from it), so two groups mean two `build_dependencies` passes over the dependency
//! tree — `.rlib` for the pass group, `.rmeta` for the compile-fail group. CI's cargo cache
//! does not restore trybuild's scratch project (`target/tests/trybuild/`; shard 0 is the
//! saver and does not run this binary), so that extra pass is paid on every run and costs
//! more than batching saves. MEASURED on CI: a split drops `compile_fail_tests` 545 s → 355 s,
//! but the separate pass group costs ~348 s of its own, taking the shard's nextest wall
//! 560–574 s → 711 s (**+158 s**, ~+2.5 min on the step). A local cold A/B suggests a
//! 1.51× WIN that does not transfer — on a developer machine the dependency passes are a much smaller
//! share of the total than on CI, so local timings are not a valid proxy here.
//!
//! The constraint is structural: nothing inside a split can avoid the second dependency
//! pass while trybuild derives one profile per `TestCases`. Attack the ~545 s some other
//! way; a split changes no diagnostic output, only the time.
//!
//! # Two case groups
//!
//! `tests/ui/*.rs` — the **blocking** group: every fixture asserts an error
//! message that *we* emit (macro-attribute validation, our own `compile_error!`
//! diagnostics). Those strings are fully under our control, so the snapshots are
//! stable across rustc versions and safe to gate CI on. The glob is
//! **non-recursive**, so it does NOT descend into the subdirectories below
//! (`const_eval/`, `type_error/`, `pass/`).
//!
//! `tests/ui/const_eval/*.rs` and `tests/ui/type_error/*.rs` — the **ignored**
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

#[test]
fn compile_fail_tests() {
    let t = trybuild::TestCases::new();
    // `tests/ui/*.rs` is a non-recursive glob — it does NOT descend into
    // `tests/ui/pass/`, so the determinism pass fixtures below are not
    // double-claimed as compile-fail cases.
    t.compile_fail("tests/ui/*.rs");
    // Nodes that opt out via `allow_non_deterministic` /
    // `uses_live_io` must COMPILE (the lint is suppressed). Pass fixtures
    // live in `tests/ui/pass/` so the compile-fail glob above skips them.
    t.pass("tests/ui/pass/*.rs");
}

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
