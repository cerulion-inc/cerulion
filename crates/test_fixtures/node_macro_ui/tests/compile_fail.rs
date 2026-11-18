// SPDX-License-Identifier: AGPL-3.0-only
//! Compile-fail tests for `#[cerulion_node]` and the derives beside it.
//!
//! Every `tests/ui/*.rs` fixture must fail to compile with exactly the
//! compiler output in the `.stderr` file next to it. Each fixture fails on a
//! diagnostic the macros emit themselves (attribute validation, our own
//! `compile_error!` messages); the `.stderr` also holds any rustc errors and
//! warnings around it. Run with:
//!
//! ```bash
//! cargo test -p node_macro_ui --test compile_fail
//! ```
//!
//! To regenerate the `.stderr` snapshots:
//!
//! ```bash
//! TRYBUILD=overwrite cargo test -p node_macro_ui --test compile_fail
//! ```
//!
//! # Why the compile-pass programs are not in this `TestCases`
//!
//! `tests/ui/pass/*.rs` are example targets of this package (listed in its
//! `Cargo.toml`, run by `tests/pass_programs.rs`), not `t.pass(...)` cases. A
//! single pass case sets trybuild's `has_pass`, and that changes how the whole
//! set runs (trybuild 1.0.125, `run.rs` and `cargo.rs`):
//!
//! - every fixture gets its own `cargo clean` plus `cargo build`, one after
//!   another, instead of one `cargo check --keep-going` over all of them;
//! - the scratch project builds its dependencies to `.rlib` rather than
//!   `.rmeta`.
//!
//! Splitting pass and fail into two `TestCases` instead builds the
//! dependencies twice, because trybuild picks one profile per set. With the
//! pass programs built by cargo itself, this set is fail-only: one dependency
//! pass, in check mode, and one batched check of every fixture.
//!
//! Check mode stops before code generation. A fixture whose error only appears
//! at code generation would compile under it, and trybuild then fails that
//! fixture ("Expected test case to fail to compile, but it succeeded."), so
//! such a regression is loud.
//!
//! The scratch project trybuild builds copies this package's dependencies and
//! dev-dependencies, which is why the suite lives in a package whose only
//! dev-dependencies are the crates the fixtures name and trybuild.
//!
//! A fixture that hits a cfg the macros emit (`feature = "cdylib"`) gets
//! rustc's unexpected-cfg warning, whose note lists the features of the
//! package that consumes the macro. This package declares none, so those
//! `.stderr` notes read `no expected values for feature`.
//!
//! The rustc-rendered groups, whose text drifts between toolchains, stay
//! `#[ignore]`d in `crates/cerulion_core/tests/macro_compile_fail_test.rs`.

/// The compile-fail fixtures this suite holds. trybuild passes a glob that
/// matches nothing ("There are no trybuild tests enabled yet."), so a fixture
/// lost in a move fails here instead; lower it when a fixture is removed on
/// purpose.
const FIXTURE_FLOOR: usize = 96;

#[test]
fn compile_fail_tests() {
    let ui = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/ui");
    let fixtures = std::fs::read_dir(&ui)
        .unwrap_or_else(|e| panic!("read {}: {e}", ui.display()))
        .map(|entry| entry.expect("a directory entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "rs"))
        .count();
    assert!(
        fixtures >= FIXTURE_FLOOR,
        "tests/ui/*.rs holds {fixtures} fixtures, fewer than {FIXTURE_FLOOR}"
    );
    let t = trybuild::TestCases::new();
    // A non-recursive glob: it does not descend into `tests/ui/pass/`.
    t.compile_fail("tests/ui/*.rs");
}
