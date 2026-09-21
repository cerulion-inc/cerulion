// An uncapturable field is a LOUD compile error naming the
// FIELD, its type, and both fixes.
//
// The message is `CerulionState`'s own `#[diagnostic::on_unimplemented]`
// (`cerulion_core/src/state.rs`); all the derive does is point rustc at the
// field's span via a `where`-clause predicate. This fixture pins the
// RENDERING, which is toolchain-fragile — hence the `#[ignore]`d group, the
// same class as the write-only-proxy diagnostics.
//
// It ALSO pins the COUNT, because trybuild snapshots EVERY diagnostic.
// Deleting the `where`-clause predicate (so the
// body's uses speak instead): this file renders 1 error block on the shipped
// code and 2 without it, the second pointing at `#[derive(..)]` on
// line 18 rather than at `cuda` on line 21 — an IDE squiggle in the wrong
// place. The memo reports 6 for the same change; it measured a
// sketch that spanned all six use sites separately, where this emission spans
// only the predicate, so rustc dedupes five of them.

use cerulion_core::state::CerulionState;

struct CudaContext;

#[derive(CerulionState)]
struct SlamNode {
    pose: f64,
    cuda: CudaContext,
}

fn main() {}
