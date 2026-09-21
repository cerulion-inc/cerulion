// An uncapturable field of an ENUM VARIANT is a loud compile
// error whose remedies are the ones that exist THERE.
//
// The sibling `state_derive_uncapturable_field.rs` pins the struct case, whose
// message names `#[cerulion(reconstruct)]`. That advice is FALSE inside an
// enum, and the derive refuses the attribute on a variant's field — so before
// this was fixed the user was sent round a closed loop, MEASURED both halves:
// `enum Link { Connected(TcpStream) }` raised `E0277` naming the escape, and
// writing the escape raised `#[cerulion(reconstruct)] is not supported on an
// enum variant's field` naming the `E0277`.
//
// The message is now `CerulionVariantMember`'s own
// `#[diagnostic::on_unimplemented]` (`cerulion_core/src/state.rs`): a blanket
// alias over `CerulionState` that exists only to own this text. It renders
// instead of the real trait's because (a) `CerulionState` is NOT a supertrait
// — with it as one rustc reports the supertrait bound and this message never
// appears, the same trap the `CanonicalMapKey` docs record — and (b) the
// blanket impl carries `#[diagnostic::do_not_recommend]`.
//
// This fixture pins the RENDERING, which is toolchain-fragile, hence the
// `#[ignore]`d group.
//
// It ALSO pins the COUNT at 1. Every member use in the enum emission names
// `CerulionVariantMember`; MEASURED, leaving even one `CerulionState` use
// behind raises its own obligation and renders the misleading note as a SECOND
// error block. The token-level half of that pin gates every PR
// (`an_enum_variants_members_are_bound_through_the_diagnostic_that_is_true_there`).

use cerulion_core::state::CerulionState;

struct CudaContext;

#[derive(CerulionState)]
enum Link {
    Idle,
    Connected(CudaContext),
}

fn main() {}
