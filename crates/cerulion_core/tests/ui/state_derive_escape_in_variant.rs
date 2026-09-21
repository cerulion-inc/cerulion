// `#[cerulion(...)]` on an enum variant's field is refused,
// and the remedy offered depends on the KEY.
//
// This is OUR OWN `compile_error!` — fully under our control and stable across
// toolchains — so it belongs in the BLOCKING group, unlike the sibling
// `type_error/state_derive_uncapturable_variant_field.rs`, whose rendering is
// rustc's.
//
// The two variants below are the whole contract:
//
// - `reconstruct` (`Link`) — the wrap remedy COMPILES but produces a variant
//   that can never be restored (`cer_read` reports `Unrestorable`, and an enum
//   restores by calling `cer_read`). So the message leads with the remedy that
//   actually restores — hold the handle OUTSIDE the enum — and offers the wrap
//   second, with that cost stated.
// - `serde` (`Payload`) — the wrap remedy WORKS: the wrapper reads back, so
//   the variant round-trips. The message must not send this user's field out
//   of the enum.
//
// A single shared message cannot be true for both, which is why the refusal
// branches. The ordering pin lives in the macro crate's own tests
// (`a_reconstruct_in_a_variant_leads_with_the_remedy_that_restores`); this
// fixture pins the rendered text a user actually sees.

use cerulion_core::state::CerulionState;

struct CudaContext;

#[derive(serde::Serialize, serde::Deserialize)]
struct Config {
    n: u32,
}

#[derive(CerulionState)]
enum Link {
    Idle,
    Connected {
        #[cerulion(reconstruct)]
        cuda: CudaContext,
    },
}

#[derive(CerulionState)]
enum Payload {
    Empty,
    Blob {
        #[cerulion(serde)]
        body: Config,
    },
}

fn main() {}
