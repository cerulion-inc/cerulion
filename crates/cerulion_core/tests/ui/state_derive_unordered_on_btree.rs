// `#[cerulion(unordered)]` on a BTree container is REFUSED
// at the attribute, not inside `std`.
//
// The escape's advertised property is "no `Ord` on the key". Rebuilding a
// `BTreeMap` goes through `insert`, whose own bound is `K: Ord`, so the escape
// can lift nothing there — and before this refusal existed the attribute
// accepted the field and then failed with `the trait bound `f64: Ord` is not
// satisfied ... required by a bound in `BTreeMap::<K, V, A>::insert``, an
// error pointing into `std` for a field the attribute had just promised to
// accept.
//
// This is OUR `compile_error!`, so its text is stable across toolchains and
// the fixture belongs in the BLOCKING group.

use cerulion_core::state::CerulionState;
use std::collections::BTreeMap;

#[derive(CerulionState, Default)]
struct Node {
    #[cerulion(unordered)]
    grid: BTreeMap<u32, u8>,
}

fn main() {}
