#![allow(unexpected_cfgs)]
// `unexpected_cfgs` is allowed for the reason 27 sibling fixtures allow it: the
// macro's `#[cfg(feature = "cdylib")]` block draws a warning whose rendering
// drifts between rustc versions, and a snapshot carrying it would be a
// toolchain trap rather than a pin on this fixture's own diagnostic.
// An EXPLICIT `#[derive(CerulionState)]` on a node struct is
// REFUSED in our own words, because `#[cerulion_node]` now emits that impl
// itself (the fold-in).
//
// Before the refusal this was rustc's `E0119` — "conflicting implementations of
// trait `CerulionState` for type `Counter`" — which is loud but tells a user
// upgrading a node nothing about WHY a derive they wrote last week is suddenly
// a duplicate, and buries the remedy. Worse, the derive also expanded over the
// `#[output]` port field, so it dragged a second, unrelated failure along.
//
// This fixture pins the ATTRIBUTE-FIRST order (`#[cerulion_node]` above the
// derive), which is where the attribute macro can see the derive in its own
// attrs. The derive-first order is pinned by the sibling fixture
// `state_derive_before_node_attr.rs`, and is caught from inside the DERIVE.
//
// This is OUR `compile_error!`, so its text is stable across toolchains and the
// fixture belongs in the BLOCKING group. Exactly ONE error is expected: the
// emission drops the `CerulionState` entry from the re-emitted struct, so
// neither rustc's E0119 nor the derive's own port-field failure follows it.

use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(CerulionState, Default)]
struct Counter {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl Counter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

fn main() {}
