#![allow(unexpected_cfgs)]
// `unexpected_cfgs` is allowed for the reason 27 sibling fixtures allow it: the
// macro's `#[cfg(feature = "cdylib")]` block draws a warning whose rendering
// drifts between rustc versions, and a snapshot carrying it would be a
// toolchain trap rather than a pin on this fixture's own diagnostic.
// The OTHER attribute order of the same mistake — the derive
// written ABOVE `#[cerulion_node]`.
//
// The two orders need two detectors, and neither one can cover both:
//
// - rustc expands the FIRST macro attribute in source order, and expanding a
//   `derive` REMOVES the `derive` attribute from the item. So when the derive
//   is written first, `#[cerulion_node]` runs against a struct that no longer
//   carries it and cannot see the collision coming.
// - Symmetrically, an attribute macro consumes its own attribute, so a derive
//   expanding AFTER `#[cerulion_node]` never sees a `#[cerulion_node]`.
//
// Each order is therefore caught by whichever macro still has the other's
// attribute in hand: this one by the derive, its sibling
// (`state_derive_on_node_struct.rs`) by the attribute macro. Both emit the ONE
// shared message, so the user reads the same story either way.
//
// Holding the attribute is not the same as RECOGNISING it, and this fixture
// pins only the canonical spelling. `use ... cerulion_node as my_node;` is
// ordinary Rust and defeats the name match the derive does here; the second,
// name-free signal that covers it — a node must declare a PORT, and in this
// order the port attributes are still on the struct — is pinned by
// `state_derive_before_an_aliased_node_attr.rs`.
//
// The derive emits ONLY the error and no impl, which is what keeps this to a
// single diagnostic: `#[cerulion_node]` still runs afterwards and still emits
// the fold-in impl, so there is nothing left for rustc's E0119 to find, and the
// `E0063` the derive would otherwise raise (its `cer_read` cannot name the
// macro's injected `__cer_rt` field) never appears.
//
// Deliberately NO `Default` in the derive list. In this order the attribute
// macro cannot see the user's derives at all, so it injects its own
// `#[derive(Default)]` — a `Default` written here would add a second, unrelated
// E0119 that has nothing to do with what this fixture pins.
//
// OUR `compile_error!` ⇒ the BLOCKING group.

use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

#[derive(CerulionState)]
#[cerulion_node(period_ms = 10)]
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
