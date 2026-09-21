#![allow(unexpected_cfgs)]
// `unexpected_cfgs` is allowed for the reason 27 sibling fixtures allow it: the
// macro's `#[cfg(feature = "cdylib")]` block draws a warning whose rendering
// drifts between rustc versions, and a snapshot carrying it would be a
// toolchain trap rather than a pin on this fixture's own diagnostic.
// The same mistake as `state_derive_before_node_attr.rs`,
// written through an ALIASED import of the node attribute.
//
// `use cerulion_core::prelude::cerulion_node as my_node;` is ordinary Rust, and
// it defeats every NAME match: the derive expands first and sees an attribute
// that is not called `cerulion_node`, and expanding the derive REMOVES the
// `derive` attribute, so `#[cerulion_node]` then runs against a struct that no
// longer carries it and its own detector finds nothing either. MEASURED before
// the fix, on exactly this fixture's shape:
//
//     error[E0119]: conflicting implementations of trait `CerulionState`
//     error[E0063]: missing field `__cer_rt` in initializer of `AliasedNode`
//        = note: upstream crates may add a new impl of trait `CerulionState`
//                for type `Vector3` in future versions
//
// — two errors and a note, all three pointing away from the one line to
// delete: an `__cer_rt` the user never wrote and cannot name, and a `Vector3`
// that has nothing to do with it.
//
// The second signal is therefore SHAPE, not a name: a `#[cerulion_node]` must
// declare at least one `#[input]`/`#[output]` field
// (`no_input_or_output_attrs_rejected.rs` pins that), and in THIS order those
// attributes are still on the struct, because the attribute macro that strips
// them has not run yet. So a struct being derived that declares a port is a
// node whatever its attribute is called, and the message is the same one both
// canonical orders give.
//
// What is still NOT caught, deliberately, because no signal exists for it:
// renaming the DERIVE (`use CerulionState as Capturable`) in the
// attribute-FIRST order. There the attribute macro holds the derive list and
// cannot resolve `Capturable` to anything, and unlike a node struct a derive
// list carries no shape to read. That lands on E0119, which is loud and names
// both sites — see `type_error/node_hand_written_state_impl.rs`.
//
// OUR `compile_error!` ⇒ the BLOCKING group.

use cerulion_core::prelude::cerulion_node as my_node;
use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

#[derive(CerulionState)]
#[my_node(period_ms = 10)]
struct AliasedNode {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl AliasedNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

fn main() {}
