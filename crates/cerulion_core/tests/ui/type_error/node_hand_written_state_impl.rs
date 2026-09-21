#![allow(unexpected_cfgs)]
// `unexpected_cfgs` is allowed for the reason 27 sibling fixtures allow it: the
// macro's `#[cfg(feature = "cdylib")]` block draws a warning whose rendering
// drifts between rustc versions, and a snapshot carrying it would be a
// toolchain trap rather than a pin on this fixture's own diagnostic.
// A HAND-WRITTEN `impl CerulionState for <node>` beside a
// `#[cerulion_node]` struct is rustc's E0119, and that is the END of the
// ladder, not a gap.
//
// The REDUNDANT-DERIVE orders are caught in our own words
// (`state_derive_on_node_struct.rs`, `state_derive_before_node_attr.rs`, and
// `state_derive_before_an_aliased_node_attr.rs`) because in each of them one
// macro still holds the other's ATTRIBUTE, or — when a `use ... as ...` alias
// makes that attribute unrecognisable — the struct itself still carries the
// PORT that proves it is a node. A hand-rolled trait impl offers neither: it
// is a SEPARATE ITEM, and an attribute macro is handed only the item it is
// written on, so `#[cerulion_node]` cannot see a sibling `impl` block in any
// order and the derive is not involved at all. There is no earlier place to
// catch this, and no shape on the node to read instead.
//
// One redundant-derive spelling lands here too, for the same
// nothing-left-to-read reason: a renamed DERIVE (`use CerulionState as
// Capturable`) with the attribute written FIRST. The attribute macro holds the
// derive list, cannot resolve `Capturable`, and a derive list carries no shape.
//
// What this fixture pins is that the diagnostic is nevertheless SUFFICIENT,
// because it is exactly the thing that would be lost silently if the fold-in's
// emission ever moved somewhere rustc attributes differently (a `const _: () =
// { .. }` wrapper, a different span): E0119 names the TRAIT, the TYPE, the
// user's own `impl` as "first implementation here", and `#[cerulion_node]`
// itself as the conflicting one. A user reading it can see both halves.
//
// The supported way to change WHAT a node captures is the per-field escapes —
// `#[cerulion(reconstruct)]`, `#[cerulion(serde)]`, `#[cerulion(unordered)]` —
// which the fold-in reads from the same place the derive reads them. A hand
// impl could not do better regardless: it cannot name the hidden `__cer_rt`
// field the macro injects, so its `cer_read` could only report `Unrestorable`,
// which is what the fold-in already does.
//
// rustc's OWN rendering ⇒ the `#[ignore]`d group, alongside the other
// `type_error/` fixtures. Run with `-- --ignored`.

use cerulion_core::prelude::*;
use cerulion_core::state::{CerulionState, StateCursor, StateError, StateSink};
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct HandImplNode {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl HandImplNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

impl CerulionState for HandImplNode {
    const STATE_SHAPE: u64 = 1234;
    fn cer_capture(&self, sink: &mut dyn StateSink) -> Result<(), StateError> {
        self.count.cer_capture(sink)
    }
    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let _ = src;
        Err(StateError::Unrestorable {
            type_name: "HandImplNode",
        })
    }
}

fn main() {}
