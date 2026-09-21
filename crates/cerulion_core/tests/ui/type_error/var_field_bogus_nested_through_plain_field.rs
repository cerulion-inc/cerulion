#![allow(unexpected_cfgs)]
//! The recursive rewrite is schema-BLIND — it cannot know a
//! segment's field kind (the registry gap). A nested path routed through a
//! PLAIN variable field (`self.image.data.len = 3`, where `data` is a
//! `uint8[]`, not a nested substruct) rewrites to
//! `__cer_image.__cer_with_nested_data(|__cer_v| __cer_v.__cer_assign_len(&(3)))?`
//! — and `__cer_with_nested_data` does not exist on the Image writer, so
//! rustc rejects it with E0599 naming a `__cer_*` method. This is the
//! accepted schema-blind cost: a `__cer_*`-named diagnostic points at
//! generated plumbing rather than silently compiling.
//!
//! Lives in the `#[ignore]`d `type_error/` group: the E0599 rendering (and
//! the Deref-chain candidate list) is rustc's — toolchain-fragile,
//! re-blessed after toolchain bumps.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct BogusNestedThroughPlainFieldNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl BogusNestedThroughPlainFieldNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `data` is a plain `uint8[]` variable field, not a nested substruct
        // — should produce E0599: no method `__cer_with_nested_data`.
        self.image.data.len = 3;
        Ok(())
    }
}

fn main() {}
