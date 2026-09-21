#![allow(unexpected_cfgs)]
//! Indexing a variable-length output field
//! (`self.image.data[0] = …`) is diagnosed by the CODEGEN-emitted
//! write-only proxy — the headline case the proxy exists for. The
//! schema-blind rewriter only intercepts simple `=` on the WHOLE field,
//! so the indexed LHS falls through to the leaf rewrite and lands on the
//! proxy ZST, whose `Index`/`IndexMut` impls are gated on a
//! never-implemented marker trait carrying
//! `#[diagnostic::on_unimplemented]` — rustc renders E0277 with OUR
//! message instead of a bare "no field `data`" / "cannot index" error.
//!
//! Lives in the `#[ignore]`d `type_error/` group: the message TEXT is
//! ours, but the E0277 rendering is rustc's — toolchain-fragile,
//! re-blessed after toolchain bumps.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct IndexProxyNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl IndexProxyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce E0277 rendered with the proxy's on_unimplemented
        // message (unsupported on variable-length fields; use `=` or
        // `fill_from`).
        self.image.data[0] = 1u8;
        Ok(())
    }
}

fn main() {}
