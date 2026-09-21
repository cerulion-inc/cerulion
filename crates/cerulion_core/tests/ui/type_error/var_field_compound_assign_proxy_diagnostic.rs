#![allow(unexpected_cfgs)]
//! Compound assignment (`+=`) on a variable-length output
//! field is diagnosed by the CODEGEN-emitted write-only proxy, not by the
//! macro. The schema-blind rewriter leaves non-`=` shapes to the leaf
//! rewrite, so `self.image.data += …` lowers to `__cer_image.data += …`
//! — the proxy ZST field on `ImageShm` — whose `AddAssign` impl is gated
//! on a never-implemented marker trait carrying
//! `#[diagnostic::on_unimplemented]`. rustc renders E0277 with OUR
//! message ("… are unsupported on variable-length fields. Replace the
//! whole value with self.<port>.data = expr, or write incrementally with
//! self.<port>.data.fill_from(|buf| …)").
//!
//! Lives in the `#[ignore]`d `type_error/` group: the message TEXT is
//! ours, but the E0277 rendering (notes, spans) is rustc's — toolchain-
//! fragile, re-blessed after toolchain bumps.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct CompoundAssignProxyNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl CompoundAssignProxyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce E0277 rendered with the proxy's on_unimplemented
        // message (unsupported on variable-length fields; use `=` or
        // `fill_from`).
        self.image.data += b"abc".as_slice();
        Ok(())
    }
}

fn main() {}
