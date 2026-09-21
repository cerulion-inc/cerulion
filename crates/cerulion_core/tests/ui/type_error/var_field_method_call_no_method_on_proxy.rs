#![allow(unexpected_cfgs)]
//! An arbitrary method call on a variable-length output
//! field (`self.image.data.push(1u8)`) is no longer intercepted by the
//! macro (the earlier friendly compile_error died with the
//! variable-field tables). It lowers via the leaf rewrite to
//! `__cer_image.data.push(1u8)` — a method call on the write-only proxy
//! ZST — and fails with rustc's E0599 naming the proxy type
//! (`__cer_wp_Image_data`), which at least points at generated plumbing
//! rather than silently compiling. `fill_from` is the ONE method the
//! rewriter routes (to the `__cer_fill_from_<field>` shim).
//!
//! Lives in the `#[ignore]`d `type_error/` group: E0599 rendering is
//! rustc's — toolchain-fragile, re-blessed after toolchain bumps.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct MethodCallProxyNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl MethodCallProxyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce E0599: no method `push` on `__cer_wp_Image_data`.
        self.image.data.push(1u8);
        Ok(())
    }
}

fn main() {}
