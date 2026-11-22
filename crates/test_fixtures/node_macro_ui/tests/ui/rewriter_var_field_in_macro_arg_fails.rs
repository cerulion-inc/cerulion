#![allow(unexpected_cfgs)]
//! A port-field access inside a macro argument must surface
//! the friendly hoist-it compile_error, NOT a generic rustc cascade at the
//! proxy/marker.
//!
//! `VisitMut::visit_expr_mut` doesn't descend into `Expr::Macro` (macro
//! tokens are opaque to syn), so `println!("{:?}", self.image.data)` would
//! silently bypass the rewriter. The guard re-parses macro tokens as a
//! comma-separated expression list; if any contains a `self.<port>.<field>`
//! shape on a declared port, the whole macro invocation is replaced with the
//! compile_error that prescribes the hoist (`let v = …; println!(…, v);`).
//!
//! This is the OUTPUT-port variant; the INPUT-port read (its
//! motivating case) is pinned by `port_field_read_in_macro_arg_fails`.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct VarFieldInMacroArgNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl VarFieldInMacroArgNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce the hoist-it compile_error naming the macro
        // (`println`) and the field (`self.image.data`).
        println!("data is {:?}", self.image.data);
        Ok(())
    }
}

fn main() {}
