#![allow(unexpected_cfgs)]
//! A DENY symbol hidden inside a MACRO argument
//! (`format!("{:?}", Instant::now())`) must still fail to compile. The
//! `syn::visit::Visit` walk treats macro token streams as opaque, so the
//! `DeterminismLintVisitor::visit_macro` override re-parses the macro args
//! and inspects them — without it, this most-common "log a banned timestamp"
//! form would slip through clean.
//!
//! Uses `format!` (no extra dep) rather than `tracing::info!` to keep the
//! fixture dependency-free; the visitor path is identical (any macro whose
//! tokens parse as a comma-separated expr list is descended into).

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::time::Instant;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct MacroArgTimeNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl MacroArgTimeNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Banned: `Instant::now()` buried in a macro argument.
        let _s = format!("{:?}", Instant::now());
        self.image.height = 1;
        Ok(())
    }
}

fn main() {}
