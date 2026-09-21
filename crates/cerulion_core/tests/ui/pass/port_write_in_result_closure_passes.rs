#![allow(unexpected_cfgs)]
//! Both documented remedies for the
//! port-write-in-closure limitation compile (numbering matches
//! USER_API's "Helper methods…" section):
//!
//! 1. (USER_API "Remedy 1") the hoisted form — compute inside the
//!    closure, write the port outside it;
//! 2. (USER_API "Remedy 2") a `Result`-returning closure under a
//!    fallible combinator (`try_for_each`) — the injected `?` on the
//!    rewritten assignment is legal there, so combinator-based port
//!    writes remain expressible.
//!
//! Companion to the `type_error/port_write_in_unit_closure` fixture
//! (the `()`-closure E0277 backstop).

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ResultClosurePortWriteNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ResultClosurePortWriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Remedy 2: fallible combinator — the closure returns Result,
        // so the rewritten `__cer_assign_x(..)?` is legal inside it.
        (0..3u32).try_for_each(|i| -> Result<(), NodeError> {
            self.out.x = f64::from(i);
            Ok(())
        })?;
        // Remedy 1: hoist — compute in the closure, write outside.
        let last = (0..3u32).map(f64::from).last().unwrap_or(0.0);
        self.out.y = last;
        self.out.z = 0.0;
        Ok(())
    }
}

fn main() {}
