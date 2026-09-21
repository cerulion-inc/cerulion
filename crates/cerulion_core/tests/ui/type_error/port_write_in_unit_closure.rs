#![allow(unexpected_cfgs)]
//! A port-field write inside a `()`-returning closure
//! (`for_each`, `map`, …) fails with rustc's E0277 — the schema-blind
//! rewrite appends `?` to EVERY port assignment (fixed fields included,
//! which were infallible Deref-assigns before that rewrite), and a closure whose
//! return type is dictated by the combinator cannot use `?`. The macro's
//! unit-return helper diagnostic inspects METHOD signatures only — a
//! closure's fallibility is not syntactically decidable (annotated
//! `-> Result` closures are legal and must keep compiling; see the
//! `port_write_in_result_closure_passes` pass fixture), so rustc's E0277
//! is the documented backstop here. USER_API's "Helper methods…" section
//! names both remedies (hoist, or `try_for_each` with a `Result`
//! closure).
//!
//! Lives in the `#[ignore]`d `type_error/` group: the E0277 rendering is
//! rustc's — toolchain-fragile, re-blessed after toolchain bumps.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct UnitClosurePortWriteNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl UnitClosurePortWriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Should produce E0277: the rewritten
        // `__cer_out.__cer_assign_x(&(f64::from(i)))?` sits in a
        // `()`-returning `for_each` closure.
        (0..3u32).for_each(|i| {
            self.out.x = f64::from(i);
        });
        Ok(())
    }
}

fn main() {}
