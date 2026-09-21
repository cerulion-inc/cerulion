//! The node-level `deadline_ms` trigger was removed
//! (decomposed into a `Data` trigger + the per-input `expect_within_ms`
//! QoS watchdog — see `#[input(trigger, expect_within_ms = N)]`).
//! Declaring `deadline_ms` must now fail as an unknown attribute,
//! pinning the removal so a regression that re-adds the parse arm is
//! caught at compile time rather than silently resurrecting the trigger.
use cerulion_core::prelude::*;

#[cerulion_node(deadline_ms = 100)]
struct DeadlineNode;

fn main() {}
