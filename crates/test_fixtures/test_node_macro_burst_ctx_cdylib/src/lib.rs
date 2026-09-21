// SPDX-License-Identifier: AGPL-3.0-only
//! Serve-many: a DATA-TRIGGER cdylib node carrying a plain
//! (NON-trigger) `#[input]` context: the cross-step-hold motivating shape, and the
//! production surface of the reuse rule: the step-frozen
//! slot serves every fire of a burst.
//!
//! Loaded by `tests/fifo_burst_within_step_iox2_test.rs`.
//!
//! # Why a NEW fixture
//!
//! No existing macro cdylib fixture is a DATA-TRIGGER node with a plain
//! non-trigger `#[input]`, which is precisely the shape the decision addresses:
//!
//! - `test_node_macro_data_trigger_cdylib` is trigger-ONLY (no context input),
//!   so it has no frozen non-trigger slot to re-serve and its burst was never
//!   capped.
//! - `test_node_macro_period_input_cdylib` has the non-trigger `#[input]` but a
//!   `period_ms` policy, so it never takes the `Data` burst path at all.
//! - `test_node_macro_sync_nontrigger_cdylib` is Sync, which aligns rather than
//!   bursts.
//!
//! # The declaration ORDER is load-bearing
//!
//! `ctx_in` is declared BEFORE `inp` on purpose. `#[cerulion_node_impl]` nests
//! one `try_view` per input in DECLARATION order, so a context input that has
//! not delivered short-circuits the whole chain (the pre-first-delivery
//! WAIT) and the trigger's own read never runs. That is exactly the collapse the
//! serve-once defect reproduced on EVERY fire after the first, and it is what
//! this fixture uses to tell correct behavior from the defect: correct behavior
//! re-reads the frozen `ctx_in` on fires 2..k and reaches `inp`; the defect
//! collapses them.
//!
//! # What crosses the FFI here
//!
//! Two additive export pairs, and this fixture needs BOTH:
//!
//! - `cerulion_node_{set_,}snapshot_inputs` — the per-step freeze of
//!   `ctx_in`, i.e. the slot under test.
//! - `cerulion_node_refill_trigger_input` — the between-fires refill
//!   of `inp`, i.e. the burst itself.
//!
//! `out` carries BOTH values (`x` = the trigger frame, `y` = the context read),
//! so a downstream sink recording `(x, y)` pairs is a complete oracle for what
//! each fire observed — the counters-via-output pattern, since a cdylib's own
//! state is unreachable from the host.

#![deny(unused_imports)]
// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
struct BurstCtxNode {
    /// Plain (non-trigger) latest-value CONTEXT, declared FIRST — see the
    /// module docs on why the order is load-bearing.
    #[input]
    ctx_in: Vector3,
    /// The trigger. Deeper than any burst these tests queue, so an eviction
    /// can never be mistaken for a firing cap.
    #[input(trigger, depth = 32)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl BurstCtxNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Forward BOTH reads so the downstream sink observes, per fire, which
        // trigger frame was consumed AND which context value was served.
        self.out.x = self.inp.x;
        self.out.y = self.ctx_in.x;
        Ok(())
    }
}
