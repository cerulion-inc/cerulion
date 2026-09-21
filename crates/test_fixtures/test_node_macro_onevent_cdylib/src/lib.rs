// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib `#[on_event]` DISPATCH round-trip fixture
//! (inert-shipping instance #7 proof).
//!
//! Before this fixture, NO cdylib declared a single `#[on_event]` handler:
//! `cdylib_qos_ffi_test` only round-trips the QoS *metadata* across the
//! info-JSON FFI, and every `on_event_*` in-process suite loads its consumer as
//! an in-process macro node — never through `DylibNodeEntry`. So the fact that
//! the type-routed `#[on_event]` handler DISPATCH (emitted into the cdylib's
//! `__cer_zero_copy_tick`, guarded by the tick-Ok outcome) actually FIRES inside
//! a dynamically-loaded `.so`/`.dylib` had never been proven behaviorally.
//!
//! This fixture declares one handler for EACH of the four reactive event kinds
//! and is driven by `cdylib_on_event_dispatch_test.rs`:
//!
//! - `BackpressureEvent` — input `samp` (`sample(3)` read-gate) decimates.
//! - `ExpectWithinEvent` — trigger input `trig` (`expect_within_ms = 20`)
//!   starves when the upstream is slow.
//! - `LivelinessEvent` — input `samp` (same port as the Backpressure handler —
//!   the (port,kind) coexistence). Driven by the REAL runtime liveliness
//!   sweep (the graceful-disconnect path of `liveliness_sweep_iox2_test`):
//!   the test wires `samp` to an absolute external topic, attaches a raw
//!   publisher, and gracefully DROPS it — the sweep observes the 1→0
//!   publisher-count edge and mints a `Lost`. The handler counts ONLY `Lost`
//!   so the attach's `Alive` edge never inflates the counter (and a
//!   never-dropped publisher reads 0 — the quiet control).
//! - `PromiseWithinEvent` — output `out` (`promise_within_ms = 25`) breaks
//!   its publish promise when the node fires slowly.
//!
//! The liveliness handler sits on `samp` (NOT `trig`) deliberately: a real
//! disconnect needs a droppable external publisher, and dropping `trig`'s
//! publisher would stop the data-triggered node from ticking — the documented
//! dispatch limitation `liveliness_sweep_iox2_test::
//! data_trigger_lost_handler_silent_but_counter_fires` pins. With the handler
//! on the held non-trigger `samp`, `trig`'s in-graph producer keeps the node
//! ticking after the drop, so the Lost dispatches.
//!
//! # Firing policy: data-trigger (not period)
//!
//! The node is DATA-TRIGGERED on `trig`, so its publish cadence == its fire
//! cadence == `trig`'s arrival rate. That is the single design choice that lets
//! ONE fixture drive both watchdogs to both fire AND quiet from a single
//! declaration (a fixed `period_ms` cannot: promise needs publishes slower than
//! 25 ms, decimation needs reads faster than the sample window). A slow `trig`
//! producer makes `trig`'s expect window and `out`'s publish promise both miss;
//! a fast `trig` producer keeps both fresh. This mirrors the proven in-process
//! shapes exactly — `on_event_watchdog_test`'s trigger `ExpectWatchConsumer`
//! and its slow/fast `PromiseProducer` pair.
//!
//! # Observability: counters cross via the output message
//!
//! A cdylib's Rust state is opaque across the FFI, so the four handler counters
//! cannot be read by `with_state`-injected atomics the way the in-process suites
//! do. Instead each handler bumps a plain `u32` field and `tick()` publishes all
//! four into the fixed `Quaternion` output (x/y/z/w — a 4-slot fixed schema, one
//! per counter). A host-side in-process sink reads them back off `out`; the
//! counters crossing the SHM boundary ARE the proof the handlers ran INSIDE the
//! cdylib (no fake data — Principle #13).
//!
//! Dispatch runs on the tick-Ok path AFTER the body, so the value published in
//! tick N reflects handler increments through tick N-1 (a one-tick publish lag);
//! the driving test records the MAX per counter, which is monotone-safe.
//!
//! Both inputs are held across steps (`trig` as a drained trigger,
//! `samp` as a held non-trigger), so the tick body does not collapse on a
//! silent/decimated step once each input has delivered at least once.
//!
//! Needs `cargo build -p test_node_macro_onevent_cdylib`.

#![deny(unused_imports)]
// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::{Quaternion, Vector3};

#[cerulion_node]
#[derive(Default)]
struct OnEventNode {
    // TRIGGER input — drives firing (data-trigger) and carries the input-side
    // watchdog. A slow producer starves the 20 ms window (ExpectWithinEvent);
    // a fast one keeps it fresh.
    #[input(trigger, expect_within_ms = 20)]
    trig: Vector3,

    // NON-trigger, sample-gated input — a producer faster than the 3 ms gate
    // decimates, queuing the edge-triggered BackpressureEvent (mirrors
    // `on_event_test`'s `sample(N)` consumer). ALSO the port whose real
    // publisher connect/disconnect the LivelinessEvent handler watches — two
    // DIFFERENT kinds on the SAME input (the (port,kind) coexistence dispatch,
    // proven in-process by `on_event_multi_handler_test`). Held across steps
    // (the non-trigger hold), so the body keeps running after the external publisher drops.
    #[input(backpressure = sample(3))]
    samp: Vector3,

    // Output with a publish promise. The node publishes on every fire, so a
    // slow fire cadence (slow `trig`) breaks the 25 ms promise
    // (PromiseWithinEvent); a fast one keeps it.
    #[output(promise_within_ms = 25)]
    out: Quaternion,

    // Sink for the input reads (keeps `trig`/`samp` live and drives the body
    // read of `samp`); mirrors `last`/`last_seen` in the in-process suites.
    last: f64,

    // Handler counters. READ (published below), so `dead_code = deny` is
    // satisfied without a manual annotation.
    bp_count: u32,
    expect_count: u32,
    live_count: u32,
    promise_count: u32,
}

#[cerulion_node_impl]
impl OnEventNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Read BOTH inputs so the gates run: reading `trig` resets its expect
        // watchdog on a fresh arrival; reading `samp` drives the `sample(3)`
        // decimation (which queues the BackpressureEvent the handler drains).
        // The reads also keep the ports live for the macro's port rewrite.
        self.last = self.trig.x + self.samp.x;

        // Publish the four counters (accumulated through the PREVIOUS tick's
        // dispatch) into the fixed Quaternion output. Crossing SHM to a
        // host-side sink is the observable proof the handlers ran in-cdylib.
        self.out.x = self.bp_count as f64;
        self.out.y = self.expect_count as f64;
        self.out.z = self.live_count as f64;
        self.out.w = self.promise_count as f64;
        Ok(())
    }

    // --- One handler per reactive event kind. Each is invoked by the dispatch
    //     the macro emits into `__cer_zero_copy_tick` on the tick-Ok path. ---

    /// BackpressureEvent — routed to `take_backpressure_event` by the param type.
    #[on_event(input = "samp")]
    fn on_backpressure(&mut self, _event: BackpressureEvent) {
        self.bp_count += 1;
    }

    /// ExpectWithinEvent — routed to `take_expect_within_event` by the param type.
    #[on_event(input = "trig")]
    fn on_expect(&mut self, _event: ExpectWithinEvent) {
        self.expect_count += 1;
    }

    /// LivelinessEvent — routed to `take_liveliness_event` by the param type.
    /// Same input as `on_backpressure` (different kind → coexistence). Counts
    /// ONLY `Lost` transitions: the real sweep also mints an `Alive` edge when
    /// the test's external publisher first attaches, and filtering it out keeps
    /// the counter a pure disconnect count (attach-then-hold reads 0 — the
    /// driving test's quiet control).
    #[on_event(input = "samp")]
    fn on_liveliness(&mut self, event: LivelinessEvent) {
        if event.state == LivelinessState::Lost {
            self.live_count += 1;
        }
    }

    /// PromiseWithinEvent — routed to `take_promise_within_event` by the param type.
    #[on_event(output = "out")]
    fn on_promise(&mut self, _event: PromiseWithinEvent) {
        self.promise_count += 1;
    }
}
