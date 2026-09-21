// SPDX-License-Identifier: AGPL-3.0-only
//! The fixture whose tick NEVER RETURNS — the one condition the
//! wedge alarm exists to report, and the one no other fixture in the tree can
//! produce.
//!
//! `tick_within_ms` times only ticks that come back, so a hung tick is invisible
//! to every counter in the system; a fixture that merely SLEEPS would be a race
//! against a wall (the class, where a nominal 150 ms is charged as
//! 1100–1696 ms under macOS background QoS) and would eventually return anyway,
//! clearing the very regime under test. This one parks FOREVER on a condvar
//! nothing ever notifies, so it is still inside its first tick however slow — or
//! fast — the machine is.
//!
//! Mode is an ENV SWITCH, the `test_node_failing_cdylib` `CER_FAIL_MODE`
//! pattern, so ONE fixture serves both the positive arm and its control:
//!
//! - `CER_HANG_MODE=1` — the FIRST tick parks forever. Later ticks never happen
//!   (the step thread is inside the first one), which is the point.
//! - anything else (including unset) — an ordinary `period_ms` node.
//!
//! The env is read ONCE, at the first tick, through the node's own frozen
//! `NodeContext` snapshot being irrelevant here: this is a fixture, and reading
//! the process env directly is what keeps it a ~20-line file.

#![deny(unused_imports)]
// P12 (the logging convention in AGENTS.md): library code never prints. It
// logs through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in `[workspace.lints]`
// because that table cannot distinguish a lib target from a test binary.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Park forever. A `Condvar` nobody notifies rather than a `sleep` loop: the
/// thread genuinely blocks, so the fixture costs no CPU while the supervisor
/// accumulates its dwell, and it can never wake up and clear the regime.
fn park_forever() -> ! {
    let pair = (std::sync::Mutex::new(()), std::sync::Condvar::new());
    let mut guard = pair.0.lock().expect("fresh mutex");
    loop {
        guard = pair.1.wait(guard).expect("fresh condvar");
    }
}

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct HangNode {
    #[output]
    cmd: Vector3,
    fired: u32,
}

#[cerulion_node_impl]
impl HangNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fired += 1;
        // Write the output BEFORE parking, so the healthy mode publishes a
        // complete frame and the hung mode's LAST complete fire is observable
        // downstream — an operator watching the topic sees it stop, which is the
        // symptom the alarm explains.
        self.cmd.x = f64::from(self.fired);
        if std::env::var("CER_HANG_MODE").as_deref() == Ok("1") {
            park_forever();
        }
        Ok(())
    }
}
