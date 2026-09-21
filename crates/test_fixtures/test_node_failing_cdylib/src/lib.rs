// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib whose `tick` returns a known `NodeError`, panics, or
//! returns Err from `init` depending on the `CER_FAIL_MODE` env var.
//!
//! Used by:
//! - the runtime_error_2 integration test (default `tick_logic` mode)
//! - the tests for FFI codes 2 (panic) and the
//!   init-fail path.
//! - `chunk_c_ffi_codes_3_4_test.rs` for FFI codes 3 (mutex
//!   poisoned via `tick_panic`) and 4 (handle not found via raw
//!   libloading — no fixture mode needed; the test simply calls
//!   `cerulion_node_tick` with a handle that was never `init`'d).
//!
//! Modes (env var `CER_FAIL_MODE`):
//! - `tick_logic` (default): tick returns `NodeError::Logic("...")` →
//!   FFI code 1, rich `LAST_ERROR` populated.
//! - `tick_panic`: tick `panic!()`s → first call returns FFI code 2
//!   (caught by `catch_unwind`) and `LAST_ERROR = "panic caught by
//!   catch_unwind"`. The macro's tick wrapper holds the NODES mutex
//!   guard across the user `tick()` body (see
//!   `cerulion_macros/src/codegen.rs` `cerulion_node_tick`), so the
//!   panic-induced unwind drops the guard mid-flight and poisons
//!   NODES. Every subsequent FFI call on this cdylib then returns
//!   FFI code 3 with `LAST_ERROR = "<entrypoint>: NODES mutex
//!   poisoned"`. This cascade is the path exercised by that test.
//! - `init_fail`: init returns Err → `cerulion_node_init` returns 0
//!   handle, `LAST_ERROR` set.

// P12 (the logging convention in AGENTS.md): library code never prints. It
// logs through `tracing`. Scoped `not(test)` so unit tests keep printing
// diagnostics, and applied at the crate root rather than in `[workspace.lints]`
// because that table cannot distinguish a lib target from a test binary.
// Pinned by `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// Mode read once on first tick (or first init for init_fail).
fn fail_mode() -> String {
    std::env::var("CER_FAIL_MODE").unwrap_or_else(|_| "tick_logic".to_string())
}

#[cerulion_node(period_ms = 1)]
struct FailingNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl FailingNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        if fail_mode() == "init_fail" {
            return Err(NodeError::Logic(
                "simulated init failure (init_fail mode)".to_string(),
            ));
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        // Touch the output so the macro's port-rewrite doesn't elide
        // anything in any branch.
        self.out.x = 0.0;

        match fail_mode().as_str() {
            "tick_panic" => {
                panic!("simulated panic (tick_panic mode)");
            }
            "init_fail" => Ok(()),
            _ /* tick_logic / default */ => Err(NodeError::Logic(
                "simulated failure for chunk C runtime_error_2 test".to_string(),
            )),
        }
    }
}
