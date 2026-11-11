// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib node using `#[cerulion_node]` + `#[cerulion_node_impl]`
//! (the minimal macro-pair fixture).
//!
//! Verifies that the macro pair correctly generates cdylib FFI entry points
//! compatible with `DylibNodeEntry::load()`. Field types are the unit-struct
//! markers (`Vector3`); the impl macro rewrites every `self.<port>` access in
//! `tick` to dispatch through a per-tick `OutputProxy<'_, T>` /
//! `InputView<'_, T>` so direct field assignment lands straight in iceoryx2
//! SHM with no memcpy on the local hot path.
//!
//! # Marker import
//!
//! `#[cerulion_node]`'s declarative-mode emission uses the field type as a
//! type argument (`loan_proxy::<Vector3>()`, `try_view::<Vector3, _>(...)`,
//! `OutputProxy<'_, Vector3>`, `InputView<'_, Vector3>`). Type-argument
//! position counts as a use for `unused_imports`, so the marker import is
//! never flagged. Verified by the crate-level `#![deny(unused_imports)]`.

// Deny unused-imports at the crate level to prove the marker imports below
// (`Vector3`) are actually consumed by the macro emission (see the marker-
// import section in the module doc).
#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::atomic::{AtomicU64, Ordering};

static TICK_COUNT: AtomicU64 = AtomicU64::new(0);

#[cerulion_node]
struct MacroCounter {
    // Marker fields: the impl macro rewrites `self.velocity_in.<f>` to the
    // per-tick `__cer_velocity_in: &InputView<'_, Vector3>` and
    // `self.cmd_out.<f>` to `__cer_cmd_out: &mut OutputProxy<'_, Vector3>`.
    // The unit-struct fields themselves are never read directly by user code.
    #[input(trigger, depth = 1)]
    velocity_in: Vector3,

    #[output]
    cmd_out: Vector3,

    panic_on_shutdown: bool,
}

#[cerulion_node_impl]
impl MacroCounter {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        let phase = ctx.env_opt("CER_TEST_STABLE_PANIC_PHASE");
        assert_ne!(phase.as_deref(), Some("init"), "fixture init panic");
        self.panic_on_shutdown = phase.as_deref() == Some("shutdown");
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        assert!(!self.panic_on_shutdown, "fixture shutdown panic");
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        // Direct SHM-to-SHM copy of the three f64 fields. The impl macro
        // wraps the body in a nested `try_view` for `velocity_in` plus a
        // `loan_proxy::<Vector3>()` for `cmd_out`, so each line below
        // compiles to a single `mov` from one shared-memory region into
        // another with no intermediate snapshot or memcpy.
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);
        self.cmd_out.x = self.velocity_in.x;
        self.cmd_out.y = self.velocity_in.y;
        self.cmd_out.z = self.velocity_in.z;
        Ok(())
    }
}

/// Helper for tests to read tick count via FFI.
#[no_mangle]
pub extern "C" fn cerulion_test_get_tick_count() -> u64 {
    TICK_COUNT.load(Ordering::Relaxed)
}

#[cfg(test)]
mod context_compile_tests {
    #[test]
    fn stable_context_helpers_compile_with_the_author_migration() {
        let cases = trybuild::TestCases::new();
        cases.pass("tests/ui/annotated_context_helper.rs");
        cases.pass("tests/ui/data_only_helpers.rs");
        cases.pass("tests/ui/qualified_context_helper.rs");
        cases.compile_fail("tests/ui/free_context_helper.rs");
    }
}
