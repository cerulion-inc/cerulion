// SPDX-License-Identifier: AGPL-3.0-only
//! Crate-level, TEST-ONLY serialization for the process-global
//! blueprint statics.
//!
//! A file-local mutex — this crate's usual tool (see `SKELETON_STATICS_LOCK` in
//! `tests/sink_dispatch_test.rs`) — cannot serve here, because the two groups of
//! tests that race live in DIFFERENT modules of the same lib test binary. Hence
//! a crate-level lock, `#[cfg(test)]` so it cannot reach a shipping build.

use std::sync::{Mutex, MutexGuard};

/// Serializes every test that reaches the process-global `BLUEPRINT_SENT`,
/// whether it does so **directly** or **through production code**:
///
/// * DIRECTLY — `blueprint::tests::send_once_guard_fires_once_then_rearms`,
///   which calls `rearm_blueprint()` and then asserts an EXACT sequence of
///   `swap` return values. Any concurrent swap corrupts that oracle.
/// * INDIRECTLY — `worker.rs`'s eight worker-spawning tests. A worker's boot
///   runs `ensure_setup`, which swaps the same flag on the PRODUCTION path
///   (`blueprint.rs:159`), from the spawned thread.
///
/// **Why this exists.** Before it existed nothing held those two groups apart
/// except libtest's dispatch ORDER, while `blueprint.rs` carried a comment
/// asserting there was "no intra-binary race". Ordering is not a mechanism — it
/// is an accident that survives exactly until someone renames a test, and the
/// viz lane runs at default parallelism by design.
///
/// **No deadlock is possible.** Production code never takes this lock:
/// `ensure_setup` is untouched. It serializes TEST BODIES only, so a worker
/// thread cannot block on it.
///
/// **DROP ORDER IS LOAD-BEARING.** Take the guard as the FIRST statement of the
/// test. Locals drop in reverse declaration order, so a guard bound first is
/// released LAST — still held while `VizLogWorker`'s `Drop` waits on the worker
/// thread that calls `ensure_setup`. Bind it after the worker (or as one
/// element of a returned tuple, which drops front-to-back) and the guard is
/// released while that thread is still running, which reopens the race silently.
///
/// **Scope of that guarantee, stated exactly.** `VizLogWorker::drop` does NOT
/// join unconditionally: it waits up to `SHUTDOWN_JOIN_TIMEOUT` and then
/// DETACHES a worker still wedged in `rec.log` (the never-block guarantee
/// extended to teardown — `worker.rs`'s `Drop`). So the lock covers the worker
/// thread for the bounded window only. A test that both WEDGES its worker past
/// that timeout and asserts on `BLUEPRINT_SENT` would still race — no such test
/// exists (the wedging tests assert on queue/latch state, not the blueprint
/// statics), and the boot `ensure_setup` runs long before any wedge, but a new
/// one must not assume the join is unconditional.
static BLUEPRINT_STATICS_LOCK: Mutex<()> = Mutex::new(());

/// Acquire [`BLUEPRINT_STATICS_LOCK`]. See its docs for the drop-order rule.
pub(crate) fn blueprint_statics_guard() -> MutexGuard<'static, ()> {
    // Poisoning is TOLERATED: one panicking test must not cascade into every
    // sibling that shares the lock. The state it guards is re-established by
    // each test (`rearm_blueprint()` / a fresh worker), so a poisoned guard
    // carries no stale invariant. Mirrors `vizd_e2e_test.rs`'s convention.
    BLUEPRINT_STATICS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}
