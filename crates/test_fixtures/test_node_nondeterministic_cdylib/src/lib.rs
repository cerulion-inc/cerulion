// SPDX-License-Identifier: AGPL-3.0-only
//! E-2: the DELIBERATELY nondeterministic node — the moat-credibility
//! fixture. Its fire SCHEDULE is deterministic (a plain `period_ms` trigger,
//! one fire per step), but the payload it publishes each tick embeds a value
//! that is FRESHLY MINTED per process from two independent non-replayable
//! sources folded into ONE field:
//!
//!   * `SystemTime::now()` nanoseconds — wall-clock, advances between the
//!     recording run and the replay run (the `ctx` clock is the sanctioned
//!     alternative — see `docs/replay_determinism_footguns.md`, G1);
//!   * a `RandomState`-seeded hash — the process-random ASLR/RNG seed the
//!     standard library mints per `RandomState::new()` (G2, unseeded RNG).
//!
//! One fixture demonstrates the whole nondeterminism class (the decision: one
//! fixture + the footguns doc, not one fixture per footgun). Recording a run of
//! this node then replaying it ALWAYS diverges: the recorded payload carries
//! run-A's minted value, the re-executed candidate produces run-B's — different
//! bytes on an identical fire schedule → a `ByteMismatch` data violation
//! (exit 1), never a structural trace divergence (exit 6). The verdict is
//! DETERMINISTIC (always "diverges") even though the OUTPUT is not: the two
//! runs collide only if two independently-seeded 64-bit folds land on the same
//! bits AND the wall clock did not advance — a ~1-in-2^64 event that the
//! monotonically-advancing time term makes effectively impossible.
//!
//! Used by:
//!   * the CLI production-path pin `subprocess_replay_of_nondeterministic_node_exits_1`
//!     (`cerulion_cli/tests/replay_cli_test.rs`) — loaded as `libticker` via
//!     `DylibNodeEntry`, recorded then replayed through the real binary;
//!   * the engine-suite flagship uses an inline in-process twin of this node
//!     (`nondeterministic_replay_is_a_byte_mismatch_not_a_trace_divergence`).

#![deny(unused_imports)]
// P12 (the repo's logging policy): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::hash::{BuildHasher, Hasher};
use std::time::{SystemTime, UNIX_EPOCH};

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

// `allow_non_deterministic` opts OUT of the compile-time lint that
// otherwise REFUSES `SystemTime::now()` (and other non-replayable reads) inside
// a node body — the framework catches this footgun class at build time; here we
// deliberately embrace it to demonstrate what replay does when a node ignores
// the guard. See `docs/replay_determinism_footguns.md`.
#[cerulion_node(period_ms = 50, allow_non_deterministic)]
#[derive(Default)]
struct NondetNode {
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl NondetNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Fold two independent non-replayable sources into one 64-bit value:
        // the advancing wall clock AND the process-random RandomState seed.
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);
        // `RandomState::new()` seeds from the process RNG; hashing a fixed key
        // through it yields a per-process-random 64-bit value.
        let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
        hasher.write_u64(0xC0FF_EE00_1351_E2E2);
        let rnd = hasher.finish();
        // `from_bits` may land on a NaN — irrelevant: replay diffs the raw wire
        // BYTES, not float values, so any distinct bit pattern is a divergence.
        self.cmd.x = f64::from_bits(nanos ^ rnd);
        Ok(())
    }
}
