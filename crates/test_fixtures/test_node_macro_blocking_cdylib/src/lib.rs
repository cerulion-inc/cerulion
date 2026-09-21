// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib tier-2 `ExternalSource::Blocking` → doorbell-pipe
//! collapse fixture.
//!
//! An `#[cerulion_node(external)]` ingress whose `external_source()` returns
//! `ExternalSource::Blocking(closure)`. A closure cannot cross the C ABI, so the
//! macro-emitted `cerulion_node_external_source` export materializes it into a
//! `pipe(2)` + a detached helper thread (via
//! `cerulion_core::graph::node::spawn_cdylib_blocking_doorbell`) and returns the
//! READ end as kind 2 (`EXTERNAL_SOURCE_KIND_DOORBELL_FD`). The host binds that
//! read end as a drained `DoorbellFdSource` and fires the node when the helper
//! rings. Exercised end-to-end by
//! `cerulion_core/tests/cdylib_blocking_doorbell_test.rs` — the runtime coverage
//! the compiled-but-unexercised kind-2 path was missing.
//!
//! Modes:
//!
//! `CER_EXT_MODE` (read on the COLLECT thread inside `external_source()`) selects
//! which [`ExternalSource`] tier this node reports, exercising the host's kind-1
//! / kind-(-1) resolver arms (review cases T2/T3); kind-2 (doorbell) is the
//! default:
//! - `device_fd`: return `ExternalSource::Fd(<read end of a real pipe>)` — a
//!   poll-only tier-1 device fd (host kind 1). The write end is kept open for the
//!   process life so the read end stays valid (no fake data, Principle #13).
//! - `error_panic`: `external_source()` PANICS on the collect thread → the
//!   macro-emitted `cerulion_node_external_source` export's `catch_unwind`
//!   returns `EXTERNAL_SOURCE_KIND_ERROR` (-1) → the host reports `None`.
//!   (Distinct from `blocking_panic`, which panics on the HELPER thread.)
//! - unset / any other value: tier-2 `ExternalSource::Blocking` → doorbell
//!   collapse (kind 2).
//!
//! `CER_FAIL_MODE` (read INSIDE the Blocking closure so a panic lands on the
//! HELPER thread — `spawn_cdylib_blocking_doorbell`'s `catch_unwind` wraps only
//! the closure call, not `external_source()`; follows the
//! `test_node_failing_cdylib` env-switch precedent), applies only to the default
//! Blocking path:
//! - default (unset / any other value): sleep ~1ms then return `true` — a
//!   continuous, level-style ring so the host fires the node on every live step.
//! - `blocking_panic`: the closure panics on its FIRST call → caught on the
//!   helper thread → the source is poisoned LOUDLY → the helper closes its write
//!   end and exits → the node never fires (containment, not a process abort).

#![deny(unused_imports)]
// Principle 12 (logging): library code never prints. It logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use std::os::fd::IntoRawFd;

#[cerulion_node(external)]
struct BlockingNode {
    #[output]
    out: Vector3,
    /// Incrementing publish payload — the downstream delivery oracle (a fresh
    /// value each fire proves the node actually ticked + published, not a stale
    /// frame). The macro auto-derives `Default`, so this starts at `0.0`.
    next_val: f64,
}

#[cerulion_node_impl]
impl BlockingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.next_val += 1.0;
        // Fixed primitive field on Vector3 → direct write into the loaned SHM slot.
        self.out.x = self.next_val;
        Ok(())
    }

    // The node's external-ingress source. `CER_EXT_MODE` picks
    // the tier (see the module docs); the default is the tier-2 blocking-SDK
    // source (the runtime drives its closure on a helper thread — each `true`
    // rings the node's doorbell; N rings before a step coalesce to one fire).
    fn external_source(&mut self) -> ExternalSource {
        match std::env::var("CER_EXT_MODE").as_deref() {
            // Kind -1: panic on the COLLECT thread (inside external_source) → the
            // macro-emitted export's catch_unwind returns EXTERNAL_SOURCE_KIND_ERROR
            // → the host reports None (node stays inert).
            Ok("error_panic") => {
                panic!("simulated external_source() panic (error arm)");
            }
            // Kind 1: a REAL poll-only device fd (a live pipe's read end). We
            // `mem::forget` the WRITE end (below) rather than dropping it: a
            // dropped/closed writer would leave the read end VALID but permanently
            // EOF/POLLHUP-readable, so the level-triggered WaitSet would fire the
            // node every step. Forgetting it keeps the device QUIESCENT — no bytes,
            // no EOF — a genuine idle pollable fd (no fake data, Principle #13; a
            // driver owns its device for the run).
            Ok("device_fd") => {
                let (reader, writer) = std::io::pipe().expect("create device-fd test pipe");
                std::mem::forget(writer);
                ExternalSource::Fd(reader.into_raw_fd())
            }
            // Kind 2 (default / blocking_panic): tier-2 Blocking → doorbell collapse.
            _ => ExternalSource::Blocking(Box::new(|| {
                // Read `CER_FAIL_MODE` INSIDE the closure so a `blocking_panic`
                // unwind lands on the HELPER thread (where the doorbell's
                // catch_unwind contains exactly this call), not on the collect
                // thread's external_source().
                if std::env::var("CER_FAIL_MODE").as_deref() == Ok("blocking_panic") {
                    panic!("simulated Blocking-source panic (doorbell test)");
                }
                // Bounded wait: keeps the helper responsive to shutdown (host
                // closes the read end → the helper's next write(2) fails → it
                // exits) and rings continuously so the host fires each live step.
                std::thread::sleep(std::time::Duration::from_millis(1));
                true
            })),
        }
    }
}
