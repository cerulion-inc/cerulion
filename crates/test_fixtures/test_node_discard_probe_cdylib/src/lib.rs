// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-tracing e2e pin fixture: a macro cdylib node that exercises the
//! cdylib-local stderr tracing subscriber (installed at `cerulion_node_init`).
//!
//! # Two consumers
//!
//! 1. `cerulion_core/tests/cdylib_tracing_stopgap_test.rs` — the
//!    original consumer, described below.
//! 2. `cerulion_core/tests/cdylib_iox2_log_level_test.rs` — reuses
//!    this fixture as the MACRO-cdylib probe for the OTHER per-linked-copy
//!    static: each tick, gated on the `CER_CDYLIB_IOX2_PROBE` env var, it prints
//!    `CDYLIB_IOX2_LEVEL=<n>` — the `iceoryx2-log` level of THIS
//!    cdylib's own statically-linked copy, which the host's `set_log_level`
//!    cannot reach. The gate keeps it INERT for every test (which never
//!    sets that var), so the two consumers do not interfere.
//!
//! Consumed by `cerulion_core/tests/cdylib_tracing_stopgap_test.rs` (a
//! subprocess-based regression pin). Every tick emits THREE USER-side probe
//! lines at distinct levels — `tracing::warn!` ("discard probe tick fired"),
//! `tracing::info!` ("discard probe info probe"), and `tracing::debug!`
//! ("discard probe debug probe") — pinning that node-side user logs become
//! host-visible on stderr AND letting the test pin the effective filter
//! exactly (warn/info visible under the default "info", debug not). Then,
//! keyed off the `CER_DISCARD_MODE` env var (the subprocess test controls the
//! child env):
//!
//! - default (unset / any non-`healthy` value): writes ONLY the `data` variable
//!   field, leaving `encoding` + `header` unwritten, so `OutputProxy::drop`
//!   fires cerulion_core's loud discard error every tick ("OutputProxy dropped
//!   without writing all declared variable fields; skipping publish"). This is
//!   the black-hole regression the stopgap fixes — with no subscriber that
//!   error dispatches to a no-op.
//! - `healthy`: writes all three variable fields → publishes normally, no
//!   discard error (the control that proves the error is tied to the
//!   unwritten-field condition, not to load/init).
//!
//! Needs `cargo build -p test_node_discard_probe_cdylib`.

#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

/// Read per tick (test fixture — the subprocess test controls the child env;
/// mirrors the `CER_FAIL_MODE` env-switch precedent in `test_node_failing_cdylib`).
fn discard_mode_is_healthy() -> bool {
    std::env::var("CER_DISCARD_MODE").as_deref() == Ok("healthy")
}

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DiscardProbe {
    #[output]
    image_out: Image,
    count: u32,
}

#[cerulion_node_impl]
impl DiscardProbe {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        // Log-level probe (env-gated ⇒ INERT for every test): report the
        // iceoryx2 log level of THIS CDYLIB's own `iceoryx2-log` static. That
        // static is distinct from the host binary's, so the host's
        // `set_log_level` does nothing for it — which is precisely how
        // `IOX2_LOG_LEVEL` became inert on the robot while a cdylib node's
        // publisher flooded iceoryx2 warnings. Consumed by
        // `cerulion_core/tests/cdylib_iox2_log_level_test.rs`.
        // P12 exception: a MACHINE-READ MARKER, not a log — see the comment
        // above; the parent test process parses it off this cdylib's stderr.
        #[allow(clippy::print_stderr)]
        if std::env::var_os("CER_CDYLIB_IOX2_PROBE").is_some() {
            eprintln!(
                "CDYLIB_IOX2_LEVEL={}",
                ::cerulion_core::iceoryx_logger::current_iox2_log_level()
            );
        }
        // USER-side probes — pin that node tick logs reach the process stderr
        // via the cdylib-local subscriber. Three distinct levels so
        // the test can pin the effective filter EXACTLY (warn/info visible under
        // the default "info", debug not).
        tracing::warn!(tick = self.count, "discard probe tick fired");
        tracing::info!(tick = self.count, "discard probe info probe");
        tracing::debug!(tick = self.count, "discard probe debug probe");

        // Touch a fixed field so the OutputProxy is loaned (its Drop then runs
        // the variable-field-written validation).
        self.image_out.height = self.count;

        // Local binding (mirrors the overflow fixture) so the slice outlives the
        // set_<field> rewrite.
        let payload = [1u8, 2, 3];
        if discard_mode_is_healthy() {
            // Write ALL variable fields → OutputProxy publishes cleanly,
            // no discard error.
            self.image_out.header = &[][..];
            self.image_out.encoding = "rgb8";
            self.image_out.data = &payload[..];
        } else {
            // Write ONLY `data`, leaving `encoding` + `header` unwritten → the
            // OutputProxy Drop fires the discard error and skips the publish.
            self.image_out.data = &payload[..];
        }
        Ok(())
    }
}
