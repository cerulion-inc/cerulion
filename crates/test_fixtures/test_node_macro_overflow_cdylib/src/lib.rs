// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib fixture exercising the in-tick spill
//! path on a variable schema (`sensor_msgs::Image`).
//!
//! The companion test (`macro_cdylib_overflow_test.rs`) loads this
//! fixture via `DylibNodeEntry`, wires it into a `NodeContext` with a
//! warm publisher + subscriber, and verifies that:
//!
//! 1. After warm-up at small payloads (≤ 1 KiB) the adaptive loan
//!    shrinks toward the steady-state working set.
//! 2. A subsequent tick that spikes to 12 KiB triggers the spill path
//!    *inside* the cdylib's `tick` body (codegen-emitted
//!    `<Name>Shm::spill_to_overflow` runs across the FFI boundary).
//! 3. The drop-time re-loan from the cdylib's `OutputProxy` lands the
//!    full 12 KiB frame on the subscriber — bit-for-bit.
//!
//! Why a dedicated cdylib fixture: the in-process
//! e2e tests (`overflow_redirect_e2e_test.rs`) prove the spill +
//! re-loan + history pipeline in plain Rust. The cdylib path adds the
//! FFI boundary — the `OutputProxy`'s `T::Writer<'_>` (carrying
//! `Arc<str>`, `Box<[u64]>`, and `MaxPayloadCapacity`)
//! is constructed by codegen that runs in the
//! cdylib AND in the host (`cerulion_core`). A struct-layout drift
//! between the two compile units would manifest as SIGSEGV at proxy
//! Drop. This
//! fixture is the canary.

#![deny(unused_imports)]
// Principle #12 (structured logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::sync::atomic::{AtomicU64, Ordering};

static TICK_COUNT: AtomicU64 = AtomicU64::new(0);
static DATA_BYTES_WRITTEN: AtomicU64 = AtomicU64::new(0);

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ImageOverflowSource {
    #[output]
    image_out: Image,
}

#[cerulion_node_impl]
impl ImageOverflowSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The host test drives behaviour by reading TICK_COUNT and arranging
        // publisher warm-up via separate publish ticks. Each call here
        // writes the same growth profile so the test can pin
        // round-trip equality on the FINAL 12 KiB tick — the
        // first `N` ticks warm the sliding window with 1 KiB
        // payloads; tick `N+1` spikes to 12 KiB to force a spill.
        let tick = TICK_COUNT.fetch_add(1, Ordering::Relaxed);

        // Fixed-section fields. The macro rewriter routes
        // `self.image_out.<f>` to the loaned `OutputProxy`.
        self.image_out.height = tick as u32;
        self.image_out.width = 1;
        self.image_out.step = 1;
        self.image_out.is_bigendian = 0;

        // Variable fields. The schema-blind rewriter routes these through
        // the uniform `__cer_assign_<f>` shims (which delegate to
        // `set_<f>(...)` / `set_header_bytes(...)`).
        self.image_out.header = &[][..];
        self.image_out.encoding = "g";

        // Spike on tick 16+ (host warms with 16 small ticks first).
        let payload_bytes: usize = if tick >= 16 { 12 * 1024 } else { 1024 };
        let payload = vec![(tick & 0xff) as u8; payload_bytes];
        self.image_out.data = &payload[..];
        DATA_BYTES_WRITTEN.store(payload_bytes as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Helper for tests: read the latest tick's payload size (so the test
/// can assert it matches the SHM-side `data().len()` byte-for-byte).
#[no_mangle]
pub extern "C" fn cerulion_test_get_data_bytes_written() -> u64 {
    DATA_BYTES_WRITTEN.load(Ordering::Relaxed)
}
