// SPDX-License-Identifier: AGPL-3.0-only
//! A MACRO cdylib whose state encoder and decoder can be made to
//! **panic**, so the generated capture/restore exports can be driven through
//! the one path that used to brick the whole library.
//!
//! # Why this fixture exists
//!
//! The generated exports run `cer_capture` / `cer_restore` while holding this
//! cdylib's `NODES` `MutexGuard`. A panic escaping that call unwinds THROUGH
//! the live guard, which **poisons `NODES`** — and `NODES` is per-cdylib, not
//! per-node, so from that moment every `tick`, `capture`, `restore` and
//! `shutdown` of every node loaded from this library fails with "NODES mutex
//! poisoned". A capture panic escaping the guard kills the node silently, for
//! the life of the process, and misattributes the cause. The inline carrier
//! wraps its own capture precisely so the node mutex is not poisoned.
//!
//! No existing fixture can reach it. The macro fixtures next door capture
//! `u64`/`String`/`Vec`/`HashMap`, whose framework impls do not panic, and the
//! raw-FFI `test_node_state_liar_cdylib` never calls `CerulionState` at all —
//! it hand-rolls its bytes, so its `NODES` is untouched by a user encoder. The
//! panic therefore has to come from a field whose `CerulionState` impl is
//! HAND-WRITTEN, which is exactly the population the trait's own docs name as
//! the residual trust boundary.
//!
//! # The shape
//!
//! One captured field, `payload: Fragile`, plus a second NODE so the blast
//! radius is observable: if the guard is poisoned, node B dies too, and B is
//! the node the test reads to tell "this capture failed" apart from "this
//! library is dead".
//!
//! `Fragile` encodes as a plain `u64` (8 LE bytes) — the payload is not the
//! subject; WHERE it panics is.
//!
//! # Modes (`STATE_PANIC_MODE`, read per call)
//!
//! | value | behaviour |
//! |---|---|
//! | unset / `off` | encode and decode normally — the ANTI-TAUTOLOGY control |
//! | `capture` | `cer_capture` panics after writing nothing |
//! | `restore` | `cer_restore` panics after consuming its bytes |
//!
//! Read per call, so ONE loaded library serves every arm — which matters here
//! more than usual: the point of the fixture is what happens to the library
//! AFTER the panic, so the arms must share one `NODES`.

#![deny(unused_imports)]
// Principle #12 (logging): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use cerulion_core::state::{CerulionState, StateCursor, StateError, StateShape, StateSink};
use native_ros2_messages::geometry_msgs::Quaternion;

/// The mode switch, read per call.
fn panic_mode() -> String {
    std::env::var("STATE_PANIC_MODE").unwrap_or_else(|_| "off".to_string())
}

/// A field whose `CerulionState` impl is hand-written and can be told to panic.
///
/// Hand-written on purpose: the derive cannot generate a panicking encoder, and
/// a hand-written impl is the trait's own documented trust boundary (see
/// `CerulionState::cer_read`'s "Contract for a hand-written impl").
#[derive(Default)]
pub struct Fragile {
    pub value: u64,
}

impl CerulionState for Fragile {
    const STATE_SHAPE: u64 = StateShape::of("test_node_state_panic_cdylib::Fragile")
        .field("value", <u64 as CerulionState>::STATE_SHAPE)
        .finish();

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        if panic_mode() == "capture" {
            panic!("simulated CerulionState encoder panic (capture arm)");
        }
        self.value.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let value = u64::cer_read(src)?;
        if panic_mode() == "restore" {
            panic!("simulated CerulionState decoder panic (restore arm)");
        }
        Ok(Self { value })
    }
}

#[cerulion_node(period_ms = 10)]
struct StatePanicNode {
    #[output]
    out: Quaternion,
    payload: Fragile,
}

#[cerulion_node_impl]
impl StatePanicNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.payload.value as f64;
        self.payload.value += 1;
        Ok(())
    }
}
