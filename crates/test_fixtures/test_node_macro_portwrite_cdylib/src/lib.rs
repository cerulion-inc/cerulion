// SPDX-License-Identifier: AGPL-3.0-only
//! A macro cdylib that exercises the FULL port-write surface
//! through the `#[cerulion_node]` codegen ACROSS the cdylib FFI boundary.
//!
//! The port-write rewriter (variable `= expr`, variable-input read accessors,
//! `fill_from`, nested-leaf sugar, `with_<field>` closures, and the loud
//! `NestedWriteConflict` / `NestedChildIncomplete` discard paths) is heavily
//! tested IN-PROCESS by `rewriter_var_field_assign_test.rs`, but was VERIFIED
//! WIRED yet UNTESTED on the cdylib (`DylibNodeEntry`) production path. This
//! fixture is the cdylib carrier; the behavioral pins live in
//! `cerulion_core/tests/cdylib_portwrite_e2e_test.rs`.
//!
//! # Shape
//!
//! An `Image`-out node driven by a variable-schema (`std_msgs::String`) trigger
//! input, so a single tick exercises, in one place:
//!
//! - a variable READ accessor on an INPUT (`self.text_in.data()`),
//! - a variable `= expr` write (`self.image.encoding = "rgb8"`),
//! - a `fill_from` write into a variable field (`self.image.data.fill_from(..)`),
//! - nested-leaf sugar (`self.image.header.frame_id = ..` +
//!   `self.image.header.stamp.sec = ..` — a variable leaf AND a depth-2 fixed
//!   leaf through the staged view),
//! - a `with_<field>` closure resuming the staged header
//!   (`self.image.with_header(|h| { h.stamp.nanosec = ..; Ok(()) })?`),
//! - fixed-field writes (`self.image.height = ..`).
//!
//! # Fail modes (env `CER_FAIL_MODE`, cribbed from `test_node_failing_cdylib`)
//!
//! - `healthy` (default): all variable fields written → publishes one frame.
//!   The delivered payload is asserted byte-level against a hand-built
//!   standalone-`Shm` oracle AND against an in-process twin (parity).
//! - `conflict`: staged leaf sugar on `header` FOLLOWED BY a whole-field
//!   `self.image.header = ..` write → the rewriter's `NestedWriteConflict`
//!   propagates out of `tick` via `?` → tick returns Err, no frame published.
//! - `partial`: stages only the fixed grand-leaf (`header.stamp.sec`), leaving
//!   the header's lone variable field (`frame_id`) unwritten → tick returns
//!   Ok, but `OutputProxy::Drop` fails the child gate
//!   (`NestedChildIncomplete: header.frame_id`) and DISCARDS the frame.
//!
//! The env is read LIVE each tick (matching `test_node_failing_cdylib`), so the
//! host sets it before stepping; the e2e test pairs `#[serial]` with an
//! `EnvVarGuard` RAII.

#![deny(unused_imports)]
// P12 (the project logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use native_ros2_messages::std_msgs::String as RosString;

/// The `fill_from` payload — a fixed, host-known byte pattern so the oracle and
/// the in-process parity twin write the SAME bytes into `image.data`.
const FILL_BYTES: &[u8] = b"lidarframe";

/// Mode read once per tick (matches `test_node_failing_cdylib::fail_mode`).
fn fail_mode() -> String {
    std::env::var("CER_FAIL_MODE").unwrap_or_else(|_| "healthy".to_string())
}

#[cerulion_node]
struct PortWriteNode {
    /// Variable-schema TRIGGER input: drives firing AND exercises a variable
    /// READ accessor (`self.text_in.data()`) — the pw-var-read-accessor row.
    #[input(trigger)]
    text_in: RosString,

    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl PortWriteNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // pw-var-read-accessor: call the variable read accessor on the input.
        // `data()` returns `Result<&str, _>` for a String view; discarding it
        // still exercises the accessor + Deref read path. (The writes below all
        // live in THIS `#[cerulion_node_impl]` tick body — inlined, not in a
        // separate `impl` block — so the port-write rewriter actually rewrites
        // them.)
        let _ = self.text_in.data();

        match fail_mode().as_str() {
            "conflict" => {
                // pw-nested-conflict: stage a nested leaf, then write the WHOLE
                // `header` field → `NestedWriteConflict`; the rewrite's `?`
                // propagates it out of tick (no frame published).
                self.image.header.frame_id = "cam0"; // staged leaf sugar
                self.image.header = b"\x01\x02\x03"; // whole-field write → conflict via `?`
                Ok(())
            }
            "partial" => {
                // pw-nested-child-incomplete: stage ONLY the fixed grand-leaf;
                // `header.frame_id` (the header's lone variable field) stays
                // unwritten → tick returns Ok, but the Drop-path staged flush
                // fails the child gate and DISCARDS the frame.
                self.image.header.stamp.sec = 5;
                // Satisfy the TOP-LEVEL variable fields so the failure isolates
                // to the staged-child branch (not the all-variables gate).
                self.image.encoding = "rgb8";
                self.image.data = b"\x01";
                Ok(())
            }
            _ /* healthy / default — the parity twin in the e2e test mirrors
                 these EXACT writes */ =>
            {
                // pw-var-assign: a plain variable `= expr` write.
                self.image.encoding = "rgb8";
                // pw-fill-from: fill a variable field directly from a producer
                // closure (zero intermediate copy — straight into the SHM slice).
                self.image.data.fill_from(|buf: &mut [u8]| {
                    buf[..FILL_BYTES.len()].copy_from_slice(FILL_BYTES);
                    Ok(FILL_BYTES.len())
                })?;
                // pw-nested-leaf: a variable leaf + a depth-2 fixed leaf through
                // the staged header view.
                self.image.header.frame_id = "cam0";
                self.image.header.stamp.sec = 5;
                // pw-with-field: the `with_<field>` closure RESUMES the staged
                // header (no conflict — both write DIFFERENT parts of `header`).
                self.image.with_header(|h| {
                    h.stamp.nanosec = 7;
                    Ok(())
                })?;
                // Fixed fields (assignment writes straight into the loaned slot).
                self.image.height = 1080;
                self.image.width = 1920;
                Ok(())
            }
        }
    }
}
