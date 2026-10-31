// SPDX-License-Identifier: AGPL-3.0-only
//! The scheduler-bounded frame cannot be stored past the scope it was handed
//! to, and rustc says so at the store.
//!
//! The CONTRACT: `with_committed_frame` quantifies over the frame's lifetime,
//! so a caller cannot write a type that names it. The frame is therefore
//! usable inside the closure and storable nowhere that outlives it, which is
//! what makes the borrow unable to outlive the shared-memory slot it reads.
//!
//! This fixture pins the RENDERING. It lives in `type_error/` because the
//! diagnostic is rustc's own lifetime error, which shifts between toolchains;
//! the `compile_fail` doctests on `transport::bounded_view` are the CI-gated
//! half that pins the refusal HAPPENING.

use cerulion_core::transport::bounded_view::{with_committed_frame, BoundedFrame};

fn main() {
    let frame_bytes = vec![0u8; 64];
    let mut escaped: Option<BoundedFrame<'_>> = None;
    with_committed_frame("/t", &frame_bytes, |frame| {
        escaped = Some(frame);
    });
    let _ = escaped;
}
