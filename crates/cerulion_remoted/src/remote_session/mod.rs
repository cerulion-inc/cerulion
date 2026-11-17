// SPDX-License-Identifier: AGPL-3.0-only
//! Resource configuration and audit inspection for authenticated sessions.

mod inspection;
mod limits;

pub(crate) const MAX_RECORDING_EVENT_BYTES: usize =
    2 * cerulion_ops_proto::session::MAX_SESSION_FRAME_BYTES;

pub use inspection::{
    list_recordings, list_recordings_with_incomplete, verify_recording_file,
    verify_recording_prefix_file, RecordingListing, RecordingPrefixSummary,
};
pub use limits::SessionLimits;
pub(crate) use limits::MAX_REAUTHORIZE_INTERVAL;
