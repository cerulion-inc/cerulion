// SPDX-License-Identifier: AGPL-3.0-only
//! Process cleanup, resource configuration and audit inspection for caller-owned sessions.

mod inspection;
mod limits;

pub use inspection::{
    list_recordings, list_recordings_with_incomplete, verify_recording_file,
    verify_recording_prefix_file, RecordingListing, RecordingPrefixSummary,
};
pub use limits::SessionLimits;

/// Linux process identity and bounded cleanup for a caller-owned session.
#[cfg(target_os = "linux")]
pub mod processes;
